import math
import numpy as np

from config import polygon_configs, radar_configs
from polygon.radar_polygon import RadarPolygon, inverse_sensor_recursion
from utils.coord_trans import coord_rotate

virtual_bound = polygon_configs['virtual_bound']
duration = radar_configs['duration']


def update_polygon_loc_Zadar(px_polygon, py_polygon, id_polygon, vehicle_motion, vehicle_rotate):
    # here the vehicle_motion contain the movement for 4 radars
    # avoid direct give list values - covers
    upd_pre_px_polygon = [None] * len(px_polygon)
    upd_pre_py_polygon = [None] * len(py_polygon)
    for idx, sen_id in enumerate(id_polygon):
        if sen_id is not None:
            sen_id = int(sen_id)
            if sen_id == 0 or sen_id == 2:
                upd_pre_px_polygon[idx] = px_polygon[idx] - vehicle_motion[0][0]
                upd_pre_py_polygon[idx] = py_polygon[idx] - vehicle_motion[0][1]
            elif sen_id == 1 or sen_id == 3:
                upd_pre_px_polygon[idx] = px_polygon[idx] - vehicle_motion[1][0]
                upd_pre_py_polygon[idx] = py_polygon[idx] - vehicle_motion[1][1]
        else:
            upd_pre_px_polygon[idx] = px_polygon[idx]
            upd_pre_py_polygon[idx] = py_polygon[idx]
        if vehicle_rotate != 0:
            upd_pre_px_polygon[idx], upd_pre_py_polygon[idx] = coord_rotate(
                upd_pre_px_polygon[idx], upd_pre_py_polygon[idx], -vehicle_rotate)

    return upd_pre_px_polygon, upd_pre_py_polygon


def update_polygon_loc_radarScene(px_polygon, py_polygon, id_polygon, vehicle_motion, vehicle_rotate):
    upd_pre_px_polygon = [None] * len(px_polygon)
    upd_pre_py_polygon = [None] * len(py_polygon)

    for idx, sen_id in enumerate(id_polygon):
        upd_pre_px_polygon[idx] = px_polygon[idx]
        upd_pre_py_polygon[idx] = py_polygon[idx]
        upd_pre_px_polygon[idx] = upd_pre_px_polygon[idx] - vehicle_motion[0]
        upd_pre_py_polygon[idx] = upd_pre_py_polygon[idx] - vehicle_motion[1]
        if vehicle_rotate != 0:
            # shift and rotate the coordinates for current time
            upd_pre_px_polygon[idx], upd_pre_py_polygon[idx] = coord_rotate(
                upd_pre_px_polygon[idx], upd_pre_py_polygon[idx], - vehicle_rotate)

    return upd_pre_px_polygon, upd_pre_py_polygon


class RadarPolygonISM:
    """
    class for updating radar polygon using recursive Inverse Sensor Model
    """
    def __init__(self, polygon_list, px_polygon, py_polygon, vel_polygon, id_polygon, snr_polygon, sectorId_polygon,
                 confScore_polygon, px, py, vel, sensor_id, snr, uncertain_vertices, vehicle_motion=None,
                 vehicle_rotate=0, is_radarScene=False):
        if vehicle_motion is None:
            vehicle_motion = [[0, 0], [0, 0]]
        self.pre_polygon_list = polygon_list

        if is_radarScene:
            self.upd_pre_px_polygon, self.upd_pre_py_polygon = update_polygon_loc_radarScene(
                px_polygon, py_polygon, id_polygon, vehicle_motion, vehicle_rotate)
        else:
            self.upd_pre_px_polygon, self.upd_pre_py_polygon = update_polygon_loc_Zadar(
                px_polygon, py_polygon, id_polygon, vehicle_motion, vehicle_rotate)

        self.pre_vel_polygon = vel_polygon
        self.pre_id_polygon = id_polygon
        self.pre_snr_polygon = snr_polygon
        self.pre_sectorID_polygon = sectorId_polygon
        self.pre_confScore_polygon = confScore_polygon
        self.uncertain_vertices = uncertain_vertices

        self.radar_polygon = RadarPolygon(list(px) + self.upd_pre_px_polygon, list(py) + self.upd_pre_py_polygon,
                                          list(vel) + vel_polygon, list(sensor_id) + id_polygon, list(snr) + snr_polygon)
        self.old_vertex_ids = [len(px) + i for i in range(len(self.upd_pre_px_polygon))]

    def check_preVertex_valid(self, node_id, sector_id):
        """check if the update of old vertex (same sector) is within certain range with current node candidate"""
        append_y = self.radar_polygon.py_arr[node_id]
        append_x = self.radar_polygon.px_arr[node_id]

        if node_id in self.old_vertex_ids:
            old_vertex = True
        else:
            old_vertex = False

        # find the element in old vertex list that has required sector_id
        if sector_id not in self.pre_sectorID_polygon:
            return False, old_vertex, None
        idx = self.pre_sectorID_polygon.index(sector_id)
        vertex_y = self.upd_pre_py_polygon[idx]
        vertex_x = self.upd_pre_px_polygon[idx]
        dist_diff = np.sqrt((vertex_y - append_y) ** 2 + (vertex_x - append_x) ** 2)

        if dist_diff <= polygon_configs['epsilon_2']:
            return True, old_vertex, idx
        else:
            return False, old_vertex, idx

    def calculate_polygon_ISM(self, angle_intr, origin=[0, 0], fov=[-math.pi / 2, math.pi / 2], add_virt=True,
                              is_remove_spike=False, frame_id=None):
        self.radar_polygon.fov = fov
        overlap_agl = 0  # parameter setup
        # initiate values
        append_y_last = 0
        append_x_last = 0
        num_virt = 0
        # calculate the range and azimuth angle of each point
        rng = np.sqrt((self.radar_polygon.px_arr - origin[0]) ** 2 + (self.radar_polygon.py_arr - origin[1]) ** 2)
        agl = np.arctan2(self.radar_polygon.px_arr - origin[0], self.radar_polygon.py_arr - origin[1])

        # sample azimuth direction, and find the closest-distance point
        num_search = int((fov[1] - fov[0]) / angle_intr)
        search_grid = np.linspace(fov[0], fov[1], num=num_search)

        for idx in range(num_search - 1):
            # for certain angle region
            x = np.where(((search_grid[idx] - overlap_agl) <= agl) & (agl < (search_grid[idx + 1] + overlap_agl)))
            x = np.asarray(x)[0]

            if x.size != 0:
                dist_sort = np.argsort(rng[x])
                for idx_rng in range(x.size):
                    x_id = dist_sort[idx_rng]
                    node_id = x[x_id]
                    if search_grid[idx] < math.radians(virtual_bound[0]) or search_grid[idx] > math.radians(
                            virtual_bound[1]):
                        is_within_fov = False
                    else:
                        is_within_fov = True

                    # check if the selected vertex is from history frame and if yes, get its corresponding idx in last
                    # frame
                    if node_id in self.old_vertex_ids:
                        is_old_vertex = True
                        prev_vertex_idx = node_id - self.old_vertex_ids[0]
                    else:
                        is_old_vertex = False
                        prev_vertex_idx = None

                    # previously virtual vertex / or negative confidence vertex is not allowed to be selected
                    if is_old_vertex and (self.pre_id_polygon[prev_vertex_idx] is None or
                                          self.pre_confScore_polygon[prev_vertex_idx] < 0):
                        continue

                    # check if the normalized exist prob is greater than threshold
                    flag_valid, sum_prob = self.radar_polygon.check_detection_prob(node_id, within_fov=is_within_fov)

                    if flag_valid:
                        if is_old_vertex:
                            conf = inverse_sensor_recursion(self.pre_confScore_polygon[prev_vertex_idx], sum_prob)
                            conf -= polygon_configs['conf_pen']     # penalty the confScore
                        else:
                            # if it is a new vertex, try to find it is track-able to the vertex in the last frame
                            flag_tracked, _, hist_idx = self.check_preVertex_valid(node_id, idx)
                            # if flag is yes, update the confidence score with inverse sensor model
                            if flag_tracked:
                                conf = inverse_sensor_recursion(self.pre_confScore_polygon[hist_idx], sum_prob)
                            # otherwise, init or check exist of vertex in the uncertain vertex dictionary
                            else:
                                conf = inverse_sensor_recursion(0, sum_prob)
                                if_exist, exist_conf = self.get_valid_uncertain_vertices(node_id, conf, frame_id)
                                if if_exist:
                                    # add the valid node to the polygon list
                                    num_virt, append_x_last, append_y_last = self.radar_polygon.add_polygon_node(
                                        node_id, num_virt, append_x_last, append_y_last, idx, conf_score=exist_conf)
                                break

                        # add the valid node to the polygon list
                        num_virt, append_x_last, append_y_last = self.radar_polygon.add_polygon_node(node_id,
                            num_virt, append_x_last, append_y_last, idx, conf_score=conf)
                        break

            elif add_virt:
                # add one far point as a virtual node
                virt_agl = search_grid[idx]
                num_virt = self.radar_polygon.add_virtual_node(virt_agl, num_virt, sector_id=idx)

        # add virtual nodes on two-side boundaries
        num_virt = self.radar_polygon.add_virtual_node(fov[1], num_virt)
        _ = self.radar_polygon.add_virtual_node(fov[0], num_virt)

        # do the spike removal
        if is_remove_spike:
            self.radar_polygon.remove_spike()

        return self.radar_polygon.poly_list, search_grid, self.radar_polygon.px_polygon, self.radar_polygon.py_polygon,\
               self.radar_polygon.vel_polygon, self.radar_polygon.sensid_polygon, self.radar_polygon.snr_polygon, \
               self.radar_polygon.sectorId_polygon, self.radar_polygon.confScore_polygon, self.uncertain_vertices

    def get_valid_uncertain_vertices(self, node_id, conf, frame_id):
        cur_len = len(self.uncertain_vertices.loc_x)
        loc_x = self.radar_polygon.px_arr[node_id]
        loc_y = self.radar_polygon.py_arr[node_id]
        vel = self.radar_polygon.vel_arr[node_id]
        radar_id = self.radar_polygon.id_arr[node_id]
        if cur_len < 1:
            # add the vertex
            self.uncertain_vertices.addNode(loc_x, loc_y, vel, radar_id, conf, frame_id)
        else:
            # check if there is uncertain vertex associated with the node_id one
            dist = np.sqrt((loc_x - np.asarray(self.uncertain_vertices.loc_x)) ** 2 +
                           (loc_y - np.asarray(self.uncertain_vertices.loc_y)) ** 2)
            index = np.argmin(dist)
            if dist[index] <= polygon_configs['epsilon_2']:
                # if yes, this is valid one, output it and associate it with the old one (remove the old from dict)
                self.uncertain_vertices.cons_age[index] += 1
                self.uncertain_vertices.latest[index] = frame_id
                self.uncertain_vertices.confScore[index] += conf
                if self.uncertain_vertices.cons_age[index] >= polygon_configs['min_age']:
                    old_conf = self.uncertain_vertices.confScore[index]
                    self.uncertain_vertices.removeNode(index)
                    return True, old_conf
            else:
                # if no, add this one to dict
                self.uncertain_vertices.addNode(loc_x, loc_y, vel, radar_id, conf, frame_id)

        return False, None
