import math
import numpy as np

from config import polygon_configs
from utils.calc_boundary import calc_boundary

virtual_bound = polygon_configs['virtual_bound']


class RadarPolygon:
    """
    class for forming radar polygon using single frame measurement
    """
    def __init__(self, px, py, vel, sensor_id, snr):
        self.px_arr = np.asarray(px)
        self.py_arr = np.asarray(py)
        self.vel_arr = np.asarray(vel)
        self.id_arr = np.asarray(sensor_id)
        self.snr_arr = np.asarray(snr)
        self.use_sign = np.ones_like(self.px_arr)  # used_id
        self.poly_list = []
        self.px_polygon = []
        self.py_polygon = []
        self.vel_polygon = []
        self.sensid_polygon = []
        self.snr_polygon = []
        self.sectorId_polygon = []
        self.confScore_polygon = []
        self.dis_rm_virt = polygon_configs['dis_rm_virt']  # distance threshold for two near-by polygon nodes
        self.search_grid = []
        self.fov = []

    def add_polygon_node(self, node_id, num_virt, append_x_last, append_y_last, sector_id, conf_score=0):
        append_y = self.py_arr[node_id]
        append_x = self.px_arr[node_id]
        append_v = self.vel_arr[node_id]
        append_id = self.id_arr[node_id]
        append_snr = self.snr_arr[node_id]

        # each point cannot be used twice
        if self.use_sign[node_id] != 1:
            return num_virt, append_x_last, append_y_last
        # check if the cross-distance between appended node and last node is smaller than a threshold
        cross_distance = calc_cross_dist(append_x_last, append_y_last, append_x, append_y)
        # remove the virtual points between them
        if num_virt != 0 and cross_distance < self.dis_rm_virt:
            self.poly_list[-num_virt:] = []
            self.py_polygon[-num_virt:] = []
            self.px_polygon[-num_virt:] = []
            self.vel_polygon[-num_virt:] = []
            self.sensid_polygon[-num_virt:] = []
            self.snr_polygon[-num_virt:] = []
            self.sectorId_polygon[-num_virt:] = []
            self.confScore_polygon[-num_virt:] = []
        # reset num_virt
        num_virt = 0

        # append the new polygon node
        self.poly_list.append(node_id)
        self.py_polygon.append(append_y)
        self.px_polygon.append(append_x)
        self.vel_polygon.append(append_v)
        self.sensid_polygon.append(append_id)
        self.snr_polygon.append(append_snr)
        self.sectorId_polygon.append(sector_id)
        self.confScore_polygon.append(conf_score)
        self.use_sign[node_id] = 0

        # update the last non-virtual node
        append_y_last = append_y
        append_x_last = append_x

        return num_virt, append_x_last, append_y_last

    def add_virtual_node(self, virt_agl, num_virt, sector_id=-1, is_bound=False):
        # add a virtual point at the boundary for the overlapping regions
        if ((self.fov[0] < virt_agl < math.radians(virtual_bound[0])) or (math.radians(virtual_bound[1]) < virt_agl <
                                                                          self.fov[1])) and not is_bound:
            return num_virt
        self.poly_list.append(-1)
        px, py = calc_boundary(virt_agl, self.fov)
        self.py_polygon.append(py)
        self.px_polygon.append(px)
        self.vel_polygon.append(0)
        self.sensid_polygon.append(None)
        self.snr_polygon.append(0)
        self.sectorId_polygon.append(sector_id)
        self.confScore_polygon.append(0)
        num_virt += 1

        return num_virt

    def remove_spike(self):
        """remove spikes according to the slopes"""
        angle_thres = polygon_configs['spike_thres_deg']
        is_not_spike = [0]

        # for each polygon point, calculate its slope with neighbouring points
        for idx in range(1, len(self.poly_list) - 1):
            d1 = math.sqrt(self.py_polygon[idx] ** 2 + self.px_polygon[idx] ** 2)
            d2 = math.sqrt(self.py_polygon[idx + 1] ** 2 + self.px_polygon[idx + 1] ** 2)
            d3 = math.sqrt(self.py_polygon[idx - 1] ** 2 + self.px_polygon[idx - 1] ** 2)
            angle1 = math.atan2(self.py_polygon[idx] - self.py_polygon[idx - 1],
                                self.px_polygon[idx] - self.px_polygon[idx - 1])
            angle2 = math.atan2(self.py_polygon[idx] - self.py_polygon[idx + 1],
                                self.px_polygon[idx] - self.px_polygon[idx + 1])
            mid_angle = abs(angle1 - angle2)
            if math.degrees(mid_angle) < angle_thres and d1 > max(d2, d3):
                continue
            is_not_spike.append(idx)

        is_not_spike.append(len(self.poly_list) - 1)
        # remove all spikes:
        self.poly_list = [self.poly_list[i] for i in is_not_spike]
        self.px_polygon = [self.px_polygon[i] for i in is_not_spike]
        self.py_polygon = [self.py_polygon[i] for i in is_not_spike]
        self.vel_polygon = [self.vel_polygon[i] for i in is_not_spike]
        self.sensid_polygon = [self.sensid_polygon[i] for i in is_not_spike]
        self.snr_polygon = [self.snr_polygon[i] for i in is_not_spike]
        self.sectorId_polygon = [self.sectorId_polygon[i] for i in is_not_spike]
        self.confScore_polygon = [self.confScore_polygon[i] for i in is_not_spike]

    def accum_detection_prob(self, node_id):
        append_y = self.py_arr[node_id]
        append_x = self.px_arr[node_id]
        rng_diff = np.sqrt((self.px_arr - append_x) ** 2 + (self.py_arr - append_y) ** 2)
        neighbors = np.asarray(np.where(rng_diff <= polygon_configs['epsilon_1']))[0]
        sum_prob = 0
        for idx in range(neighbors.size):
            prob = calc_detection_prob(self.snr_arr[neighbors[idx]], rng_diff[neighbors[idx]])
            sum_prob += prob

        return sum_prob

    def check_detection_prob(self, node_id, within_fov=True):
        sum_prob = self.accum_detection_prob(node_id)
        sum_prob_norm = (sum_prob - polygon_configs['prob_mean']) / polygon_configs['prob_var']
        sum_prob_sigmoid = 0.5 + 0.5 / (1 + math.exp(-sum_prob_norm))

        if within_fov:
            if sum_prob_sigmoid >= polygon_configs['sum_prob_thres']:
                return True, sum_prob_sigmoid
            else:
                return False, sum_prob_sigmoid
        else:
            # when current agl is out of the radar measurement (single-frame) fov, relax the check threshold
            # (as long as not virtual element at boundary)
            if abs(self.px_arr[node_id]) > 1:
                return True, sum_prob_sigmoid
            else:
                return False, sum_prob_sigmoid

    def analyze_sum_prob_distrb(self):
        # calculate the range and azimuth angle of each point
        sum_prob_list = []
        for node_id in range(self.px_arr.size):
            sum_prob = self.accum_detection_prob(node_id)
            sum_prob_list.append(sum_prob)
        # analyze the mean and distribution of sum_prob
        mean_ = np.mean(np.asarray(sum_prob_list))
        var_ = math.sqrt(np.var(np.asarray(sum_prob_list)))

        return mean_, var_

    def calculate_polygon(self, angle_intr, origin=[0, 0], fov=[-math.pi/2, math.pi/2], add_virt=True, is_remove_spike=False):
        self.fov = fov
        overlap_agl = 0  # parameter setup
        # initiate values
        append_y_last = 0
        append_x_last = 0
        num_virt = 0
        # calculate the range and azimuth angle of each point
        rng = np.sqrt((self.px_arr - origin[0]) ** 2 + (self.py_arr - origin[1]) ** 2)
        agl = np.arctan2(self.px_arr - origin[0], self.py_arr - origin[1])

        # sample azimuth direction, and find the closest-distance point
        num_search = int((fov[1] - fov[0]) / angle_intr)
        search_grid = np.linspace(fov[0], fov[1], num=num_search)
        self.search_grid = search_grid

        for idx in range(num_search - 1):
            # for certain angle region
            x = np.where(((search_grid[idx] - overlap_agl) <= agl) & (agl < (search_grid[idx + 1] + overlap_agl)))
            x = np.asarray(x)[0]
            if x.size != 0:
                dist_sort = np.argsort(rng[x])
                for idx_rng in range(x.size):
                    x_id = dist_sort[idx_rng]
                    node_id = x[x_id]
                    # check if the accumulated detection prob is greater than epsilon_1
                    flag_valid, sum_prob = self.check_detection_prob(node_id)
                    if flag_valid:
                        # add the valid node to the polygon list
                        conf = inverse_sensor_recursion(0, sum_prob)
                        num_virt, append_x_last, append_y_last = self.add_polygon_node(node_id, num_virt, append_x_last,
                            append_y_last, idx, conf_score=conf)
                        break
            elif add_virt:
                # add one far point as a virtual node
                virt_agl = search_grid[idx]
                num_virt = self.add_virtual_node(virt_agl, num_virt, sector_id=idx)

        # add virtual nodes on two-side boundaries
        num_virt = self.add_virtual_node(fov[1], num_virt)
        _ = self.add_virtual_node(fov[0], num_virt)
        # remove spikes in polygon
        if is_remove_spike:
            self.remove_spike()

        return self.poly_list, search_grid, self.px_polygon, self.py_polygon, self.vel_polygon, self.sensid_polygon, \
               self.snr_polygon, self.sectorId_polygon, self.confScore_polygon


def calc_cross_dist(append_x_last, append_y_last, append_x, append_y):
    cross_dist = max(math.sqrt(append_y_last ** 2 + append_x_last ** 2),
                     math.sqrt(append_y ** 2 + append_x ** 2)
                     ) * abs(math.atan2(append_y, append_x) - math.atan2(append_y_last, append_x_last))

    return cross_dist


def calc_detection_prob(snr, dist_diff):
    prob = polygon_configs['pfa'] ** (1.0 / (1.0 + snr))
    prob_gaussian = prob / (2 * math.pi * polygon_configs['gauss_var'] ** 2) * math.exp(
        - 0.5 * dist_diff ** 2 / polygon_configs['gauss_var'] ** 2)

    return prob_gaussian


def inverse_sensor_recursion(pre_conf, det_prob):
    eps = 1e-5
    conf = pre_conf + math.log(det_prob / (1 - det_prob + eps)) - 0

    return conf
