import time
import math
import numpy as np

from config import snr_elevation, init_radar_len
from data_loader import DataLoaderRadarScene
from utils.visualize import visualize_gridmap
from utils.save_result import save_gridmap_results
from utils.coord_trans import coord_rotate
from polygon_radarscene_inverseSensor import convert_carEgo2radarEgo


def inverse_sensor_recursion_array(pre_conf, det_prob):
    conf = pre_conf + np.log(np.divide(det_prob, (1 - det_prob))) - 0
    return conf


def Gauss2d_kernel(index_x, index_y, size, sigma=0.5):
    """
    generate 2d gaussian occupancy region
    """
    ax = np.linspace(-index_x, -index_x + size[0] - 1, size[0])
    ay = np.linspace(-index_y, -index_y + size[1] - 1, size[1])
    gauss_x = np.exp(-0.5 * np.square(ax) / np.square(sigma))
    gauss_y = np.exp(-0.5 * np.square(ay) / np.square(sigma))
    kernel = np.outer(gauss_x, gauss_y)

    return kernel


def sigmoid(x):
    return 1 / (1 + math.exp(-x))


class GridMapIsm:
    """
    class for forming radar polygon using single frame measurement
    """

    def __init__(self, px, py, vel, sensor_id, snr, map_resu, map_range, map_size, map_start_index):
        self.px_arr = np.asarray(px)
        self.py_arr = np.asarray(py)
        self.vel_arr = np.asarray(vel)
        self.id_arr = np.asarray(sensor_id)
        self.snr_arr = np.asarray(snr)
        self.map_resolution = map_resu  # meters
        self.map_range = map_range  # meters
        self.map_size = map_size
        self.map_start_index = map_start_index

    def loc2index(self, loc_x, loc_y):
        """
        transform the loc in car-center coordinates to index in grid map
        :param map_size:
        :param map_resolution:
        :return:
        """
        idx_x = int(loc_x / self.map_resolution) + self.map_start_index[0]
        idx_y = int(loc_y / self.map_resolution) + self.map_start_index[1]

        if -1 < idx_x < self.map_size[0] and -1 < idx_y < self.map_size[1]:
            flag_valid = True
        else:
            flag_valid = False

        return int(idx_x), int(idx_y), flag_valid

    def index2loc(self, idx_x, idx_y):
        loc_x = (idx_x - self.map_start_index[0]) * self.map_resolution
        loc_y = (idx_y - self.map_start_index[1]) * self.map_resolution

        return loc_x, loc_y

    def project_measures2startCoord(self, loc_x, loc_y, rotate):
        self.px_arr, self.py_arr = coord_rotate(self.px_arr + loc_x, self.py_arr + loc_y, -rotate)

    def inverse_sensor_accumulation_Werbe(self, grid_maps, r_scale=0.02, a_scale=0.05, a_off=0):
        """
        Werbe methods:
        :param a_off:
        :param a_scale:
        :param r_scale:
        :param grid_maps: (N, M)
        :return: updated grid_maps using inverse sensor models
        """
        rng = np.sqrt(self.px_arr ** 2 + self.py_arr ** 2)
        local_plaus_all = np.zeros(self.map_size)
        for idx in range(len(self.px_arr)):
            index_x, index_y, flag_valid = self.loc2index(self.px_arr[idx], self.py_arr[idx])
            if not flag_valid:
                continue
            # calculate plausibility
            p_rng = math.exp(-r_scale * rng[idx] ** 2)
            p_a = 1 - 1 / (1 + math.exp(-a_scale * (math.sqrt(self.snr_arr[idx] / (rng[idx] ** 4)) + a_off)))
            local_plaus = (p_rng + p_a) * Gauss2d_kernel(index_x, index_y, self.map_size)
            local_plaus_all += local_plaus

        # update grid map with ISM model
        local_plaus_all = np.clip(local_plaus_all, 0.5, 1-1e-8)
        grid_maps = inverse_sensor_recursion_array(grid_maps, local_plaus_all)

        return grid_maps

    def inverse_sensor_accumulation_Li(self, grid_maps, d_ref=30):
        """
        Le methods
        :param d_ref: reference distance parameter
        :param grid_maps:
        :param measurements:
        :return:
        """
        rng = np.sqrt(self.px_arr ** 2 + self.py_arr ** 2)
        local_plaus_all = np.zeros(self.map_size)
        for idx in range(len(self.px_arr)):
            index_x, index_y, flag_valid = self.loc2index(self.px_arr[idx], self.py_arr[idx])
            if not flag_valid:
                continue
            # calculate plausibility
            A = math.sqrt(self.snr_arr[idx] / (rng[idx] ** 4))
            A = A - 40 * math.log10(rng[idx] / d_ref)
            A = sigmoid(A)      # normalization
            local_plaus = A * Gauss2d_kernel(index_x, index_y, self.map_size, sigma=0.1)
            local_plaus_all += local_plaus

        # update grid map with ISM model
        local_plaus_all = np.clip(local_plaus_all, 0.5, 1 - 1e-8)
        grid_maps = inverse_sensor_recursion_array(grid_maps, local_plaus_all)

        return grid_maps


def main(is_save_gridmap=True, is_save_image=True, is_werbe=False):
    # print('***************************start grid map ISM*******************************')
    if is_werbe:
        save_txt_folder_name = 'radarScene143_ism_werbe'
        save_img_folder_name = 'radarScene143_ism_werbe'
    else:
        save_txt_folder_name = 'radarScene143_ism_Li'
        save_img_folder_name = 'radarScene143_ism_Li'
    data_loader = DataLoaderRadarScene()

    map_resu = 0.3  # meters
    map_range = [60, 60]  # meters, [x, y]
    map_size = [int(map_range[0] / map_resu), int(map_range[1] / map_resu)]
    map_start_index = [120, 120]
    process_frames = [3, 36, 64, 136, 144, 199, 244, 286, 335, 365]

    time_total = 0
    runs_total = 0
    for frame_id in process_frames:
        start_id_file = frame_id
        stop_id_file = 2
        for id_file in range(start_id_file, stop_id_file, -1):
            data_front = data_loader.load_radar_data(id_file)
            cur_pos_x, cur_pos_y, cur_yaw, ts = data_loader.load_ego_motion(id_file)
            data_front[4] = np.maximum(data_front[4] + snr_elevation, 0)  # boost and clip snr
            data_front[1] = - data_front[1]  # flip the direction for y
            data_front[0] = data_front[0] - init_radar_len
            GridMap = GridMapIsm(data_front[1], data_front[0], data_front[2], data_front[3], data_front[4],
                                 map_resu, map_range, map_size, map_start_index)

            if id_file == 0 or id_file == start_id_file:
                start = time.time()  # starting time
                # generate new grid maps, and the origin (center of car) corresponds to the start point in map
                maps = np.zeros((map_size[0], map_size[1]), dtype=float)
                if is_werbe:
                    # inverse sensor update
                    maps = GridMap.inverse_sensor_accumulation_Werbe(maps)
                else:
                    maps = GridMap.inverse_sensor_accumulation_Li(maps)
                end = time.time()  # end time
                history_ego = [cur_pos_x, cur_pos_y, cur_yaw, ts]
            else:
                radar_motion, ego_rotate, _ = convert_carEgo2radarEgo(cur_pos_x, cur_pos_y, cur_yaw, ts, history_ego)
                start = time.time()  # starting time
                # project the measurements to the coordinates of origin (starting point)
                GridMap.project_measures2startCoord(radar_motion[0], radar_motion[1], ego_rotate)
                # inverse sensor update
                if is_werbe:
                    maps = GridMap.inverse_sensor_accumulation_Werbe(maps)
                else:
                    maps = GridMap.inverse_sensor_accumulation_Li(maps)
                end = time.time()  # end time

            runs_total += 1
            time_total += (end - start)  # total time taken
        # save_results to pickle file for further evaluation
        if is_save_gridmap:
            save_gridmap_results(save_txt_folder_name, str(frame_id).zfill(4)+'.png', maps)

        # plot the radar polygon on bev image
        if is_save_image:
            visualize_gridmap(save_img_folder_name, str(frame_id).zfill(4)+'.png', maps)

    print(f"Runtime of the program is {time_total / runs_total}")
    # print('***************************end polygon formation for single frame*******************************')


if __name__ == '__main__':
    is_werbe = False
    main(is_werbe=is_werbe)
