import math
import numpy as np

from utils.coord_trans import coord_rotate


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
        :param loc_x:
        :param loc_y:
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
        :param grid_maps:
        :param d_ref: reference distance parameter
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
            A = sigmoid(A)  # normalization
            local_plaus = A * Gauss2d_kernel(index_x, index_y, self.map_size, sigma=0.1)
            local_plaus_all += local_plaus

        # update grid map with ISM model
        local_plaus_all = np.clip(local_plaus_all, 0.5, 1 - 1e-8)
        grid_maps = inverse_sensor_recursion_array(grid_maps, local_plaus_all)

        return grid_maps

    def project_map2currentCoord(self, grid_maps, center_x, center_y, rotate):
        """
        project the grid map to the current coordinate
        :param center_y:
        :param center_x:
        :param grid_maps:
        :param rotate:
        :return:
        """
        # create a new grid map centered at current coordinate of radar
        newMaps = np.zeros((self.map_size[0], self.map_size[1]), dtype=float)
        # get the location of each cell
        for idx in range(self.map_size[0]):
            for idy in range(self.map_size[1]):
                loc_x, loc_y = self.index2loc(idx, idy)
                # convert loc_x, loc_y to the start coordinate
                loc_x_startIndex, loc_y_startIndex = coord_rotate(loc_x + center_x, loc_y + center_y, -rotate)
                idx_x_startIndex, idx_y_startIndex, flag_valid = self.loc2index(loc_x_startIndex, loc_y_startIndex)
                if flag_valid:
                    newMaps[idx, idy] = grid_maps[idx_x_startIndex, idx_y_startIndex]

        return newMaps
