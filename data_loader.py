import os
import math
import pickle
import numpy as np

from config import loc_radar1, loc_radar2, loc_radar3, loc_radar4
from utils.preprocess import parse_4radar_data
from utils.coord_trans import coord_rotate


class DataLoaderRadarScene:
    """
        class for load sensor data for selected radar scene data
        """
    def __init__(self, file_name='scene.pickle'):
        # load data
        with open(file_name, 'rb') as handle:
            [seq_scenes, seq_scene_keys, radar_data] = pickle.load(handle)
        self.seq_scenes = seq_scenes
        self.seq_scene_keys = seq_scene_keys
        self.radar_data = radar_data
        self.odometry_ts = [tmp[0] for tmp in self.radar_data[0]]
        self.frame_number = 0   # better to use from fourth frame they are complete

        current_id_list = []
        self.key_frames = []
        key_in_frame = []
        for key in self.seq_scene_keys:
            scenes = self.seq_scenes[key]
            radar_id = scenes['sensor_id']  # in range of 1, 2, 3, 4
            if radar_id not in current_id_list:
                key_in_frame.append(key)
                current_id_list.append(radar_id)
            else:
                self.frame_number += 1
                self.key_frames.append(key_in_frame)
                current_id_list = [radar_id]
                key_in_frame = [key]

    def load_radar_data(self, frame_id):
        px = []
        py = []
        snr = []
        vel = []
        sensor_id = []

        key_in_frame = self.key_frames[frame_id]
        for key in key_in_frame:
            scenes = self.seq_scenes[key]
            radar_indexs = scenes['radar_indices']

            for id_sel in range(radar_indexs[0], radar_indexs[1]):
                radar_data_sel = self.radar_data[1][id_sel]
                if math.sqrt(radar_data_sel[7] ** 2 + radar_data_sel[8] ** 2) < 25:     # max distance
                    px.append(radar_data_sel[7])
                    py.append(radar_data_sel[8])
                    snr.append(radar_data_sel[4])
                    vel.append(radar_data_sel[5])
                    sensor_id.append(radar_data_sel[1])

        # all data has already been transformed to the same coordinate with origin at the real axis of car
        px_front = np.asarray(px)
        py_front = np.asarray(py)
        vel_front = np.asarray(vel)
        sensor_id_front = np.asarray(sensor_id)
        snr_front = np.asarray(snr)

        return [px_front, py_front, vel_front, sensor_id_front, snr_front]

    def load_ego_motion(self, frame_id):
        key_in_frame = self.key_frames[frame_id]
        count = 0
        cur_pos_x = 0
        cur_pos_y = 0
        cur_yaw = 0
        for key in key_in_frame:
            count += 1
            index, ts = find_closet_timestamp(key, self.odometry_ts)
            cur_pos_x += self.radar_data[0][index][1]
            cur_pos_y += self.radar_data[0][index][2]
            cur_yaw += self.radar_data[0][index][3]

        cur_pos_x = cur_pos_x / count   # of car
        cur_pos_y = cur_pos_y / count   # of car
        cur_yaw = cur_yaw / count   # of car

        return cur_pos_x, cur_pos_y, cur_yaw, ts


def find_closet_timestamp(ts, ts_list):
    diff = [int(ts)-int(tmp) for tmp in ts_list]
    diff = np.abs(np.asarray(diff))
    index = np.where(diff == np.amin(diff))[0][0]

    return index, ts_list[index]


if __name__ == '__main__':
    a = DataLoaderRadarScene()
    # print(a.frame_number)
    # print(a.load_radar_data(3))
    print(a.load_ego_motion(3))
