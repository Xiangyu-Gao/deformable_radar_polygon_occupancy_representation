import os
import numpy as np

from utils.pypcd import PointCloud
from config import polygon_configs
from utils.timestamp import match_timestamp


def parse_point_cloud(radar_dir, file_name, ses_id):
    if file_name is None:
        return np.asarray([]), np.asarray([]), np.asarray([]), np.asarray([]), np.asarray([])

    path = os.path.join(radar_dir, file_name)
    pc = PointCloud.from_path(path)
    new_cloud_data = pc.pc_data.view(np.float32).reshape(pc.pc_data.shape + (-1,))
    pz = new_cloud_data[:, 2]

    # remove the points with height exceeding threshold
    px = new_cloud_data[(pz <= polygon_configs['height_thres']) & (pz > polygon_configs['height_thres_low']), 0]
    py = - new_cloud_data[(pz <= polygon_configs['height_thres']) & (pz > polygon_configs['height_thres_low']), 1]
    vel = new_cloud_data[(pz <= polygon_configs['height_thres']) & (pz > polygon_configs['height_thres_low']), 4]
    snr = new_cloud_data[(pz <= polygon_configs['height_thres']) & (pz > polygon_configs['height_thres_low']), 3]
    sensor_id = np.ones_like(px) * ses_id

    return px, py, pz, vel, sensor_id, snr


def parse_4radar_data(dirs, files, file_id, num_file):
    len_list = [len(file) for file in files]
    radar_id = len_list.index(num_file)
    file_name = files[radar_id][file_id]
    data_all = []

    for idx in range(len(files)):
        if idx == radar_id:
            px, py, pz, vel, sensor_id, snr = parse_point_cloud(dirs[radar_id], file_name, radar_id)
        else:
            new_file_name = match_timestamp(files[idx], file_name[:-4], max_offset=100)
            px, py, pz, vel, sensor_id, snr = parse_point_cloud(dirs[idx], new_file_name, idx)
        data_all.append([px, py, pz, vel, sensor_id, snr])

    return data_all, file_name
