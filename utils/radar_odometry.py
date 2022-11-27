# set parent directory as sys path
import inspect
import os
import sys
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

import pickle
import math
import numpy as np
from data_loader import DataLoader4Sensor
from sklearn import linear_model
from config import radar_configs, loc_radar1, loc_radar2, loc_radar3, loc_radar4
from utils.coord_trans import coord_rotate
from utils.timestamp import match_timestamp
from utils.odometry_test import savitzky_golay

loc_radar = [loc_radar1, loc_radar2, loc_radar3, loc_radar4]


"""
estimate the pose information of vehicle
"""
if __name__ == '__main__':
    save_folder_name = 'car_moving4_pred'
    root_dir = 'C:/Users/Xiangyu Gao/Desktop/volvo intern/polymol_radar_tracking/polymol_radar_tracking/aug_05_dynamic'
    save_name = 'aug_05_dynamic_pose_axis'  # vx, vy, yaw rate of rear axis center
    save_name2 = 'aug_05_dynamic_pose'      # vx, vy of 4 sensors
    save_name3 = 'aug_05_dynamic_pose_ego'  # accumulated rotation angle and instaneous localization of rear axis center
    data_loader = DataLoader4Sensor(root_dir, is_separate4=True)
    save_data = []
    save_data2 = []
    yaw = []
    rotate_angles = [abs(loc_radar1[2]) + 270, 90 - abs(loc_radar2[2]), abs(loc_radar3[2]) + 270,
                     90 - abs(loc_radar4[2])]
    dist2rearAxis = [[loc_radar1[0], 3.45], [loc_radar2[0], 3.45], [-loc_radar3[0], -1.19], [-loc_radar4[0], -1.19]]
    S_matx_sign = [[1, -1], [-1, 1], [1, -1], [-1, 1]]

    for id_file in range(data_loader.num_file):
        datas = data_loader.load_radar_data(id_file)
        file_name = datas[-1]
        bev_name = match_timestamp(data_loader.bev_files, file_name[:-4])   # match the radar and bev image
        pose_sensors = []
        for i in range(4):
            # for each sensor
            py = datas[i][0]    # indeed is py from dataLoader
            px = datas[i][1]    # indeed is px from dataLoader
            vel = datas[i][2]
            rng = np.sqrt(py ** 2 + px ** 2)
            cos_theta = np.divide(py, rng)
            sin_theta = np.divide(px, rng)
            angle_matx = np.concatenate((sin_theta[:, np.newaxis], cos_theta[:, np.newaxis]), 1)

            # Robustly fit linear model with RANSAC algorithm
            ransac = linear_model.RANSACRegressor(residual_threshold=1)
            ransac.fit(angle_matx, vel)
            inlier_mask = ransac.inlier_mask_

            # linear regression to solve ego motions
            regressor = linear_model.LinearRegression(fit_intercept=False)
            regressor.fit(angle_matx, vel)
            pose_sensors.append(-regressor.coef_)

        # project radar velocities to same vx vy coordinates
        # estimate the yaw rate and project the calculated velocity to the same plane
        S_matx = []
        V_matx = []
        for i2 in range(4):
            radar_vel = pose_sensors[i2]
            vx, vy = coord_rotate(radar_vel[0], radar_vel[1], math.radians(rotate_angles[i2]))
            if i2 > 1:
                vx = -vx
                vy = -vy
            S_matx.append([S_matx_sign[i2][0]*dist2rearAxis[i2][1], 1, 0])
            S_matx.append([S_matx_sign[i2][1]*dist2rearAxis[i2][0], 0, 1])
            V_matx.append([vy])
            V_matx.append([vx])

        X_matx = np.linalg.inv(np.transpose(S_matx) @ S_matx) @ np.transpose(S_matx) @ V_matx
        pose_axis = [X_matx[2], X_matx[1], X_matx[0], bev_name]   # vx, vy, yaw rate
        save_data2.append(pose_sensors)
        save_data.append(pose_axis)
        yaw.append(pose_axis[2])
        print('finished ', id_file)

    # calculate the accumulated rotation (yaw)
    rotation_save = []  # rear axis rotation
    loc_axis = [[], []]     # localization of rear axis
    rotation = 0
    loc_x = 0
    loc_y = 0
    yaw_lps = savitzky_golay(np.squeeze(np.asarray(yaw)), 21, 4)    # window size 51, polynomial order 3
    for idx in range(len(save_data)):
        ego_pose = save_data[idx]
        vx = ego_pose[0]
        vy = ego_pose[1]
        vx_proj, vy_proj = coord_rotate(vx, vy, rotation)
        loc_x += vx_proj * radar_configs['duration']
        loc_y += vy_proj * radar_configs['duration']
        loc_axis[0].append(loc_x)
        loc_axis[1].append(loc_y)
        rotation_save.append(rotation)
        rotation += yaw_lps[idx] * radar_configs['duration'] * 2.25

    with open(save_name, "wb") as fp:  # Pickling
        pickle.dump(save_data, fp)
    fp.close()
    with open(save_name2, "wb") as fs:  # Pickling
        pickle.dump(save_data2, fs)
    fs.close()
    with open(save_name3, "wb") as fe:  # Pickling
        pickle.dump([rotation_save, loc_axis], fe)
    fe.close()
