import time
import numpy as np

from config import snr_elevation, init_radar_len
from dataLoader.data_loader import DataLoaderRadarScene
from utils.visualize import visualize_gridmap, visualize_polygon_radarScene
from utils.save_result import save_results_radarScene
from polygon_radarscene_inverseSensor import convert_carEgo2radarEgo
from gridMap.GridMapIsm import GridMapIsm
from polygon.meerpohl_polygon import MeerpohlPolygon


def main(is_save_polygon=True, is_save_image=True):
    print('***************************radar scene data*******************************')
    print('***************************start polygon formation for Meerpohl*******************************')
    save_txt_folder_name = 'radarScene143_meerpohl'
    save_img_folder_name = 'radarScene143_meerpohl'
    data_loader = DataLoaderRadarScene()

    map_resu = 0.3  # meters
    map_range = [60, 60]  # meters, [x, y]
    map_size = [int(map_range[0] / map_resu), int(map_range[1] / map_resu)]
    map_start_index = [120, 120]

    time_total = 0
    runs_total = 0
    start_id = 3
    for id_file in range(start_id, data_loader.frame_number):
        data_front = data_loader.load_radar_data(id_file)
        cur_pos_x, cur_pos_y, cur_yaw, ts = data_loader.load_ego_motion(id_file)
        data_front[4] = np.maximum(data_front[4] + snr_elevation, 0)  # boost and clip snr
        data_front[1] = - data_front[1]  # flip the direction for y
        data_front[0] = data_front[0] - init_radar_len

        # initialize a grid map
        GridMap = GridMapIsm(data_front[1], data_front[0], data_front[2], data_front[3], data_front[4],
                             map_resu, map_range, map_size, map_start_index)
        # initialize the Meerpohl Polygon class
        polygon = MeerpohlPolygon(rC=30, rHC=2, nPts=180, tau=20, gridMapClass=GridMap)

        # initialize an empty grid map for the first frame or every 30 frames
        if id_file == 0 or id_file == start_id or (id_file - start_id) % 30 == 0:
            start = time.time()  # starting time
            # generate new grid maps, and the origin (center of car) corresponds to the start point in map
            maps = np.zeros((map_size[0], map_size[1]), dtype=float)
            # inverse sensor update
            maps = GridMap.inverse_sensor_accumulation_Werbe(maps)

            # generate Meerpohl polygon
            px_polygon, py_polygon, px_polygon_ind, py_polygon_ind, conf_polygon = polygon.create_polygon_on_grid(maps)

            end = time.time()  # end time
            history_ego = [cur_pos_x, cur_pos_y, cur_yaw, ts]
        else:
            radar_motion, ego_rotate, _ = convert_carEgo2radarEgo(cur_pos_x, cur_pos_y, cur_yaw, ts, history_ego)
            start = time.time()  # starting time
            # project the measurements to the coordinates of origin (starting point)
            GridMap.project_measures2startCoord(radar_motion[0], radar_motion[1], ego_rotate)
            # inverse sensor update
            maps = GridMap.inverse_sensor_accumulation_Werbe(maps)

            # convert the grid map to the current coordinate
            newMaps = GridMap.project_map2currentCoord(maps, radar_motion[0], radar_motion[1], ego_rotate)
            # generate the Meerpohl polygon
            px_polygon, py_polygon, px_polygon_ind, py_polygon_ind, conf_polygon = polygon.create_polygon_on_grid(newMaps)

            end = time.time()  # end time

        runs_total += 1
        time_total += (end - start)  # total time taken

        # save_results to pickle file for further evaluation
        if is_save_polygon:
            save_results_radarScene(save_txt_folder_name, id_file, px_polygon, py_polygon, conf_polygon)
        # plot the radar polygon on bev image
        if is_save_image:
            # visualize_gridmap(save_img_folder_name, str(id_file).zfill(4)+'.png', maps)
            visualize_polygon_radarScene(data_front[1], data_front[0], px_polygon, py_polygon, conf_polygon,
                                         save_img_folder_name, id_file)

        print('Finished', id_file)

    print(f"Runtime of the program is {time_total / runs_total}")
    # print('***************************end polygon formation for single frame*******************************')


if __name__ == '__main__':
    main()
