import time
import math
import numpy as np

from dataLoader.data_loader import DataLoaderRadarScene
from polygon.radar_polygon import RadarPolygon
from polygon.radar_polygon_ISM import RadarPolygonISM
from polygon.polygon_predict import polygon_predict
from polygon.vertexNodes import vertexNodes
from utils.visualize import visualize_polygon_radarScene, visualize_polygon_radarScene_withpred
from utils.save_result import save_results_radarScene
from config import polygon_configs, snr_elevation, init_radar_len, radar_configs
from utils.coord_trans import coord_rotate


def convert_carEgo2radarEgo(car_pos_x, car_pos_y, car_agl, ts, history_ego=None, is_in_frame=False):
    if history_ego is None:
        return [0, 0], 0, 0
    else:
        rotate = car_agl - history_ego[2]   # in radarScene coordinate
        car_ego_x = car_pos_x - history_ego[0]
        car_ego_y = car_pos_y - history_ego[1]
        car_ego_x, car_ego_y = coord_rotate(car_ego_x, car_ego_y, 1.5 * math.pi + history_ego[2])
        dt = (int(ts) - int(history_ego[-1])) * 1e-6
        v_results = [car_ego_x, car_ego_y]

        if is_in_frame:
            return v_results, rotate, dt / radar_configs['duration']
        else:
            return v_results, rotate, dt


def main(is_save_polygon=True, is_save_image=True, is_withpred=False):
    print('***************************radar scene data*******************************')
    print('***************************start polygon formation for ISM*******************************')
    save_txt_folder_name = 'radarScene143_ism_poly'
    save_img_folder_name = 'radarScene143_ism_poly'
    data_loader = DataLoaderRadarScene()
    sampling_azi = polygon_configs['sampling_reg']  # degree
    time_total = 0
    process_frame = 0
    init_fov = [-math.radians(140), math.radians(140)]
    ism_fov = [-math.radians(175), math.radians(175)]
    uncertain_vertices = vertexNodes()
    py_polygon_pred = []
    px_polygon_pred = []
    dt_frame = 1

    start_id = 3
    stride = 1
    for id_file in range(start_id, data_loader.frame_number // stride):
        data_front = data_loader.load_radar_data(id_file * stride)
        cur_pos_x, cur_pos_y, cur_yaw, ts = data_loader.load_ego_motion(id_file * stride)
        data_front[4] = np.maximum(data_front[4] + snr_elevation, 0)  # boost and clip snr
        data_front[1] = - data_front[1]  # flip the direction for y
        data_front[0] = data_front[0] - init_radar_len

        if id_file == start_id:
            radar_polygon1 = RadarPolygon(data_front[1], data_front[0], data_front[2], data_front[3], data_front[4])  # switch the x, y direction
            start = time.time()  # starting time
            polygon_list, _, py_polygon, px_polygon, vel_polygon, id_polygon, snr_polygon, sectorId_polygon, \
                conf_polygon = radar_polygon1.calculate_polygon(math.radians(sampling_azi), fov=init_fov,
                                                                is_remove_spike=True)
            end = time.time()  # end time
            history_ego = [cur_pos_x, cur_pos_y, cur_yaw, ts]
        else:
            radar_motion, ego_rotate, dt_frame = convert_carEgo2radarEgo(cur_pos_x, cur_pos_y, cur_yaw, ts, history_ego,
                                                                         is_in_frame=True)
            uncertain_vertices.update_nodes(id_file, radar_motion, ego_rotate)
            radar_polygonISM1 = RadarPolygonISM(polygon_list, py_polygon, px_polygon, vel_polygon, id_polygon,
                                                snr_polygon, sectorId_polygon, conf_polygon, data_front[1],
                                                data_front[0], data_front[2], data_front[3], data_front[4],
                                                uncertain_vertices, vehicle_motion=radar_motion,
                                                vehicle_rotate=ego_rotate, is_radarScene=True)
            start = time.time()  # starting time
            polygon_list, _, py_polygon, px_polygon, vel_polygon, id_polygon, snr_polygon, sectorId_polygon, \
                conf_polygon, uncertain_vertices = radar_polygonISM1.calculate_polygon_ISM(math.radians(sampling_azi),
                fov=ism_fov, is_remove_spike=True, frame_id=id_file)
            end = time.time()  # end time
            history_ego = [cur_pos_x, cur_pos_y, cur_yaw, ts]

        time_total += (end - start)  # total time taken
        process_frame += 1

        if is_save_image:
            # save polygon results
            if not is_withpred:
                visualize_polygon_radarScene(data_front[1], data_front[0], py_polygon, px_polygon, conf_polygon,
                                             save_img_folder_name, id_file)
            else:
                # predict the polygon for next frame using the current polygon

                py_polygon_cur_pred, px_polygon_cur_pred = polygon_predict(polygon_list, py_polygon, px_polygon,
                                                                           vel_polygon, id_polygon,
                                                                           pred_frame_num=dt_frame)
                visualize_polygon_radarScene_withpred(data_front[1], data_front[0], py_polygon, px_polygon,
                                                      conf_polygon, py_polygon_pred, px_polygon_pred,
                                                      save_img_folder_name + '_pred', id_file)
                py_polygon_pred = py_polygon_cur_pred
                px_polygon_pred = px_polygon_cur_pred

        if is_save_polygon:
            # save_results to txt file for further evaluation
            save_results_radarScene(save_txt_folder_name, id_file, py_polygon, px_polygon, conf_polygon)
        print('Finished', id_file)

    print(f"Runtime of the program is {time_total / process_frame}")
    # print('***************************end polygon formation for single frame*******************************')


if __name__ == '__main__':
    is_withpred = False
    main(is_withpred=is_withpred)
