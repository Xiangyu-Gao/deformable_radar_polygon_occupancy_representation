import math
import time
import numpy as np

from polygon.radar_polygon import RadarPolygon
from dataLoader.data_loader import DataLoaderRadarScene
from utils.visualize import visualize_polygon_radarScene
from utils.save_result import save_results_radarScene
from config import polygon_configs, snr_elevation


def main(is_calc_config=False, is_save_polygon=True, is_save_image=True):
    print('***************************radar scene data*******************************')
    print('***************************start polygon formation for single frame*******************************')
    fov = [-math.radians(140), math.radians(140)]
    save_txt_folder_name = 'radarScene143_poly'
    save_img_folder_name = 'radarScene143_poly'
    data_loader = DataLoaderRadarScene()
    sampling_azi = polygon_configs['sampling_reg']  # degree
    time_total = 0
    mean1_all = 0
    var1_all = 0

    start_id = 3
    for id_file in range(start_id, data_loader.frame_number):
        data_front = data_loader.load_radar_data(id_file)
        data_front[4] = np.maximum(data_front[4] + snr_elevation, 0)    # boost and clip snr
        data_front[1] = - data_front[1]    # flip the direction for y
        radar_polygon1 = RadarPolygon(data_front[1], data_front[0], data_front[2], data_front[3], data_front[4])   # switch the x, y direction
        if is_calc_config:
            # analyze the distribution of sum_det_prob and update config when 'is_update_config=True'
            mean1, var1 = radar_polygon1.analyze_sum_prob_distrb()
            mean1_all += mean1
            var1_all += var1
            # # update config
            # polygon_configs['prob_mean'] = mean1
            # polygon_configs['prob_var'] = var1 * 2 / 3  # rescale 2*sigma to the location of 3 in sigmoid (value 0.9526)

        start = time.time()  # starting time
        polygon_list, _, py_polygon, px_polygon, vel_polygon, id_polygon, snr_polygon, _, conf_polygon \
            = radar_polygon1.calculate_polygon(math.radians(sampling_azi), fov=fov, is_remove_spike=False)
        end = time.time()  # end time
        time_total += (end - start)  # total time taken

        # plot the radar point clouds on plain coordinates
        if is_save_image:
            visualize_polygon_radarScene(data_front[1], data_front[0], py_polygon, px_polygon, conf_polygon,
                                         save_img_folder_name, id_file)
        # save_results to txt file for further evaluation
        if is_save_polygon:
            save_results_radarScene(save_txt_folder_name, id_file, py_polygon, px_polygon, conf_polygon)
        print('Finished', id_file)

    frames_len = data_loader.frame_number - start_id
    if is_calc_config:
        print('prob_mean: {:.6f}, prob_var: {:.6f}'.format(mean1_all / frames_len, var1_all / frames_len * 2 / 3))

    print(f"Runtime of the program is {time_total / frames_len}")
    print('***************************end polygon formation for single frame*******************************')


if __name__ == '__main__':
    main()
