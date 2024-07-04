import time
import numpy as np

from polygon.ziegler_polygon import zieglerPolygon
from dataLoader.data_loader import DataLoaderRadarScene
from utils.visualize import visualize_polygon_radarScene
from utils.save_result import save_results_radarScene
from config import snr_elevation


def main(is_save_polygon=True, is_save_image=True):
    print('***************************radar scene data*******************************')
    print('***************************start polygon formation for Ziegler*******************************')
    save_txt_folder_name = 'radarScene143_ziegler'
    save_img_folder_name = 'radarScene143_ziegler'
    data_loader = DataLoaderRadarScene()
    time_total = 0
    radar_polygon1 = zieglerPolygon(is_half_bond=False)  # initialization

    start_id = 3
    for id_file in range(start_id, data_loader.frame_number):
        data_front = data_loader.load_radar_data(id_file)
        data_front[4] = np.maximum(data_front[4] + snr_elevation, 0)    # boost and clip snr
        data_front[1] = - data_front[1]    # flip the direction for y

        start = time.time()  # starting time
        py_polygon, px_polygon = radar_polygon1.create_polygon(data_front[1], data_front[0])
        conf_polygon = [1] * np.size(py_polygon, 0)
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

    print(f"Runtime of the program is {time_total / frames_len}")
    print('***************************end polygon formation for single frame*******************************')


if __name__ == '__main__':
    main()
