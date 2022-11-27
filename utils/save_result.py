import os
import pickle


def save_results(save_folder_name, bev_name, px_polygon_front, py_polygon_front, px_polygon_rear,
                 py_polygon_rear, conf_polygon_front, conf_polygon_rear):
    # create folder if not exist
    directory = os.path.join('./results_poly', save_folder_name)
    if not os.path.exists(directory):
        os.makedirs(directory)

    save_data = [px_polygon_front, py_polygon_front, px_polygon_rear, py_polygon_rear, conf_polygon_front, conf_polygon_rear]
    # open the file
    with open(os.path.join(directory, bev_name[:-4]), "wb") as fp:  # Pickling
        pickle.dump(save_data, fp)
    fp.close()


def save_gridmap_results(save_folder_name, bev_name, maps):
    # create folder if not exist
    directory = os.path.join('./results_gridmap', save_folder_name)
    if not os.path.exists(directory):
        os.makedirs(directory)

    save_data = [maps]
    # open the file
    with open(os.path.join(directory, bev_name[:-4]), "wb") as fp:  # Pickling
        pickle.dump(save_data, fp)
    fp.close()


def save_results_radarScene(save_folder_name, frame_id, px_polygon_front, py_polygon_front, conf_polygon_front):
    # create folder if not exist
    directory = os.path.join('./results_poly', save_folder_name)
    if not os.path.exists(directory):
        os.makedirs(directory)

    save_data = [px_polygon_front, py_polygon_front, conf_polygon_front]
    # open the file
    with open(os.path.join(directory, str(frame_id).zfill(4)), "wb") as fp:  # Pickling
        pickle.dump(save_data, fp)
    fp.close()
