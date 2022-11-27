import os
import json
import pickle
import matplotlib.pyplot as plt
import numpy as np

from utils.coord_trans import coord_radar2bev, coor_bev2realworld, coord_rotate
from utils.calc_iou import polygon_iou
from utils.timestamp import match_timestamp
from utils.is_insidePolygon import is_insidePolygon


# Python code to merge dict using a single expression
def Merge(dict1, dict2):
    res = {**dict1, **dict2}
    return res


def coord_image2world(px_polygon, py_polygon):
    px_polygon_new = [0.03963 * ele - 36.62 for ele in px_polygon]
    py_polygon_new = [-0.06111 * ele + 33.62 for ele in py_polygon]

    return px_polygon_new, py_polygon_new


def calc_polygons_iou(px_polygon, py_polygon, polygon_x_gt, polygon_y_gt):
    polygon_glob = [[px_polygon[idl], py_polygon[idl]] for idl in range(len(px_polygon))]
    polygon_gt = [[polygon_x_gt[idl], polygon_y_gt[idl]] for idl in range(len(polygon_x_gt))]
    # plt.plot(px_polygon_glob, py_polygon_glob, 'b-')
    # plt.plot(polygon_x_gt, polygon_y_gt, 'r-')
    # plt.show()
    # input()

    # calculate the IoU between combined one and polygon_gt
    iou = polygon_iou(polygon_glob, polygon_gt)
    return iou


def calc_polygon_mse(px_polygon, py_polygon, polygon_x_gt, polygon_y_gt):
    # for a random point within the free space of polygon_gt, check if it is in the new formed polygon
    count = 0
    mse = 0
    x_list = list(np.linspace(-10, 10, num=67))
    y_list = list(np.linspace(-10, 10, num=100))
    for idx1 in x_list:
        for idx2 in y_list:
            if is_insidePolygon(polygon_x_gt, polygon_y_gt, idx1, idx2):
                count += 1
                # check if this point is in the new formed polygon
                flag1 = is_insidePolygon(px_polygon, py_polygon, idx1, idx2)
                if flag1:
                    mse += 0
                else:
                    mse += 1

    print(mse, count)
    return mse/count


def calc_gradmap_mse(gridmap, map_start_index, map_resolution, polygon_x_gt, polygon_y_gt, conf_thres=40):
    # for point within the free space of polygon_gt, check if it is free in the grid map based on confidence
    count = 0
    mse = 0
    x_list = list(np.linspace(-10, 10, num=67))
    y_list = list(np.linspace(-10, 10, num=100))
    for idx1 in x_list:
        for idx2 in y_list:
            if is_insidePolygon(polygon_x_gt, polygon_y_gt, idx1, idx2):
                # convert the index to the real-world measurement values
                idx_x = int(idx1 / map_resolution) + map_start_index[0]
                idx_y = int(idx2 / map_resolution) + map_start_index[1]
                if -1 < idx_x < gridmap.shape[0] and -1 < idx_y < gridmap.shape[1]:
                    count += 1
                    # check if this point has confidence greater than a threshold
                    if gridmap[idx_x, idx_y] <= conf_thres:
                        mse += 0
                    else:
                        mse += 1
    print(mse, count)
    return mse / count


def evaluate_smoothness(eva_dir, is_output_all=False):
    # load the data that needs to be evaluated
    all_files = os.listdir(eva_dir)
    iou_smooth = 0
    iou_all = []
    for idx, file_name in enumerate(all_files):
        # load the file_name file
        fp = open(os.path.join(eva_dir, file_name), 'rb')
        data_list = pickle.load(fp)
        fp.close()
        px_polygon, py_polygon = data_list[0], data_list[1]
        if idx % 2 == 0:
            # skip
            prev_px_polygon, prev_py_polygon = px_polygon, py_polygon
        else:
            iou = calc_polygons_iou(px_polygon, py_polygon, prev_px_polygon, prev_py_polygon)
            iou_smooth += iou
            iou_all.append(iou)

    if is_output_all:
        return iou_smooth / (len(all_files)//2), iou_all
    else:
        return iou_smooth / (len(all_files)//2)


def evaluate_poly_gt(eva_dir, gt_dirs, is_output_all=False):
    for idx, gt_dir in enumerate(gt_dirs):
        # Opening JSON file
        f = open(gt_dir)
        # returns JSON object as a dictionary
        data_tmp = json.load(f)
        # Closing file
        f.close()
        if idx == 0:
            data = data_tmp
        else:
            data = Merge(data, data_tmp)

    # load the data that needs to be evaluated
    all_files = sorted(os.listdir(eva_dir))
    iou_sum = 0
    iou_all = []
    count = 0

    # Iterating through the json
    for attr in list(data.keys()):
        file_name = match_timestamp(all_files, attr[:-4])
        # load the file_name file
        fp = open(os.path.join(eva_dir, file_name), 'rb')
        data_list = pickle.load(fp)
        fp.close()
        px_polygon, py_polygon = data_list[0], data_list[1]
        polygon_x_gt = data[attr]["regions"]["0"]["shape_attributes"]["all_points_x"]
        polygon_y_gt = data[attr]["regions"]["0"]["shape_attributes"]["all_points_y"]

        polygon_x_gt, polygon_y_gt = coord_image2world(polygon_x_gt, polygon_y_gt)
        iou = calc_polygons_iou(px_polygon, py_polygon, polygon_x_gt, polygon_y_gt)
        # print(iou)
        # if int(file_name) > 1628200068590000:
        #     continue
        iou_sum += iou
        count += 1
        iou_all.append([all_files.index(file_name), iou])

    if is_output_all:
        return iou_sum/count, iou_all
    else:
        return iou_sum/count


def evaluate_poly_mse(eva_dir, gt_dirs):
    for idx, gt_dir in enumerate(gt_dirs):
        # Opening JSON file
        f = open(gt_dir)
        # returns JSON object as a dictionary
        data_tmp = json.load(f)
        # Closing file
        f.close()
        if idx == 0:
            data = data_tmp
        else:
            data = Merge(data, data_tmp)

    # Iterating through the json
    all_files = sorted(os.listdir(eva_dir))
    for attr in list(data.keys()):
        file_name = match_timestamp(all_files, attr[:-4])
        # load the file_name file
        fp = open(os.path.join(eva_dir, file_name), 'rb')
        data_list = pickle.load(fp)
        fp.close()
        px_polygon, py_polygon = data_list[0], data_list[1]
        polygon_x_gt = data[attr]["regions"]["0"]["shape_attributes"]["all_points_x"]
        polygon_y_gt = data[attr]["regions"]["0"]["shape_attributes"]["all_points_y"]

        polygon_x_gt, polygon_y_gt = coord_image2world(polygon_x_gt, polygon_y_gt)
        mse = calc_polygon_mse(px_polygon, py_polygon, polygon_x_gt, polygon_y_gt)
        return mse


def evaluate_gridmap_mse(eva_dir, gt_dirs, map_start_index, map_resu):
    for idx, gt_dir in enumerate(gt_dirs):
        # Opening JSON file
        f = open(gt_dir)
        # returns JSON object as a dictionary
        data_tmp = json.load(f)
        # Closing file
        f.close()
        if idx == 0:
            data = data_tmp
        else:
            data = Merge(data, data_tmp)

    # Iterating through the json
    all_files = sorted(os.listdir(eva_dir))
    for attr in list(data.keys()):
        file_name = match_timestamp(all_files, attr[:-4])
        # load the file_name file
        fp = open(os.path.join(eva_dir, file_name), 'rb')
        data_list = pickle.load(fp)
        fp.close()
        # load the data that needs to be evaluated
        grid_map = data_list[0]
        polygon_x_gt = data[attr]["regions"]["0"]["shape_attributes"]["all_points_x"]
        polygon_y_gt = data[attr]["regions"]["0"]["shape_attributes"]["all_points_y"]

        polygon_x_gt, polygon_y_gt = coord_image2world(polygon_x_gt, polygon_y_gt)

        mse = calc_gradmap_mse(grid_map, map_start_index, map_resu, polygon_x_gt, polygon_y_gt, conf_thres=10)

        return mse


if __name__ == '__main__':
    # eva_dir_ = 'results_poly/radarScene143_poly'
    # gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    # iou, iou_all = evaluate_poly_gt(eva_dir_, gt_dir_, is_output_all=True)
    # print(iou)
    # iou_smooth, iou_smooth_all = evaluate_smoothness(eva_dir_, is_output_all=True)
    # print(iou_smooth)
    #
    # eva_dir_ = 'results_poly/radarScene143_ism_poly'
    # gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    # iou, iou_all = evaluate_poly_gt(eva_dir_, gt_dir_, is_output_all=True)
    # print(iou)
    # iou_smooth, iou_smooth_all = evaluate_smoothness(eva_dir_, is_output_all=True)
    # print(iou_smooth)

    # eva_dir_ = 'results_poly/radarScene143_poly'
    # gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    # mse = evaluate_poly_mse(eva_dir_, gt_dir_)
    # print('mse is: ', mse)
    #
    # eva_dir_ = 'results_poly/radarScene143_ism_poly'
    # gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    # mse = evaluate_poly_mse(eva_dir_, gt_dir_)
    # print('mse is: ', mse)

    eva_dir_ = 'results_gridmap/radarScene143_ism_werbe'
    gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    map_start_index = [120, 120]
    map_resolution = [0.3]
    mse = evaluate_gridmap_mse(eva_dir_, gt_dir_, map_start_index, map_resolution)
    print('mse is: ', mse)

    eva_dir_ = 'results_gridmap/radarScene143_ism_Li'
    gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    map_start_index = [120, 120]
    map_resolution = [0.3]
    mse = evaluate_gridmap_mse(eva_dir_, gt_dir_, map_start_index, map_resolution)
    print('mse is: ', mse)
