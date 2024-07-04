import os
import math
import json
import pickle

from utils.calc_iou import polygon_iou
from utils.timestamp import match_timestamp


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


def calc_polygon_conf(px_polygon, py_polygon, polygon_x_gt, polygon_y_gt):
    polygon_glob = [[px_polygon[idl], py_polygon[idl]] for idl in range(len(py_polygon))]
    # calculate the confidence of all valid vertices, valid mean the vertex is close to a gt vertex
    conf_sum = 0
    for px, py in polygon_glob:
        flag = validate_poly_vertex(px, py, polygon_x_gt, polygon_y_gt, threshold=4)
        if flag:
            conf_sum += 1   # valid
    if len(polygon_glob) > 0:
        return conf_sum / len(polygon_glob)
    return 0


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


def validate_poly_vertex(px, py, polygon_gt_x, polygon_gt_y, threshold):
    # verify if (px, py) is located to a polygon vertex withing threshold distance
    n = len(polygon_gt_x)
    for idx in range(n):
        vertex_x = polygon_gt_x[idx]
        vertex_y = polygon_gt_y[idx]
        if math.sqrt((px - vertex_x) ** 2 + (py - vertex_y) ** 2) < threshold:
            return True
    return False


def evaluate_confidence_over_time(eva_dir, gt_dirs, is_output_all=False):
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
    conf_sum = 0     # confidence over time
    conf_all = []
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
        conf = calc_polygon_conf(px_polygon, py_polygon, polygon_x_gt, polygon_y_gt)
        conf_sum += conf
        count += 1
        conf_all.append([all_files.index(file_name), conf])

    if is_output_all:
        return conf_sum / count, conf_all
    else:
        return conf_sum / count


if __name__ == '__main__':
    eva_dir_ = 'res/results_poly/radarScene143_poly'
    gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    iou, iou_all = evaluate_poly_gt(eva_dir_, gt_dir_, is_output_all=True)
    print('iou_gt for single-shot polygon is: ', iou)
    iou_smooth, iou_smooth_all = evaluate_smoothness(eva_dir_, is_output_all=True)
    print('iou_smooth for single-shot polygon is: ', iou_smooth)
    conf_over_time, conf_all = evaluate_confidence_over_time(eva_dir_, gt_dir_, is_output_all=True)
    print('conf_over_time for single-shot polygon is: ', conf_over_time)

    eva_dir_ = 'res/results_poly/radarScene143_ism_poly'
    gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    iou, iou_all = evaluate_poly_gt(eva_dir_, gt_dir_, is_output_all=True)
    print('iou_gt for ISM-based polygon is: ', iou)
    iou_smooth, iou_smooth_all = evaluate_smoothness(eva_dir_, is_output_all=True)
    print('iou_smooth for ISM-based polygon is: ', iou_smooth)
    conf_over_time, conf_all = evaluate_confidence_over_time(eva_dir_, gt_dir_, is_output_all=True)
    print('conf_over_time for single-shot polygon is: ', conf_over_time)

    eva_dir_ = 'res/results_poly/radarScene143_meerpohl'
    gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    iou, iou_all = evaluate_poly_gt(eva_dir_, gt_dir_, is_output_all=True)
    print('iou_gt for Meerpohl polygon is: ', iou)
    iou_smooth, iou_smooth_all = evaluate_smoothness(eva_dir_, is_output_all=True)
    print('iou_smooth for Meerpohl polygon is: ', iou_smooth)
    conf_over_time, conf_all = evaluate_confidence_over_time(eva_dir_, gt_dir_, is_output_all=True)
    print('conf_over_time for Meerpohl polygon is: ', conf_over_time)

    eva_dir_ = 'res/results_poly/radarScene143_ziegler'
    gt_dir_ = ['label_gt/labels_radarscenee_2022-11-22-01-56-56.json']
    iou, iou_all = evaluate_poly_gt(eva_dir_, gt_dir_, is_output_all=True)
    print('iou_gt for Ziegler polygon is: ', iou)
    iou_smooth, iou_smooth_all = evaluate_smoothness(eva_dir_, is_output_all=True)
    print('iou_smooth for Ziegler polygon is: ', iou_smooth)
    conf_over_time, conf_all = evaluate_confidence_over_time(eva_dir_, gt_dir_, is_output_all=True)
    print('conf_over_time for Ziegler polygon is: ', conf_over_time)
