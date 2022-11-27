import math

from config import polygon_configs, radar_configs, sensor_locations

vel_thres = polygon_configs['vel_thres']
duration = radar_configs['duration']


def polygon_predict(polygon_list, polygon_px, polygon_py, polygon_vel, polygon_id, is_clean=False, pred_frame_num=1):
    """ return the updated/predicted polygon"""
    if is_clean:
        polygon_list, polygon_px, polygon_py, polygon_vel, polygon_id = clean_polygon(polygon_list, polygon_px,
                                                                                      polygon_py, polygon_vel,
                                                                                      polygon_id)
    polygon_py_new = polygon_py.copy()
    polygon_px_new = polygon_px.copy()

    for id, poly_idx in enumerate(polygon_list):
        if poly_idx != -1 and abs(polygon_vel[id]) > vel_thres:
            sensor_x = sensor_locations[int(polygon_id[id]) - 1][0]
            sensor_y = sensor_locations[int(polygon_id[id]) - 1][1]

            # predict the radial shift for next frame
            delta_x, delta_y = predict_node_radial_movment(sensor_x, sensor_y, polygon_px[id], polygon_py[id],
                                                           polygon_vel[id])
            # update the py_polygon and px_polygon for prediction
            polygon_py_new[id] = polygon_py[id] + delta_y * pred_frame_num
            polygon_px_new[id] = polygon_px[id] + delta_x * pred_frame_num

    return polygon_px_new, polygon_py_new


def predict_node_radial_movment(sensor_x, sensor_y, px, py, vel):
    """predict the radial movement for a given point with velocity"""
    dist = math.sqrt((px - sensor_x) ** 2 + (py - sensor_y) ** 2)
    ratio = abs(vel) * duration / dist

    if vel > 0:
        delta_x = ratio * (px - sensor_x)
        delta_y = ratio * (py - sensor_y)
    else:
        delta_x = - ratio * (px - sensor_x)
        delta_y = - ratio * (py - sensor_y)

    return delta_x, delta_y


def clean_polygon(polygon_list, px_polygon, py_polygon, vel_polygon, id_polygon):
    """ clean the polygon for prediction: remove the None elements from polygon_list """
    polygon_list_new = []
    px_polygon_new = []
    py_polygon_new = []
    vel_polygon_new = []
    id_polygon_new = []

    for idx, ele in enumerate(polygon_list):
        if ele is not None:
            polygon_list_new.append(ele)
            px_polygon_new.append(px_polygon[idx])
            py_polygon_new.append(py_polygon[idx])
            vel_polygon_new.append(vel_polygon[idx])
            id_polygon_new.append(id_polygon[idx])

    return polygon_list_new, px_polygon_new, py_polygon_new, vel_polygon_new, id_polygon_new
