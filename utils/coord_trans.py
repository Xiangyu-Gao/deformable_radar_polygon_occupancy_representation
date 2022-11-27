import math

import numpy as np


def coord_rotate(px, py, rotate_angle):
    # the rotate angle is the anti-clock direction from [px, py] to [px_new, py_new]
    px_new = math.sin(rotate_angle) * py + math.cos(rotate_angle) * px
    py_new = math.cos(rotate_angle) * py - math.sin(rotate_angle) * px

    return px_new, py_new


def coord_uss2bev(px, py):
    px = np.asarray(px)
    py = np.asarray(py)

    px_new = 2.4 * (px + 200 / 24 * 10)
    py_new = 2.4 * (py - 39.5 + 325 / 24 * 10)

    return px_new, py_new


def coord_radar2uss(px, py, vet_dis=28.9, mode='front'):
    px = np.asarray(px)
    py = np.asarray(py)

    factor = 10
    m2cm = 100 / factor

    if mode == 'front':
        px_new = px * m2cm
        py_new = - py * m2cm + vet_dis / factor
    elif mode == 'rear':
        px_new = - px * m2cm
        py_new = py * m2cm + vet_dis / factor

    return px_new, py_new


def coord_radar2bev(px, py, mode='front'):
    if mode == 'front':
        vet_dis = 28.9
    elif mode == 'rear':
        vet_dis = 480

    px_new, py_new = coord_radar2uss(px, py, vet_dis, mode)
    px_new, py_new = coord_uss2bev(px_new, py_new)

    return px_new, py_new


def coor_bev2realworld(px, py):
    """
    transform the bev pixel to the real world coordinated origin at car center
    :param px:
    :param py:
    :return:
    """
    factor = 10
    m2cm = 100 / factor
    vet_dis = 247.5  # center of car

    # bev --> uss
    px_new = px / 2.4 - 200 / 24 * 10
    py_new = py / 2.4 + 39.5 - 325 / 24 * 10
    # uss --> center of car
    px_new = px_new / m2cm
    py_new = (py_new - vet_dis / factor) / (-m2cm)

    return px_new, py_new
