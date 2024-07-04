import math


def coord_rotate(px, py, rotate_angle):
    # the rotate angle is the anti-clock direction from [px, py] to [px_new, py_new]
    px_new = math.sin(rotate_angle) * py + math.cos(rotate_angle) * px
    py_new = math.cos(rotate_angle) * py - math.sin(rotate_angle) * px

    return px_new, py_new
