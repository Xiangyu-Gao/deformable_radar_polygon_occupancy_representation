import math

from config import radar_configs, loc_radar1


def calc_boundary(azimuth_rad, fov=[-math.pi/2, math.pi/2]):
    max_dis = radar_configs['max_dis']
    if azimuth_rad == 0:
        return 0, max_dis

    a = abs(loc_radar1[0])
    b = 1 / math.tan(azimuth_rad)
    R = max_dis
    if azimuth_rad == fov[1]:
        return a, 0
    elif azimuth_rad == fov[0]:
        return -a, 0

    if azimuth_rad < 0:
        # boundary is arc curve
        x = (-a - math.sqrt(R ** 2 - (a * b) ** 2 + (b * R) ** 2)) / (1 + b ** 2)
        y = b * x
    else:
        # boundary is a line, symmetry
        x = (a + math.sqrt(R ** 2 - (a * b) ** 2 + (b * R) ** 2)) / (1 + b ** 2)
        y = b * x

    return x, y


def gene_boundary(search_grid, num_search):
    bound_x = []
    bound_y = []

    for idx in range(num_search - 1):
        azimuth_rad = search_grid[idx]
        px, py = calc_boundary(azimuth_rad)
        bound_x.append(px)
        bound_y.append(py)

    return bound_x, bound_y


def shrink_fov(px_polygon, py_polygon):
    uss_max_dist = 5
    px_polygon_new = px_polygon.copy()
    py_polygon_new = py_polygon.copy()

    for idx in range(len(px_polygon)):
        px = px_polygon[idx]
        py = py_polygon[idx]
        dist = math.sqrt(px ** 2 + py ** 2)

        if dist > uss_max_dist:
            angle = math.atan(px / py)
            px_polygon_new[idx] = uss_max_dist * math.sin(angle)
            py_polygon_new[idx] = uss_max_dist * math.cos(angle)

    return px_polygon_new, py_polygon_new
