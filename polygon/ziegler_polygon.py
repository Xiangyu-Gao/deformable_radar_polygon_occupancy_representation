import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import ConvexHull
from shapely.geometry import Polygon


class zieglerPolygon:
    """
    class for forming radar polygon using single frame measurement
    """
    def __init__(self, rC=30, is_half_bond=True):
        """
        :param rC: max distance for the line
        """
        self.rC = rC
        self.is_half_bond = is_half_bond    # whole view or half bond view

    def create_polygon(self, px, py):
        """
        :param px: input x array
        :param py: input y array
        :return:
        """
        # split left side and right side
        px_arr = np.asarray(px).reshape((-1, 1))
        py_arr = np.asarray(py).reshape((-1, 1))
        left_inds = np.where(px_arr <= 0)[0]
        right_inds = np.where(px_arr > 0)[0]
        points = np.concatenate((px_arr, py_arr), axis=1)  # points in 2-D
        left_points = points[left_inds, :]
        right_points = points[right_inds, :]

        # create left hull
        left_hull = ConvexHull(left_points)
        left_hull_vertex_x = left_points[left_hull.vertices, 0].squeeze()
        left_hull_vertex_y = left_points[left_hull.vertices, 1].squeeze()
        # create right hull
        right_hull = ConvexHull(right_points)
        right_hull_vertex_x = right_points[right_hull.vertices, 0].squeeze()
        right_hull_vertex_y = right_points[right_hull.vertices, 1].squeeze()

        # transform the occupancy polygon to the free space polygon
        coords1 = []
        for i in range(left_hull_vertex_x.shape[0]):
            coords1.append((left_hull_vertex_x[i], left_hull_vertex_y[i]))
        polygon1 = Polygon(coords1)
        if not polygon1.is_valid:
            polygon1 = polygon1.buffer(0)

        eps = 1
        left_bond = np.min(left_hull_vertex_x) + eps
        right_bond = np.max(right_hull_vertex_x) - eps
        bot_bond = min(np.min(left_hull_vertex_y), np.min(right_hull_vertex_y)) - eps
        top_bond = max(np.max(left_hull_vertex_y), np.max(right_hull_vertex_y)) + eps
        if self.is_half_bond:
            coords2 = [(0, 0), (left_bond, 0), (left_bond, self.rC), (right_bond, self.rC), (right_bond, 0)]
        else:
            coords2 = [(left_bond, bot_bond), (left_bond, top_bond), (right_bond, top_bond), (right_bond, bot_bond)]
        polygon2 = Polygon(coords2)
        fs_polygon = polygon2.difference(polygon1)

        coords3 = []
        for i in range(right_hull_vertex_x.shape[0]):
            coords3.append((right_hull_vertex_x[i], right_hull_vertex_y[i]))
        polygon3 = Polygon(coords3)
        if not polygon3.is_valid:
            polygon3 = polygon3.buffer(0)
        fs_polygon = fs_polygon.difference(polygon3)

        if fs_polygon.type == 'MultiPolygon':
            vertex_x = []
            vertex_y = []
            for poly in list(fs_polygon):
                xx, yy = poly.exterior.coords.xy
                vertex_x += list(xx)
                vertex_y += list(yy)
        else:
            xx, yy = fs_polygon.exterior.coords.xy
            vertex_x = list(xx)
            vertex_y = list(yy)

        # # visualization
        # plt.plot(left_hull_vertex_x, left_hull_vertex_y, 'b-')
        # plt.scatter(left_points[:, 0], left_points[:, 1])
        #
        # plt.plot(right_hull_vertex_x, right_hull_vertex_y, 'b-')
        # plt.scatter(right_points[:, 0], right_points[:, 1])
        #
        # plt.plot(vertex_x, vertex_y, 'k-')
        # plt.show()
        # input()

        return vertex_x, vertex_y   # the free space polygon
