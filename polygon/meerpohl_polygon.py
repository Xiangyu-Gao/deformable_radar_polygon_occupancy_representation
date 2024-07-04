import math

from bresenham import bresenham


def bresenham_circle(x0, y0, radius):
  x = radius
  y = 0
  err = 0
  points = []

  while x >= y:
    points.append((x0 + x, y0 + y))
    points.append((x0 + y, y0 + x))
    points.append((x0 - y, y0 + x))
    points.append((x0 - x, y0 + y))
    points.append((x0 - x, y0 - y))
    points.append((x0 - y, y0 - x))
    points.append((x0 + y, y0 - x))
    points.append((x0 + x, y0 - y))

    y += 1
    err += 1 + 2*y
    if 2*(err-x) + 1 > 0:
      x -= 1
      err += 1 - 2*x

  return points


class MeerpohlPolygon:
    """
    class for forming radar polygon using single frame measurement
    """
    def __init__(self, rC=30, rHC=5, nPts=180, tau=40, gridMapClass=None):
        """
        :param rC: max distance for the line
        :param rHC: radius for the half circle
        :param nPts: number of sampling points
        :param tau: threshold for grid map

        """
        self.rC = rC
        self.rHC = rHC
        self.nPts = nPts
        self.tau = tau
        self.gridMapClass = gridMapClass    # grid map class

    def create_line(self, theta_k):
        """
        return the coordinate of a line starts from grid origin with theta_k angle and rC distance
        :param theta_k:
        :return:
        """
        start = [0, 0]  # start_point in meters
        end = [start[0]+math.cos(theta_k)*self.rC, start[1]+math.sin(theta_k)*self.rC]  # end point in meters
        start_cell = self.gridMapClass.loc2index(start[0], start[1])
        end_cell = self.gridMapClass.loc2index(end[0], end[1])
        point_list = list(bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1]))

        return point_list

    def create_polygon_on_grid(self, grid_map):
        """
        :param grid_map: input
        :return:
        """
        theta_0 = 0
        px_polygon = []
        py_polygon = []
        px_polygon_ind = []
        py_polygon_ind = []
        confScore_polygon = []

        for k in range(self.nPts):
            theta_k = theta_0 + math.pi - 2 * math.pi * k / self.nPts
            # create a line
            L_k = self.create_line(theta_k)
            for l_point in L_k:
                # create a circle based on point
                cir_points = bresenham_circle(l_point[0], l_point[1], self.rHC)
                # filter the circle and get the half circle
                half_cir_points = []
                s1 = math.atan2(l_point[1], l_point[0])
                for c_point in cir_points:
                    s2 = math.atan2(c_point[1] - l_point[1], c_point[0] - l_point[0])
                    if abs(s1-s2) < math.pi/2:
                        half_cir_points.append(c_point)
                for q in cir_points:
                    # verify collision cell
                    if 0 <= q[0] < grid_map.shape[0] and 0 <= q[1] < grid_map.shape[1] and \
                            grid_map[q[0], q[1]] > self.tau:
                        # check if located on half circle
                        # convert cell index to meters
                        pos = self.gridMapClass.index2loc(q[0], q[1])
                        # save results
                        px_polygon.append(pos[0])
                        py_polygon.append(pos[1])
                        px_polygon_ind.append(q[0])
                        py_polygon_ind.append(q[1])
                        confScore_polygon.append(grid_map[q[0], q[1]])

        return px_polygon, py_polygon, px_polygon_ind, py_polygon_ind, confScore_polygon
