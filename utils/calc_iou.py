from shapely.geometry import Polygon


def polygon_iou(polygon_1, polygon_2):
    """
    Calculate the IOU of two polygons
    :param polygon_1: [[row1, col1], [row2, col2], ...]
         :param polygon_2: Same as above
    :return: iou
    """
    polygon1_shape = Polygon(polygon_1)
    polygon2_shape = Polygon(polygon_2)

    # check if the polygon is valid
    if not polygon1_shape.is_valid:
        polygon1_shape = polygon1_shape.buffer(0)
    if not polygon2_shape.is_valid:
        polygon2_shape = polygon2_shape.buffer(0)

    # limit the radar polygon to the image fov
    image_polygon = Polygon([[0, 600], [400, 600], [400, 0], [0, 0]])
    polygon1_shape = polygon1_shape.intersection(image_polygon)
    polygon2_shape = polygon2_shape.intersection(image_polygon)

    # Calculate Intersection and union, and tne IOU
    polygon_intersection = polygon1_shape.intersection(polygon2_shape).area
    polygon_union = polygon1_shape.union(polygon2_shape).area
    iou = polygon_intersection / polygon_union

    return iou


if __name__ == '__main__':
    pol1_xy = [[130, 27], [129.52, 27], [129.45, 27.1], [130.13, 26], [160, 25], [160, 30]]
    pol2_xy = [[30, 27.200001], [129.52, 27.34], [129.45, 27.1], [130.13, 26.950001], [160, 25], [160, 30]]
    iou = polygon_iou(pol1_xy, pol2_xy)
    print('IOU={:.5f}'.format(iou))