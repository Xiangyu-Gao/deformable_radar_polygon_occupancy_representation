import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image
from config import polygon_configs, radar_enable
from utils.coord_trans import coord_radar2bev
from utils.calc_iou import polygon_iou


def visualize(px, py, px_polygon, py_polygon, polygon_list, bound_x, bound_y, save_folder, frame_id):
    """
    visualize radar polygon
    """
    # create folder if not exist
    directory = os.path.join('./results_radar', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    # plot all points
    plt.plot(px, py, 'go')

    # plot all selected points
    plt.plot(px_polygon, py_polygon, 'ro')  # plot a line between each pair of selected points

    # connect the points to form the polygon
    for i in range(len(px_polygon) - 1):
        x_values = [px_polygon[i], px_polygon[i + 1]]
        y_values = [py_polygon[i], py_polygon[i + 1]]
        plt.plot(x_values, y_values, 'k')

    # connect the first point with last point
    x_values = [px_polygon[-1], px_polygon[0]]
    y_values = [py_polygon[-1], py_polygon[0]]
    plt.plot(x_values, y_values, 'k')

    # fill the polygon region
    plt.fill(px_polygon, py_polygon, 'skyblue')

    # highlight virtual points with black marker
    virtual_list = [ids for ids, s in enumerate(polygon_list) if s == -1]
    py_virtual = [py_polygon[ids] for ids in virtual_list]
    px_virtual = [px_polygon[ids] for ids in virtual_list]
    plt.plot(px_virtual, py_virtual, 'ko')  # plot all virtual points

    # plot boundary with dotted line
    plt.plot(bound_x, bound_y, 'k--')  # plot a dotted line for field of view boundary

    # plot the car itself
    # specify the location of (left, bottom), width, height
    rect = mpatches.Rectangle((-1.07, -4.661), 2.14, 4.95, fill=False, color="purple", linewidth=2)
    plt.gca().add_patch(rect)

    plt.axis([-26, 26, -5, 26])
    plt.xlabel("x (meter)")
    plt.ylabel("y (meter)")

    # Set the scale interval of the x-axis to 5, and store it in the variable
    x_major_locator = plt.MultipleLocator(5)
    y_major_locator = plt.MultipleLocator(5)
    ax = plt.gca()
    ax.xaxis.set_major_locator(x_major_locator)
    # Set the main scale of the x-axis to a multiple of 1
    ax.yaxis.set_major_locator(y_major_locator)

    # set figure size
    fig = plt.gcf()
    fig.set_size_inches(18, 9)
    plt.savefig(f'./results_radar/{save_folder:s}/{frame_id:04d}.png')
    plt.cla()


def visualize_bev(px, py, px_polygon, py_polygon, polygon_list, bound_x, bound_y, save_folder, frame_id):
    """
    plot radar polygon in bev coordinates
    """
    # create folder if not exist
    directory = os.path.join('./results_radar', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    # plot all points
    px, py = coord_radar2bev(px, py)
    plt.plot(px, py, 'go')

    # plot all selected points
    px_polygon, py_polygon = coord_radar2bev(px_polygon, py_polygon)
    plt.plot(px_polygon, py_polygon, 'ro')  # plot a line between each pair of selected points

    # connect the points to form the polygon
    for i in range(len(px_polygon) - 1):
        x_values = [px_polygon[i], px_polygon[i + 1]]
        y_values = [py_polygon[i], py_polygon[i + 1]]
        plt.plot(x_values, y_values, 'k')

    # connect the first point with last point
    x_values = [px_polygon[-1], px_polygon[0]]
    y_values = [py_polygon[-1], py_polygon[0]]
    plt.plot(x_values, y_values, 'k')

    # fill the polygon region
    plt.fill(px_polygon, py_polygon, 'skyblue')

    # highlight virtual points with black marker
    virtual_list = [ids for ids, s in enumerate(polygon_list) if s == -1]
    py_virtual = [py_polygon[ids] for ids in virtual_list]
    px_virtual = [px_polygon[ids] for ids in virtual_list]
    plt.plot(px_virtual, py_virtual, 'ko')  # plot all virtual points

    # plot boundary with dotted line
    bound_x, bound_y = coord_radar2bev(bound_x, bound_y)
    plt.plot(bound_x, bound_y, 'k--')  # plot a dotted line for field of view boundary

    # plot the car itself
    # specify the location of (left, bottom), width, height
    car_coord_x, car_coord_y = coord_radar2bev(-1.07, -4.661)
    rect = mpatches.Rectangle((car_coord_x, car_coord_y), 21.4 * 2.4, 49.5 * 2.4, fill=False, color="purple",
                              linewidth=2)
    plt.gca().add_patch(rect)

    plt.axis([0, 400, 0, 600])

    # set figure size
    fig = plt.gcf()
    fig.set_size_inches(8, 12)
    plt.savefig(f'./results_radar/{save_folder:s}/{frame_id:s}.png')
    plt.cla()


def plot_on_bev(px_polygon_front, py_polygon_front, px_polygon_rear, py_polygon_rear, conf_polygon_front,
                conf_polygon_rear, bev_dir, save_folder, file_name, is_plot_node=False):
    """
    plot radar polygon on bev
    """
    # create folder if not exist
    directory = os.path.join('./results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    img = Image.open(bev_dir)
    plt.imshow(img)

    # transform the polygon to the bev coordinates
    px_polygon_front, py_polygon_front = coord_radar2bev(px_polygon_front, py_polygon_front, mode='front')
    px_polygon_rear, py_polygon_rear = coord_radar2bev(px_polygon_rear, py_polygon_rear, mode='rear')

    # fill the polygon region
    if radar_enable[0]:
        plt.fill(px_polygon_front, py_polygon_front, 'skyblue', alpha=0.6)
    if radar_enable[1]:
        plt.fill(px_polygon_rear, py_polygon_rear, 'skyblue', alpha=0.6)

    # plot the vertexes
    if radar_enable[0] and is_plot_node:
        plt.scatter(px_polygon_front, py_polygon_front, s=50, c=np.clip(conf_polygon_front, 0, 1), cmap='Reds')
    if radar_enable[1] and is_plot_node:
        plt.scatter(px_polygon_rear, py_polygon_rear, s=50, c=np.clip(conf_polygon_rear, 0, 1), cmap='Reds')

    # configure the saved figure
    plt.axis([0, 400, 600, 0])
    cbar = plt.colorbar()
    cbar.set_label('Confidence Score Colormap')

    # show figure
    plt.axis('off')
    # plt.show()

    # save figure
    plt.savefig(f'./results_bev/{save_folder:s}/{file_name:s}')
    plt.cla()
    cbar.remove()


def plot_on_bev_withpred(px_polygon_front, py_polygon_front, vel_polygon_front, px_polygon_rear, py_polygon_rear,
                         vel_polygon_rear, px_polygon_front_pred, py_polygon_front_pred, px_polygon_rear_pred,
                         py_polygon_rear_pred, px_polygon_front_cur_pred, py_polygon_front_cur_pred,
                         px_polygon_rear_cur_pred, py_polygon_rear_cur_pred, bev_dir, save_folder, file_name,
                         is_plotDop=False, is_remove_pred=False):
    """
    plot predicted radar polygon on bev
    if is_plotDop = True, also plot the Doppler velocity as arrow
    """
    # create folder if not exist
    directory = os.path.join('./results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    vel_thres = polygon_configs['vel_thres']
    arrow_scale = 5
    img = Image.open(bev_dir)
    plt.imshow(img)

    # calculate iou between the predicted polygon and the real-obtained polygon
    polygon1_front = [[px_polygon_front[idl], py_polygon_front[idl]] for idl in range(len(px_polygon_front))]
    polygon2_front = [[px_polygon_front_pred[idl], py_polygon_front_pred[idl]] for idl in
                      range(len(px_polygon_front_pred))]
    iou_front = polygon_iou(polygon1_front, polygon2_front)

    polygon1_rear = [[px_polygon_rear[idl], py_polygon_rear[idl]] for idl in range(len(px_polygon_rear))]
    polygon2_rear = [[px_polygon_rear_pred[idl], py_polygon_rear_pred[idl]] for idl in range(len(px_polygon_rear_pred))]
    iou_rear = polygon_iou(polygon1_rear, polygon2_rear)

    iou = (iou_rear + iou_front) / 2

    # plot iou text
    plt.text(15, 25, 'IoU=' + format(iou, '.4f'), fontsize=12, color='red')

    # transform the polygon to the bev coordinates
    px_polygon_front, py_polygon_front = coord_radar2bev(px_polygon_front, py_polygon_front, mode='front')
    px_polygon_rear, py_polygon_rear = coord_radar2bev(px_polygon_rear, py_polygon_rear, mode='rear')
    px_polygon_front_pred, py_polygon_front_pred = coord_radar2bev(px_polygon_front_pred, py_polygon_front_pred,
                                                                   mode='front')
    px_polygon_rear_pred, py_polygon_rear_pred = coord_radar2bev(px_polygon_rear_pred, py_polygon_rear_pred,
                                                                 mode='rear')
    px_polygon_front_cur_pred, py_polygon_front_cur_pred = coord_radar2bev(px_polygon_front_cur_pred,
                                                                           py_polygon_front_cur_pred, mode='front')
    px_polygon_rear_cur_pred, py_polygon_rear_cur_pred = coord_radar2bev(px_polygon_rear_cur_pred,
                                                                         py_polygon_rear_cur_pred, mode='rear')

    # plot the predicted polygon figure from last frame
    if len(px_polygon_front_pred) > 0 and not is_remove_pred:
        plt.plot(px_polygon_front_pred, py_polygon_front_pred, 'b--')  # plot all predicted points
        plt.plot(px_polygon_rear_pred, py_polygon_rear_pred, 'b--')  # plot all predicted points
        plt.plot([px_polygon_front_pred[-1], px_polygon_front_pred[0]],
                 [py_polygon_front_pred[-1], py_polygon_front_pred[0]], 'b--')  # connect the first point and last point
        plt.plot([px_polygon_rear_pred[-1], px_polygon_rear_pred[0]],
                 [py_polygon_rear_pred[-1], py_polygon_rear_pred[0]], 'b--')  # connect the first point and last point

    # fill the polygon region
    if radar_enable[0]:
        plt.fill(px_polygon_front, py_polygon_front, 'skyblue', alpha=0.9)
        if is_plotDop:
            # plot the velocity vector for each point with more than 0.1 velocity
            for idf in range(len(px_polygon_front)):
                if abs(vel_polygon_front[idf]) > vel_thres:
                    # plot an arrow for this point based on predicted movement
                    delta_x = px_polygon_front_cur_pred[idf] - px_polygon_front[idf]
                    delta_y = py_polygon_front_cur_pred[idf] - py_polygon_front[idf]
                    plt.arrow(px_polygon_front[idf], py_polygon_front[idf], delta_x * arrow_scale,
                              delta_y * arrow_scale,
                              width=0.1, head_width=0.5, color='red')
    if radar_enable[1]:
        plt.fill(px_polygon_rear, py_polygon_rear, 'skyblue', alpha=0.9)
        if is_plotDop:
            for idr in range(len(px_polygon_rear)):
                if abs(vel_polygon_rear[idr]) > vel_thres:
                    # plot an arrow for this point based on predicted movement
                    delta_x = px_polygon_rear_cur_pred[idr] - px_polygon_rear[idr]
                    delta_y = py_polygon_rear_cur_pred[idr] - py_polygon_rear[idr]
                    plt.arrow(px_polygon_rear[idr], py_polygon_rear[idr], delta_x * arrow_scale, delta_y * arrow_scale,
                              width=0.5, head_width=5, color='maroon')

    # configure the saved figure
    plt.axis([0, 400, 600, 0])
    # plt.axis([-300, 700, 800, -200])

    # show figure
    plt.axis('off')
    # plt.show()

    # save figure
    plt.savefig(f'./results_bev/{save_folder:s}/{file_name:s}')
    plt.cla()

    return iou


def visualize_point_cloud(px_front, py_front, px_rear, py_rear, save_folder, file_name):
    """
    visualize radar point cloud
    """
    # create folder if not exist
    directory = os.path.join('./results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    # plot out the point clouds
    plt.plot(px_front, py_front, 'go')

    plt.plot(px_rear, -4.95 - py_rear, 'go')

    # plot the car itself
    # specify the location of (left, bottom), width, height
    rect = mpatches.Rectangle((-1.07, -4.95), 2.14, 4.95, fill=False, color="purple",
                              linewidth=2)
    plt.gca().add_patch(rect)

    # configure
    plt.axis([-20, 20, -20, 20])

    # show figure
    # plt.axis('off')
    # plt.show()

    # save figure
    plt.savefig(f'./results_bev/{save_folder:s}/{file_name:s}')
    plt.cla()


def plot_point_cloud_on_bev(px_front, py_front, snr_front, px_rear, py_rear, snr_rear, bev_dir, save_folder, file_name):
    """
    plot radar point cloud on bev
    """
    # create folder if not exist
    directory = os.path.join('./results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    img = Image.open(bev_dir)
    plt.imshow(img)

    # transform the polygon to the bev coordinates
    px_front, py_front = coord_radar2bev(px_front, py_front, mode='front')
    px_rear, py_rear = coord_radar2bev(px_rear, py_rear, mode='rear')

    # fill the polygon region
    if radar_enable[0]:
        plt.scatter(px_front, py_front, c=np.asarray(snr_front), cmap='Greens')
    if radar_enable[1]:
        plt.scatter(px_rear, py_rear, c=np.asarray(snr_rear), cmap='Greens')

    # configure the saved figure
    plt.axis([0, 400, 600, 0])
    cbar = plt.colorbar()
    cbar.set_label('SNR Intensity Colormap')
    plt.clim(4, 28)

    # show figure
    plt.axis('off')
    # plt.show()

    # save figure
    plt.savefig(f'./results_bev/{save_folder:s}/{file_name:s}')
    plt.cla()
    cbar.remove()


def visualize_gridmap(save_folder, file_name, maps):
    # create folder if not exist
    directory = os.path.join('./results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)
    plt.imshow(maps.T, cmap="viridis")
    plt.axis('off')
    cbar = plt.colorbar()
    cbar.set_label('Confidence Score Colormap')
    # plt.clim(0, 100)   # werbe method
    # plt.clim(50, 150)  # Li method
    # save figure
    plt.savefig(f'./results_bev/{save_folder:s}/{file_name:s}')
    plt.cla()
    cbar.remove()


def visualize_polygon_radarScene(px, py, px_polygon, py_polygon, conf_polygon, save_folder, frame_id, is_plot_node=True):
    """
    visualize radar polygon
    """
    # create folder if not exist
    directory = os.path.join('./results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    # plot all points
    plt.plot(px, py, 'go', zorder=1)

    # fill the polygon region
    plt.fill(px_polygon, py_polygon, 'skyblue', alpha=0.6)

    # plot the vertexes
    if is_plot_node:
        plt.scatter(px_polygon, py_polygon, s=100, c=np.clip(conf_polygon, 0, 1), cmap='Reds', zorder=2)

    # plot the car itself, specify the location of (left, bottom), width, height
    rect = mpatches.Rectangle((-1.07, -4.7), 2.14, 4.95, fill=False, color="purple", linewidth=2)
    plt.gca().add_patch(rect)

    plt.axis([-26, 26, -15, 20])
    plt.rcParams.update({'font.size': 20})
    plt.xlabel("x (meter)", fontsize=20)
    plt.ylabel("y (meter)", fontsize=20)

    # Set the scale interval of the x-axis to 5, and store it in the variable
    x_major_locator = plt.MultipleLocator(5)
    y_major_locator = plt.MultipleLocator(5)
    ax = plt.gca()
    ax.xaxis.set_major_locator(x_major_locator)
    # Set the main scale of the x-axis to a multiple of 1
    ax.yaxis.set_major_locator(y_major_locator)
    cbar = plt.colorbar()
    cbar.set_label('Confidence Score Colormap')

    # set figure size
    fig = plt.gcf()
    fig.set_size_inches(19, 12)
    plt.savefig(f'./results_bev/{save_folder:s}/{frame_id:04d}.png')
    plt.cla()
    cbar.remove()


def visualize_polygon_radarScene_withpred(px, py, px_polygon, py_polygon, conf_polygon, px_polygon_pred, py_polygon_pred,
                                          save_folder, frame_id, is_plot_node=True):
    """
    plot predicted radar polygon on bev
    if is_plotDop = True, also plot the Doppler velocity as arrow
    """
    # create folder if not exist
    directory = os.path.join('./results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)

    # plot all points
    plt.plot(px, py, 'go', zorder=1)

    # fill the polygon region
    plt.fill(px_polygon, py_polygon, 'skyblue', alpha=0.6)

    # plot the vertexes
    if is_plot_node:
        plt.scatter(px_polygon, py_polygon, s=100, c=np.clip(conf_polygon, 0, 1), cmap='Reds', zorder=2)

    # plot the car itself, specify the location of (left, bottom), width, height
    rect = mpatches.Rectangle((-1.07, -4.7), 2.14, 4.95, fill=False, color="purple", linewidth=2)
    plt.gca().add_patch(rect)

    plt.rcParams.update({'font.size': 20})
    plt.xlabel("x (meter)", fontsize=20)
    plt.ylabel("y (meter)", fontsize=20)

    # Set the scale interval of the x-axis to 5, and store it in the variable
    x_major_locator = plt.MultipleLocator(5)
    y_major_locator = plt.MultipleLocator(5)
    ax = plt.gca()
    ax.xaxis.set_major_locator(x_major_locator)
    # Set the main scale of the x-axis to a multiple of 1
    ax.yaxis.set_major_locator(y_major_locator)
    cbar = plt.colorbar()
    cbar.set_label('Confidence Score Colormap')

    # set figure size
    fig = plt.gcf()
    fig.set_size_inches(18, 9)

    # calculate iou between the predicted polygon and the real-obtained polygon
    polygon1_front = [[px_polygon[idl], py_polygon[idl]] for idl in range(len(px_polygon))]
    polygon2_front = [[px_polygon_pred[idl], py_polygon_pred[idl]] for idl in range(len(px_polygon_pred))]
    iou= polygon_iou(polygon1_front, polygon2_front)

    # plot iou text
    plt.text(15, 25, 'IoU=' + format(iou, '.4f'), fontsize=12, color='red')

    # plot the predicted polygon figure from last frame
    if len(px_polygon_pred) > 0:
        plt.plot(px_polygon_pred, py_polygon_pred, 'b--')  # plot all predicted points
        # connect the first point and last point
        plt.plot([px_polygon_pred[-1], px_polygon_pred[0]], [py_polygon_pred[-1], py_polygon_pred[0]], 'b--')

    plt.savefig(f'./results_bev/{save_folder:s}/{frame_id:04d}.png')
    plt.cla()
    cbar.remove()

    return iou