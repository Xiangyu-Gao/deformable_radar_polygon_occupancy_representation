import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from utils.calc_iou import polygon_iou


def visualize_gridmap(save_folder, file_name, maps):
    # create folder if not exist
    directory = os.path.join('./res/results_bev', save_folder)
    if not os.path.exists(directory):
        os.makedirs(directory)
    plt.imshow(maps.T, cmap="viridis")
    plt.axis('off')
    cbar = plt.colorbar()
    cbar.set_label('Confidence Score Colormap')
    # plt.clim(0, 100)   # werbe method
    # plt.clim(50, 150)  # Li method
    # save figure
    plt.savefig(f'./res/results_bev/{save_folder:s}/{file_name:s}')
    plt.cla()
    cbar.remove()


def visualize_polygon_radarScene(px, py, px_polygon, py_polygon, conf_polygon, save_folder, frame_id, is_plot_node=True):
    """
    visualize radar polygon
    """
    # create folder if not exist
    directory = os.path.join('./res/results_bev', save_folder)
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
    plt.savefig(f'./res/results_bev/{save_folder:s}/{frame_id:04d}.png')
    plt.cla()
    cbar.remove()


def visualize_polygon_radarScene_withpred(px, py, px_polygon, py_polygon, conf_polygon, px_polygon_pred, py_polygon_pred,
                                          save_folder, frame_id, is_plot_node=True):
    """
    plot predicted radar polygon on bev
    if is_plotDop = True, also plot the Doppler velocity as arrow
    """
    # create folder if not exist
    directory = os.path.join('./res/results_bev', save_folder)
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

    plt.savefig(f'./res/results_bev/{save_folder:s}/{frame_id:04d}.png')
    plt.cla()
    cbar.remove()

    return iou