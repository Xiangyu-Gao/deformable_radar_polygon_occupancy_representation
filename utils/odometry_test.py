# set parent directory as sys path
import inspect
import os
import sys
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

import pickle
import math
import matplotlib.pyplot as plt
import numpy as np
from config import radar_configs
from utils.coord_trans import coord_rotate


def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    ---------
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial

    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError as msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order + 1)
    half_window = (window_size - 1) // 2
    # precompute coefficients
    b = np.mat([[k ** i for i in order_range] for k in range(-half_window, half_window + 1)])
    m = np.linalg.pinv(b).A[deriv] * rate ** deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs(y[1:half_window + 1][::-1] - y[0])
    lastvals = y[-1] + np.abs(y[-half_window - 1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))

    return np.convolve(m[::-1], y, mode='valid')


if __name__ == '__main__':
    ego_axis = [[], []]
    yaw = []
    rotation = 0
    old_pose = 0
    with open('aug_05_dynamic_pose_axis', "rb") as fp:  # Unpickling
        ego_data = pickle.load(fp)

    for idx in range(700):
        ego_pose = ego_data[idx]
        yaw.append(ego_pose[2])

    yaw_lps = savitzky_golay(np.squeeze(np.asarray(yaw)), 21, 4)    # window size 51, polynomial order 3
    # plt.plot(np.asarray(yaw), 'g-')
    # plt.plot(np.asarray(yaw_lps), 'r-')
    # plt.show()

    for idx in range(650):
        ego_pose = ego_data[idx]
        vx = ego_pose[0]
        vy = ego_pose[1]
        vx_proj, vy_proj = coord_rotate(vx, vy, rotation)
        print(math.degrees(rotation), rotation, ego_pose[2], ego_pose[3])
        ego_axis[0].append(vx_proj)
        ego_axis[1].append(vy_proj)
        rotation = rotation + yaw_lps[idx] * radar_configs['duration'] * 2.25

    # visualize the trajectory of sensors, plot all points
    plt.plot(np.cumsum(np.asarray(ego_axis[0])), np.cumsum(np.asarray(ego_axis[1])), 'g-')
    plt.show()
    # configure the saved figure
