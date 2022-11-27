import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from scipy.interpolate import make_interp_spline, BSpline
from utils.is_insidePolygon import is_insidePolygon


def smooth_curve(px, py):
    px = np.asarray(px)
    py = np.asarray(py)
    # 300 represents number of points to make between T.min and T.max
    pxnew = np.linspace(px.min(), px.max(), 300)
    spl = make_interp_spline(px, py, k=3)  # type: BSpline
    pynew = spl(pxnew)

    return pxnew, pynew


if __name__ == '__main__':
    # define a polygon
    polygon_x = [610.7993311036789, 0, 216.1956521739131, 1135.660535117057, 2104.929765886288, 1687.3394648829433,
                 2066.61872909699, 1710.3260869565217, 1059.0384615384617, 610.7993311036789]
    polygon_y = [314.1505016722408, 693.4297658862877, 1769.9698996655518, 2130.09364548495, 2103.2759197324413,
                 1340.8862876254182, 1003.7491638795987, 122.59531772575251, 620.6387959866221, 314.1505016722408]

    # define curve trajectories
    traj1_x = [1135.660535117057, 1166.309364548495, 1231.4381270903011, 1319.5535117056857,
               1442.1488294314381, 1572.4063545150502, 1717.9882943143814, 1840.5836120401339, 1993.8277591973247,
               2177.7207357859534, 2399.92474916388, 2564.6622073578596]
    traj1_y = [2057.3026755852843, 1800.61872909699, 1616.7257525083612, 1467.3127090301002, 1310.23745819398,
               1172.3177257525083, 1049.7224080267558, 961.6070234113712, 881.1538461538462, 781.5451505016722,
               689.5986622073578, 628.3010033444816]

    traj2_x = [1135.660535117057, 1127.9983277591973, 1120.336120401338, 1108.8428093645487, 1059.0384615384617,
               1001.5719063545151, 936.4431438127091, 878.9765886287626, 813.8478260869566, 668.2658862876254]
    traj2_y = [2076.458193979933, 1900.227424749164, 1643.5434782608695, 1421.339464882943, 1179.979933110368,
               938.6204013377926, 747.0652173913044, 586.1588628762541, 421.4214046822743, 199.2173913043478]

    traj3_x = [13.1471571906355, 93.60033444816058, 231.52006688963215, 396.2575250836121, 580.1505016722408,
               729.5635451505017, 913.4565217391305, 1032.2207357859531, 1105.0117056856188, 1139.4916387959868]
    traj3_y = [612.9765886287626, 670.4431438127091, 766.2207357859531, 900.309364548495, 1065.046822742475,
               1241.2775919732442, 1517.1170568561872, 1697.1789297658863, 1888.7341137123747, 2049.6404682274247]

    plt.figure(figsize=(6, 6))
    # plot polygon
    plt.plot(polygon_x, polygon_y, '-', color='grey', linewidth=1)
    plt.fill(polygon_x, polygon_y, 'skyblue', alpha=0.9)

    # plot car itself
    rect = mpatches.Rectangle((1043.7140468227426, 1984.5117056856188), 175, 305, fill=False, color="purple",
                              linewidth=2)
    plt.gca().add_patch(rect)

    # plot smoothed curves
    px_traj, py_traj = smooth_curve(traj1_x, traj1_y)
    plt.plot(px_traj, py_traj, '--', color='darkslategrey', linewidth=2)
    traj2_x.reverse()
    traj2_y.reverse()
    px_traj, py_traj = smooth_curve(traj2_x, traj2_y)
    plt.plot(px_traj, py_traj, '--', color='darkslategrey', linewidth=2)
    px_traj, py_traj = smooth_curve(traj3_x, traj3_y)
    plt.plot(px_traj, py_traj, '--', color='darkslategrey', linewidth=2)

    # collision detection of the trajectory samples
    for idx in range(len(traj1_x)):
        if is_insidePolygon(polygon_x, polygon_y, traj1_x[idx], traj1_y[idx]):
            plt.plot(traj1_x[idx], traj1_y[idx], 'o', color='green')
        else:
            plt.plot(traj1_x[idx], traj1_y[idx], 'ro')

    for idx in range(len(traj2_x)):
        if is_insidePolygon(polygon_x, polygon_y, traj2_x[idx], traj2_y[idx]):
            plt.plot(traj2_x[idx], traj2_y[idx], 'o', color='green')
        else:
            plt.plot(traj2_x[idx], traj2_y[idx], 'ro')

    for idx in range(len(traj3_x)):
        if is_insidePolygon(polygon_x, polygon_y, traj3_x[idx], traj3_y[idx]):
            plt.plot(traj3_x[idx], traj3_y[idx], 'o', color='green')
        else:
            plt.plot(traj3_x[idx], traj3_y[idx], 'ro')

    plt.axis([-50, 2200, 2300, 0])
    plt.axis('off')
    plt.show()
