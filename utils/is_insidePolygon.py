import numpy as np
import matplotlib.pyplot as plt


def is_insidePolygon(polygon_x, polygon_y, node_x, node_y):
    p_len = len(polygon_x)
    # create matrix P
    P = np.zeros((p_len, p_len))
    Q = np.zeros((p_len, 1))
    R = np.zeros((p_len, 1))
    for idx in range(p_len):
        P[idx, idx] = 1
        P[idx, (idx + 1) % p_len] = -1
        # if max(polygon_x[idx], polygon_x[(idx + 1) % p_len]) > node_x:
        #     Q[idx] = 1
        # else:
        #     Q[idx] = 0
        if max(polygon_y[idx], polygon_y[(idx + 1) % p_len]) > node_y > min(polygon_y[idx], polygon_y[(idx + 1) % p_len]):
            Q[idx] = 1
        else:
            Q[idx] = 0

    Xn = np.asarray(polygon_x).reshape(p_len, 1)
    Yn = np.asarray(polygon_y).reshape(p_len, 1)
    AB = np.asarray([node_x, node_y]).reshape(2, 1)

    F = P.dot(np.concatenate((Yn, -Xn), axis=1)).dot(AB) - np.multiply(P.dot(Yn), Xn) + np.multiply(P.dot(Xn), Yn)
    F_eval = F * P.dot(Yn) * Q
    count_neg = (F_eval < 0).sum()
    # print(F_eval, count_neg)

    if count_neg % 2 == 0:
        return False
    else:
        return True


if __name__ == '__main__':
    polygon_x = [-1, 0, 1, 1, -1]
    polygon_y = [1, 0.5, 1, -1, -1]
    node_x = -0.5
    node_y = 0.5
    # is_insidePolygon(polygon_x, polygon_y, 0, 0)
    print(is_insidePolygon(polygon_x, polygon_y, node_x, node_y))
    plt.plot(polygon_x, polygon_y, 'go')
    plt.plot(node_x, node_y, 'ro')
    plt.show()