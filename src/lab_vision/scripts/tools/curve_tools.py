#!/usr/bin/env python
# encoding:utf-8

import numpy as np


# 多边形周长
# shape of polygon: [N, 2]
def Perimeter(polygon):
    N, d = polygon.shape
    if N < 3 or d != 2:
        raise ValueError

    permeter = 0.
    for i in range(N):
        permeter += np.linalg.norm(polygon[i - 1] - polygon[i])
    return permeter


# 面积
def Area(polygon):
    N, d = polygon.shape
    if N < 3 or d != 2:
        raise ValueError

    area = 0.
    vector_1 = polygon[1] - polygon[0]
    for i in range(2, N):
        vector_2 = polygon[i] - polygon[0]
        area += np.abs(np.cross(vector_1, vector_2))
        vector_1 = vector_2
    return area / 2


# |r| < 1
# r > 0, 内缩
# r < 0, 外扩
def calc_shrink_width(polygon, r):
    area = Area(polygon)
    perimeter = Perimeter(polygon)
    L = area * (1 - r ** 2) / perimeter
    return L if r > 0 else -L


def shrink_polygon(polygon, r):
    N, d = polygon.shape
    if N < 3 or d != 2:
        raise ValueError

    shrinked_polygon = []
    L = calc_shrink_width(polygon, r)
    for i in range(N):
        Pi = polygon[i]
        v1 = polygon[i - 1] - Pi
        v2 = polygon[(i + 1) % N] - Pi

        normalize_v1 = v1 / np.linalg.norm(v1)
        normalize_v2 = v2 / np.linalg.norm(v2)

        sin_theta = np.abs(np.cross(normalize_v1, normalize_v2))

        Qi = Pi + L / sin_theta * (normalize_v1 + normalize_v2)
        shrinked_polygon.append(Qi)
    return np.asarray(shrinked_polygon)

"""
data_cs = data[data[:, 1].argsort()] # 按第2列进行排序
data_rs = data[:, data[1].argsort()] # 按第2行进行排序
"""
def get_rect_range(target_area):
    """
    求出四边形的每个边的中点
    :param target_area:
    :return:
    """
    if not isinstance(target_area, np.ndarray):
        return

    # 按照y进行排序
    target_area_y = target_area[target_area[:, 1].argsort()]
    # print target_area_y
    start_y_p = target_area_y[:2]
    end_y_p = target_area_y[-2:]

    target_area_x = target_area[target_area[:, 0].argsort()]
    # print target_area_x
    start_x_p = target_area_x[:2]
    end_x_p = target_area_x[-2:]

    return np.int0((
        np.mean(start_y_p, axis=0),
        np.mean(end_y_p, axis=0),
        np.mean(start_x_p, axis=0),
        np.mean(end_x_p, axis=0),
    ))


if __name__ == '__main__':
    # vect = np.array([20, -20])
    # vect = vect / np.linalg.norm(vect)
    #
    # print vect
    #
    # target_area = np.array([
    #     [0, 0],
    #     [0, 100],
    #     [50, 100],
    #     [50, 0],
    # ])
    # rst = get_rect_range(target_area)
    # print rst

    for i in range(3):
        print i
        if i == 2:
            break
    else:
        print "haha"

