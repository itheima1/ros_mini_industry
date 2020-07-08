#!/usr/bin/env python
# encoding:utf-8

import numpy as np
from operator import itemgetter


def sort_rect(curve):
    """
    对四边形的四个点进行排序
    :param curve:
    :return:
    """
    # 按照x正序排列
    rst_x = sorted(curve, key=itemgetter(0))
    # print(repr(rst_x))
    # 取x最小的2个，根据y正序列，作为0，1
    rect_left = sorted(rst_x[:2], key=itemgetter(1))
    # 把剩余的2个，根据y倒序列，作为3，4
    rect_right = sorted(rst_x[-2:], key=itemgetter(1), reverse=True)
    rst = rect_left + rect_right
    return rst


def calc_vector(rect, refer_vector=np.array([0, 1])):
    """
    根据4个点，计算离参考的方向最近的作为Y向量
    :param rect: 四个点
    :param refer_vector: 参考向量，默认为Y轴正方向
    :return: x和y向量
    """
    vector_ab = rect[1] - rect[0]
    vector_ad = rect[3] - rect[0]

    norm_ab = np.linalg.norm(vector_ab)
    norm_ad = np.linalg.norm(vector_ad)

    norm_refer = np.linalg.norm(refer_vector)

    cosangle_ab = vector_ab.dot(refer_vector) / (norm_ab * norm_refer)
    cosangle_ad = vector_ad.dot(refer_vector) / (norm_ad * norm_refer)

    if cosangle_ab > cosangle_ad:
        vector_x = vector_ad
        vector_y = vector_ab
    else:
        vector_x = vector_ab
        vector_y = vector_ad

    return (vector_x, vector_y)
