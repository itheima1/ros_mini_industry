#!/usr/bin/env python
# encoding:utf-8
import cv2
import numpy as np
from common.geometry_util import *

"""
输入： 图片
输出：
    原料区域盒子位姿列表（小车区域A）
    成品区域盒子位姿列表（小车区域A）
    待取成品位姿列表（传送带感光器区）

盒子位姿：
    中心坐标 (x,y)
    短边向量 (x,y)
"""


def find_box(img_masked, img_color_masked, task_str="default"):
    # 用分水岭技术（漫洪填充）解决连接问题

    rst_lst = []
    # 提取盒子列表
    _, contours, hierarchy = cv2.findContours(img_masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i, cnt in enumerate(contours):
        cv2.drawContours(img_color_masked, contours, i, (0, 0, 255), 1)

        area = cv2.contourArea(cnt)  # 面积
        cnt_len = cv2.arcLength(cnt, True)  # 周长
        approx_curve = cv2.approxPolyDP(cnt, cnt_len * 0.02, True)  # 近似多边形
        curve_count = approx_curve.shape[0]  # 近似多边形边数
        if (5000 < area < 15000) and cv2.isContourConvex(approx_curve):
            # print "---------------------area: ",area
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # 绘制包容盒
            cv2.drawContours(img_color_masked, [box], 0, (255, 255, 0), 2)

            center = np.int0((box[0] + box[2]) / 2)

            cv2.circle(img_color_masked, tuple(center), 3, (0, 255, 0), 2)

            cv2.putText(img_color_masked, "({},{})".format(center[0], center[1]), tuple(center),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 160, 80), 2, cv2.LINE_AA)
            # 对点进行排序
            rst = sort_rect(box)
            vector_x, vector_y = calc_vector(rst)

            if task_str == "line":
                # 调整一下方向
                vector_x, vector_y = (vector_y, -vector_x)

            vector_x = np.int0(vector_x)
            vector_y = np.int0(vector_y)

            # 找到短的边，作为x轴向量
            cv2.arrowedLine(img_color_masked, tuple(center), tuple(center + vector_x), (0, 0, 255), 2, cv2.LINE_AA)
            cv2.arrowedLine(img_color_masked, tuple(center), tuple(center + vector_y), (0, 255, 0), 2, cv2.LINE_AA)

            for j in range(len(rst)):
                # 绘制四个顶点
                cv2.circle(img_color_masked, tuple(rst[j]), 3, (0, 0, 255), 2)
                cv2.putText(img_color_masked, str(j), tuple(rst[j]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1,
                            cv2.LINE_AA)

            # print("面积：{}， 边数：{}，中心点：{} x轴：{}".format(area, curve_count, center, vector_x))

            # 小车上盒子的中心x值可信度高， 所以y方向水平向下
            # 传送带上盒子的中心y值可信度高，所以y方向水平向左
            rst_lst.append([center, vector_y])

    # 按照x由小到大排序
    rst_lst.sort(key=lambda point: point[0][0])
    # print("共 [{}] 个点： {}".format(len(rst_lst), rst_lst))

    return rst_lst
