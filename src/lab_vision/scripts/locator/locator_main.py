#!/usr/bin/env python
# encoding:utf-8
import cv2
import numpy as np
from locator.laser_locator import LaserRectLocator
from locator.box_locator import BoxLaserLocator


class LocatorMain():

    def __init__(self):
        self.laser_locator = LaserRectLocator()
        self.box_laser_locator = BoxLaserLocator()

        self.target_rect_area = None
        self.box_center = None

    def run(self, frame):

        # 尝试定位盒子
        box_rst  = self.box_laser_locator.detect(frame)

        img_show = frame.copy()
        h, w, c = img_show.shape
        # 中心画一条线，保证在盒子在中心位置附近
        cv2.line(img_show, (w / 2, 0), (w / 2, h), (50, 255, 50), 1, cv2.LINE_AA)
        cv2.line(img_show, (0, h / 2), (w, h / 2), (255, 50, 255), 1, cv2.LINE_AA)

        if box_rst is None:
            print "-----------没找到盒子，执行定位------------"
            # 2. 如果没检测到盒子，先计算矩形框的包容盒，确定中心位置（根据二值化并查找边缘）
            # 根据矩形框和中心构建坐标系，绘制
            # 计算其偏移后的预测位置，绘制
            laser_rect = self.laser_locator.detect(frame)
            if laser_rect is None:
                print "未检测到矩形激光标定目标"
            else:
                laser_rect_area, rect_center = laser_rect

                # 绘制中心和圆环
                cv2.circle(img_show, tuple(np.int0(rect_center)), 4, (0, 0, 255), -1)
                cv2.circle(img_show, tuple(np.int0(rect_center)), 10, (0, 255, 255), 2)

                points = np.int0(laser_rect_area)
                # 绘制最小有向包容盒
                cv2.drawContours(img_show, [points], 0, (0, 0, 255), 2)
        else:
            print "-----------找到盒子，计算偏移量------------"
            # 1. 检测到盒子，并计算中点（注意用离中心的偏移量来修正x位置） 如果没有最新的标定数据，则要求其有激光实施标定
            target_rect_area , box_center_float = box_rst
            # 绘制最小有向包容盒
            cv2.drawContours(img_show, [np.int0(target_rect_area)], 0, (0, 0, 255), 2)

            # rst_dict 保存了两个点： box_center 是计算出来的盒子中心， laser_circle 激光点中心

            self.target_rect_area = target_rect_area
            self.box_center = np.int0(box_center_float)

        # 计算并绘制预测中点（根据得到的坐标轴）
        if self.target_rect_area is not None:
            cv2.circle(img_show, tuple(self.box_center), 10, (0, 200, 0), 2)  # 圆环
            cv2.circle(img_show, tuple(self.box_center), 3, (100, 100, 0), -1)  # 圆心



        # 然后计算偏移后的预测位置和盒子中点的偏移量

        # （可以和直接在盒子上的点作对比，如果盒子上检测不到点，或者和预期差异较大，则使用预测的点）

        cv2.imshow("image_final", img_show)

        # 最后返回偏移量(x,y)和旋转角度Θ
