#!/usr/bin/env python
# encoding:utf-8
import cv2
import numpy as np


class LaserRectLocator(object):

    def __init__(self):
        self.threshold = 150
        self.win_name = "LaserRectLocator"
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("threshold:", self.win_name, self.threshold, 255, lambda x: self.update_args("threshold", x))

        self.kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        self.image_backup = None

    def update_args(self, arg_name, value):
        if arg_name == "threshold":
            self.threshold = value
            self.detect(self.image_backup)

    def detect(self, image):
        self.image_backup = image

        copy_image = image.copy()

        gray = cv2.cvtColor(copy_image, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)

        # 把掩膜来个先闭后开，去掉噪声
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.kernal)

        # cv2.imshow(self.win_name, binary)

        # 计算最小外接圆
        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for i, cnt in enumerate(contours):
            # 绘制原始曲线
            cv2.drawContours(copy_image, contours, i, (255, 0, 0), 2)
            area = cv2.contourArea(cnt)
            cnt_len = cv2.arcLength(cnt, True)
            approx_curve = cv2.approxPolyDP(cnt, cnt_len * 0.02, True)  # 近似多边形
            curve = approx_curve.shape[0]  # 近似多边形边数

            # print("目标区域面积: [{}]，边个数: [{}]".format(area, curve))

            if area > 50000 and curve <= 5:
                rect = cv2.minAreaRect(cnt)
                target_area = cv2.boxPoints(rect)
                # points = np.int0(target_area)
                # 绘制最小有向包容盒
                # cv2.drawContours(copy_image, [points], 0, (0, 0, 255), 2)
                # 包容盒中心，蓝色
                rect_center = (target_area[0] + target_area[2]) / 2
                # cv2.circle(copy_image, tuple(np.int0(rect_center)), 4, (0, 0, 255), -1)
                # cv2.circle(copy_image, tuple(np.int0(rect_center)), 10, (0, 255, 255), 2)

                return target_area, rect_center

                # 计算并绘制适量矩中心
                # mm = cv2.moments(cnt)
                # if mm['m00'] == 0:
                #     continue
                # cx = mm['m10'] / mm['m00']
                # cy = mm['m01'] / mm['m00']
                # cv2.circle(copy_image, (int(cx), int(cy)), 2, (0, 255, 100), -1)


        return None
        # cv2.imshow("LaserRectLocator-Result", copy_image)
