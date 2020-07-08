#!/usr/bin/env python
# encoding:utf-8
import cv2
import numpy as np
from common.abs_detector import AbsDetector


class LaserRectLocator(AbsDetector):

    def __init__(self):
        super(LaserRectLocator, self).__init__()
        self.h_min = 40
        self.h_max = 100
        self.s_min = 235  # 教室晚上60, 教室白天83,
        self.s_max = 255
        self.v_min = 54
        self.v_max = 255

        # self.threshold = 200
        # self.win_name = "LaserRectLocator"
        # cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        # cv2.createTrackbar("threshold:", self.win_name, self.threshold, 255, lambda x: self.update_args("threshold", x))
        self.kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        self.init_track_bar("LaserRect")
        self.image_backup = None

        self.binary_merge = None
        self.image_merge = None
        self.merge_count_max = 50
        self.merge_count = 0

    # def update_args(self, arg_name, value):
    #     if arg_name == "threshold":
    #         self.threshold = value
    #         self.detect(self.image_backup)

    def angle_cos(self, p0, p1, p2):
        d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
        return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

    def detect(self, image):
        self.image_backup = image

        copy_image = image.copy()

        # 只取中间的区域进行判断，以节省时间
        copy_image = cv2.GaussianBlur(copy_image, (3, 3), 0)

        # gray = cv2.cvtColor(copy_image, cv2.COLOR_BGR2GRAY)
        # ret, mask = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)
        # mask = cv2.adaptiveThreshold(gray, self.threshold, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 5 * 5, 3)

        # 这里或者用hsv效果也不错
        hsv = cv2.cvtColor(copy_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

        # 取反
        mask = cv2.bitwise_not(mask)

        # 把掩膜来个先闭后开，去掉噪声
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernal)

        # print "合并次数", self.merge_count

        if self.merge_count > self.merge_count_max:
            self.merge_count = 0
            self.binary_merge = None
            self.image_merge = None

        if self.binary_merge is None or self.image_merge is None:
            self.binary_merge = mask.copy()
            self.image_merge = copy_image.copy()

        self.binary_merge = cv2.add(self.binary_merge, mask)
        self.image_merge = cv2.add(self.image_merge, copy_image, mask)
        self.merge_count += 1

        if self.win_name:
            tmp_mask = cv2.resize(mask, None, fx=0.5, fy=0.5)
            cv2.imshow(self.win_name, tmp_mask)
            cv2.imshow(self.win_name + "-merge", self.binary_merge)

        _, contours, hierarchy = cv2.findContours(self.binary_merge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rst = None
        for i, cnt in enumerate(contours):
            # 绘制原始曲线
            cv2.drawContours(copy_image, contours, i, (255, 0, 0), 2)
            area = cv2.contourArea(cnt)
            cnt_len = cv2.arcLength(cnt, True)
            approx_curve = cv2.approxPolyDP(cnt, cnt_len * 0.02, True)  # 近似多边形
            curve = approx_curve.shape[0]  # 近似多边形边数
            # print("目标区域面积: [{}]，边个数: [{}]".format(area, curve))

            # 把最近10次检测到的多边形进行加强
            if 80000 > area > 20000 and 4 <= curve <= 5:
                # cv2.convexHull()
                rect = cv2.minAreaRect(cnt)
                target_area = cv2.boxPoints(rect)
                # 绘制逼近曲线
                # cv2.drawContours(copy_image, [approx_curve], 0, (0,255, 0), 2)
                # 绘制最小有向包容盒
                # points = np.int0(target_area)
                # cv2.drawContours(copy_image, [points], 0, (0, 0, 255), 2)
                # 包容盒中心，蓝色
                rect_center = (target_area[0] + target_area[2]) / 2
                # cv2.circle(copy_image, tuple(np.int0(rect_center)), 4, (0, 0, 255), -1)
                # cv2.circle(copy_image, tuple(np.int0(rect_center)), 10, (0, 255, 255), 2)

                approx_curve = approx_curve.reshape(-1, 2)
                max_cos = np.max([self.angle_cos(approx_curve[i], approx_curve[(i+1) % 4], approx_curve[(i+2) % 4] ) for i in xrange(4)])
                if max_cos < 0.1: # 最大角是直角
                    rst = target_area, rect_center
                # 计算并绘制适量矩中心
                # mm = cv2.moments(cnt)
                # if mm['m00'] == 0:
                #     continue
                # cx = mm['m10'] / mm['m00']
                # cy = mm['m01'] / mm['m00']
                # cv2.circle(copy_image, (int(cx), int(cy)), 2, (0, 255, 100), -1)

        # cv2.imshow("LaserRectLocator-Result", copy_image)
        # cv2.imshow("LaserRectLocator-Result_merge", self.image_merge)

        return rst
