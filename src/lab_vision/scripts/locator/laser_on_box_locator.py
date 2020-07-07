#!/usr/bin/env python
# encoding:utf-8
import cv2

class LaserOnBoxLocator(object):

    def __init__(self):
        self.threshold = 230
        self.win_name = ""
        self.kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    def init_track_bar(self, name):
        self.win_name = "LaserOnBoxLacator"
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("threshold:", self.win_name, self.threshold, 255, lambda x: self.update_args("threshold", x))

    def update_args(self, arg_name, value):
        if arg_name == "threshold":
            self.threshold = value

    def get_win_name(self):
        return self.win_name

    def get_trackbar_pos(self, track_bar_name):
        return cv2.getTrackbarPos(track_bar_name, self.get_win_name())

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)

        # 把掩膜来个先闭后开，去掉噪声
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.kernal)

        # 计算最小外接圆
        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(image, contours, -1, (0, 0, 255), 2)
        # print("contour id", i)

        if self.get_win_name():
            cv2.imshow(self.get_win_name(), image)

        # 绘制外切圆
        for contour in contours:
            # 计算面积
            area = cv2.contourArea(contour)
            # 该函数计算曲线长度或闭合轮廓周长。True是否封闭
            perimeter = cv2.arcLength(contour, True)
            if 300 > area > 100:
                # 获取最小的外切圆
                (x, y), radius = cv2.minEnclosingCircle(contour)
                # cv2.circle(binary, (int(x), int(y)), int(radius), (255, 255, 0), 2)
                # print("轮廓面积：{} 周长：{:.3f} 圆心：{} 半径：{:.3f}".format(area, perimeter, (x, y), radius))
                return (x, y), radius

        return None
