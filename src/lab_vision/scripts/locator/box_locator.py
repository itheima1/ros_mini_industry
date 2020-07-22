#!/usr/bin/env python
# encoding:utf-8
import cv2
from common.abs_detector import AbsDetector
from collections import Counter


class BoxLaserLocator(AbsDetector):

    def __init__(self):
        super(BoxLaserLocator, self).__init__()
        self.h_min = 40
        self.h_max = 100
        self.s_min = 0  # 教室晚上60, 教室白天83,
        self.s_max = 255
        self.v_min = 54
        self.v_max = 255
        self.kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        self.init_track_bar("BoxLaserLocator")

    def detect(self, image):
        if image is None:
            print "没有获取到图像"
            return
        img_copy = image.copy()

        # 高斯滤波
        # img_copy = cv2.GaussianBlur(img_copy, (3, 3), 0)
        # 均值迁移滤波
        # img_copy = cv2.pyrMeanShiftFiltering(img_copy, 10, 30)

        # 双边滤波器：效果不错，就是太慢
        # Diameter 相邻元素直径范围, 可通过sigmaSpace得到, 一般设置为0
        # sigmaColor 颜色空间范围, 值越大, 合并的色调区域越大
        # sigmaSpace 坐标空间范围, 值越大, 合并的像素范围越广, 如果设置了Diameter, 此值无效
        img_copy = cv2.bilateralFilter(img_copy, 0, 30, 4)

        # 根据颜色提取目标区域提取正方形区域
        hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

        # 显示参数图
        win_name = self.get_win_name()
        if win_name:
            tmp_mask = cv2.resize(mask, None, fx=0.5, fy=0.5)
            cv2.imshow(win_name, tmp_mask)

        # 把绿色区域涂黑，保留盒子
        img_copy[mask != 0] = [0, 0, 0]

        # cv2.imshow("img_copy", img_copy)

        # 盒子区域为白色, 其他涂黑
        mask = cv2.bitwise_not(mask)

        # 把掩膜来个先闭后开，去掉噪声
        dst_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernal)
        # cv2.imshow("dst_img", dst_img)

        _, contours, hierarchy = cv2.findContours(dst_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rst = None
        for i, cnt in enumerate(contours):

            area = cv2.contourArea(cnt)  # 面积
            cnt_len = cv2.arcLength(cnt, True)  # 周长
            approx_curve = cv2.approxPolyDP(cnt, cnt_len * 0.02, True)  # 近似多边形
            curve = approx_curve.shape[0]  # 近似多边形边数

            # 绘制原始曲线
            print("目标区域面积: [{}]，边个数: [{}]".format(area, curve))
            cv2.drawContours(img_copy, contours, i, (255, 0, 0), 2)
            if 220000 > area > 50000:
                # 绘制逼近曲线
                cv2.drawContours(img_copy, [approx_curve], 0, (120, 200, 20), 2)

                rect = cv2.minAreaRect(cnt)
                target_area = cv2.boxPoints(rect)

                rst = target_area, (target_area[0] + target_area[2]) / 2

        # cv2.imshow("box_locator-tmp", img_copy)
        return rst
