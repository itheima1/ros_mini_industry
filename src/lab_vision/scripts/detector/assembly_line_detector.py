#! /usr/bin/env python
# coding=utf-8

import cv2
import numpy as np

lower = 40
upper = 100
sss = 83
vvv = 54
kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

def find_assembly_line(image):
    height, width = image.shape[:2]
    # 根据颜色提取目标区域提取正方形区域
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (lower, sss, vvv), (upper, 255, 255))
    # cv2.imshow("bin_img", mask)
    copy = image.copy()
    copy[mask != 0] = [0, 0, 0]

    dst_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
    # cv2.imshow("dst_img", dst_img)

    _, contours, hierarchy = cv2.findContours(dst_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    target_area = None
    contour = None
    for i, cnt in enumerate(contours):
        # 绘制原始曲线
        cv2.drawContours(copy, contours, i, (255, 0, 0), 2)

        # # 轴对齐包容盒
        # x, y, w, h = cv2.boundingRect(cnt)
        # mm = cv2.moments(cnt)
        # if mm['m00'] == 0:
        #     continue
        # cx = mm['m10'] / mm['m00']
        # cy = mm['m01'] / mm['m00']
        #
        # # 绘制矩形框
        # cv2.rectangle(copy, (x, y), (x + w, y + h), (255, 255, 0), 3)
        # # 绘制中心
        # cv2.circle(copy, (np.int(cx), np.int(cy)), 2, (0, 0, 255), -1)

        area = cv2.contourArea(cnt)  # 面积
        cnt_len = cv2.arcLength(cnt, True)  # 周长
        approx_curve = cv2.approxPolyDP(cnt, cnt_len * 0.02, True)  # 近似多边形
        curve = approx_curve.shape[0]  # 近似多边形边数

        if area > 100000:
            print("目标区域面积: [{}]，边个数: [{}]".format(area, curve))
            rect = cv2.minAreaRect(cnt)
            target_area = cv2.boxPoints(rect)
            target_area = np.int0(target_area)
            contour = cnt
            # 绘制逼近曲线
            cv2.drawContours(copy, [approx_curve], 0, (120, 200, 20), 2)

            # 绘制最小有向包容盒
            cv2.drawContours(copy, [target_area], 0, (0, 0, 255), 2)

    # cv2.imshow("copy_dst", copy)

    if target_area is None:
        print("未发现目标区域")
    else:
        # x, y, w, h = cv2.boundingRect(target_area)
        # print(x, y, w, h)
        print(target_area)
        # 把这个矩形区域的高度缩小到90% (切掉上下边缘)

        roi_mask = np.zeros([height, width], np.uint8)
        roi_mask = cv2.fillPoly(roi_mask, [target_area], (255, 255, 255))

        img_masked = cv2.bitwise_and(dst_img, dst_img, mask=roi_mask)
        img_color_masked = cv2.bitwise_and(image, image, mask=roi_mask)

        img_masked[roi_mask == 0] = [255]

        # roi_mask = dst_img[y: y + h, x: x + w]
        # cv2.imshow("img_masked", img_masked)
        return img_masked, img_color_masked

    return None
