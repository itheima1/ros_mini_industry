#! /usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np
from operator import itemgetter
from itheima_msgs.srv import GetBoxPoses, GetBoxPosesRequest, GetBoxPosesResponse
from itheima_msgs.msg import BoxPose


bridge = CvBridge()
lower = 40
upper = 80
sss = 83
vvv = 54
kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
mat = None

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

    box = None
    for i, cnt in enumerate(contours):
        cv2.drawContours(copy, contours, i, (255, 0, 0), 3)

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

        if area > 50000:
            print(area, curve)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # 绘制包容盒
            cv2.drawContours(copy, [box], 0, (0, 0, 255), 2)
            # cv2.drawContours(copy, [approx_curve], 0, (100, 0, 255), 2)

    # cv2.imshow("copy_dst", copy)

    if box is None:
        print("未发现目标区域")
    else:
        x, y, w, h = cv2.boundingRect(box)
        print(x, y, w, h)
        roi_mask = np.zeros([height, width], np.uint8)
        roi_mask = cv2.fillPoly(roi_mask, [box], (255, 255, 255))

        img_masked = cv2.bitwise_and(dst_img, dst_img, mask = roi_mask)
        img_color_masked = cv2.bitwise_and(image, image, mask = roi_mask)

        img_masked = cv2.bitwise_not(img_masked)

        # roi_mask = dst_img[y: y + h, x: x + w]
        # cv2.imshow("img_masked", img_masked)
        return img_masked, img_color_masked

    return None


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
def calc_vector_x(rect):
    """
    根据4个点，计算短边向量(朝上)
    :param rst: 四个点
    :return: 短边向量
    """
    vector_ab = rect[1] - rect[0]
    norm_ab = np.linalg.norm(vector_ab)
    vector_ad = rect[3] - rect[0]
    norm_ad = np.linalg.norm(vector_ad)

    if norm_ab < norm_ad:
        vector_x = vector_ab
        vector_y = vector_ad
    else:
        vector_x = vector_ad
        vector_y = vector_ab

    return (-vector_x, vector_y)



def find_box(img_masked, img_color_masked):
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
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # 绘制包容盒
            cv2.drawContours(img_color_masked, [box], 0, (255, 255, 0), 2)

            center = np.int0((box[0] + box[2]) / 2)
            print("面积：{}， 边数：{}，中心点：{}".format(area, curve_count, center))

            cv2.circle(img_color_masked, tuple(center), 3, (0, 255, 0), 2)

            # 对点进行排序
            rst = sort_rect(box)
            vector_x, vector_y = calc_vector_x(rst)
            vector_x = np.int0(vector_x)
            vector_y = np.int0(vector_y)

            for j in range(len(rst)):
                # 绘制中心
                cv2.circle(img_color_masked, tuple(rst[j]), 3, (0, 0, 255), 2)
                cv2.putText(img_color_masked, str(j), tuple(rst[j]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1,
                            cv2.LINE_AA)

            # 找到短的边，作为x轴向量
            cv2.arrowedLine(img_color_masked, tuple(center), tuple(center + vector_x), (0, 0, 255), 2, cv2.LINE_AA)
            cv2.arrowedLine(img_color_masked, tuple(center), tuple(center + vector_y), (0, 255, 0), 2, cv2.LINE_AA)

            center[1] = center[1] + 540

            rst_lst.append((center, vector_x))
    # cv2.imshow("img_color_masked", img_color_masked)

    return rst_lst


def image_callback(msg):
    if not isinstance(msg, Image): return

    global mat
    mat = bridge.imgmsg_to_cv2(msg, "bgr8")


def box_callback(req):
    while mat is None:
        rospy.sleep(0.01)

    img = mat.copy()
    # image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    h, w, c = img.shape
    print("width: {}, height: {}, channel: {}".format(w, h, c))
    # 图片感兴趣区域提取
    img = img[int(h / 2):, :]
    # 提取流水线区域
    img_masked, img_color_masked = find_assembly_line(img)

    # cv2.imshow("img_masked", img_masked)
    # 在流水线指定区域找盒子
    rst_lst = find_box(img_masked, img_color_masked)
    # 打印盒子列表 [(center, vector_x),(center, vector_x) ...]
    # cv2.imshow("image", img)
    # cv2.waitKey(1)

    response = GetBoxPosesResponse()
    for item in rst_lst:
        center = item[0]
        vector_x = item[1]

        pose = BoxPose()
        pose.center.append(center[0])
        pose.center.append(center[1])
        pose.vector_x.append(vector_x[0])
        pose.vector_x.append(vector_x[1])

        response.poses.append(pose)

    return response


if __name__ == '__main__':
    rospy.init_node("box_locator_node")

    subscriber = rospy.Subscriber("/kinect2/hd/image_color", Image, image_callback)

    service = rospy.Service("/box/poses", GetBoxPoses, box_callback)

    rospy.spin()
