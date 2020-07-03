#! /usr/bin/env python
# coding=utf-8

import rospy
import sys
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from detector.assembly_line_detector import find_assembly_line
from detector.box_detector import find_box
from detector.agv_detector import find_agv_desktop

from itheima_msgs.srv import GetBoxPoses, GetBoxPosesRequest, GetBoxPosesResponse
from itheima_msgs.msg import BoxPose

bridge = CvBridge()


def start_find(img):
    # img= pic.copy()
    # image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    h, w, c = img.shape
    print("width: {}, height: {}, channel: {}".format(w, h, c))
    # 提取流水线区域

    print("----------------开始查找传送带盒子--------------")
    assembly_line_box_lst = []
    line_rst = find_assembly_line(img)
    if not line_rst:
        print("未发现流水线，请检查并确认！ <<<<<<<<<<")
    else:
        img_masked, img_color_masked = line_rst
        # cv2.imshow("img_masked", img_masked)
        # 在流水线指定区域找盒子
        assembly_line_box_lst = find_box(img_masked, img_color_masked, "line")

    print("----------------开始查找AVG盒子--------------")
    # 提取AGV小车区域
    agv_box_lst = []
    agv_rst = find_agv_desktop(img)
    if not agv_rst:
        print("未发现AGV小车，请检查并确认！ <<<<<<<<<<")
    else:
        agv_img_masked, agv_img_color_masked = agv_rst
        agv_box_lst = find_box(agv_img_masked, agv_img_color_masked, "agv")

    # 打印盒子列表 [(center, vector_x),(center, vector_x) ...]
    # cv2.imshow("image", img)
    return assembly_line_box_lst, agv_box_lst


rst_lst = None


def image_callback(msg):
    if not isinstance(msg, Image): return

    mat = bridge.imgmsg_to_cv2(msg, "bgr8")

    img = mat.copy()

    global rst_lst
    rst_lst = start_find(img)

    print("-----------", cv2.__version__, sys.version)

    half_mat = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    cv2.imshow("image", half_mat)
    cv2.waitKey(10)


def box_callback(req):
    while rst_lst is None:
        rospy.sleep(0.01)

    assembly_line_box_lst = rst_lst[0]
    agv_box_lst = rst_lst[1]

    response = GetBoxPosesResponse()
    for item in assembly_line_box_lst:
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

    # cv2.namedWindow("bin_img", cv2.WINDOW_AUTOSIZE)
    # cv2.createTrackbar("h_min:", "bin_img", h_min, 255, lambda x: exec("global h_min; h_min = x; filter_by_color()"))
    # cv2.createTrackbar("h_max:", "bin_img", h_max, 255, lambda x: exec("global h_max; h_max = x; filter_by_color()"))
    # cv2.createTrackbar("s_min:", "bin_img", s_min, 255, lambda x: exec("global s_min; s_min = x; filter_by_color()"))
    # cv2.createTrackbar("s_max:", "bin_img", s_max, 255, lambda x: exec("global s_max; s_max = x; filter_by_color()"))
    # cv2.createTrackbar("v_min:", "bin_img", v_min, 255, lambda x: exec("global v_min; v_min = x; filter_by_color()"))
    # cv2.createTrackbar("v_max:", "bin_img", v_max, 255, lambda x: exec("global v_max; v_max = x; filter_by_color()"))


    rospy.spin()

    cv2.destroyAllWindows()
