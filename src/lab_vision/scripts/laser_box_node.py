#! /usr/bin/env python
# coding=utf-8

import rospy
import sys
from sensor_msgs.msg import Image
from locator.locator_main import LocatorMain

from cv_bridge import CvBridge, CvBridgeError
import cv2
# import common.global_ctl as g_ctl
from itheima_msgs.srv import GetLaserBoxLocator, GetLaserBoxLocatorRequest, GetLaserBoxLocatorResponse

bridge = CvBridge()

rst = None

locator = None


def image_callback(msg):
    if not isinstance(msg, Image): return

    cv_mat = None
    try:
        cv_mat = bridge.imgmsg_to_cv2(msg, "bgr8")
        # cv_mat = bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print(e)

    if cv_mat is None:
        print "cv_mat: None"
        return

    img = cv_mat.copy()

    if locator is None:
        return

    global rst

    # cv2.imshow("img_raw", img)
    rst = locator.run(img)
    #rst = ((20, -10), -30)

    # print("-----------", cv2.__version__, sys.version)
    # half_mat = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    # cv2.imshow("image", half_mat)
    action = cv2.waitKey(10) & 0xFF
    if action == ord('s') or action == ord('S'):
        rst.save_params()
    elif action != 255:
        locator.handle_action(action)


def box_callback(req):
    print "收到定位请求---------box_callback"
    while rst is None:
        rospy.sleep(0.1)

    center_offset = rst[0]
    angle_degree = rst[1]

    response = GetLaserBoxLocatorResponse()

    response.center_offset_x = center_offset[0]
    response.center_offset_y = center_offset[1]
    response.angle_degree = angle_degree
    return response


# def capture_image(locator):


if __name__ == '__main__':
    rospy.init_node("laser_box_node")

    laser_mark_pkg_path = rospy.get_param("~laser_mark_pkg_path", "../")
    # laser_mark_pkg_path = "/home/ty/Workspace/ROS/ros_mini_industry/src/lab_vision"
    # g_ctl.update_debug_mode(True)
    # print "debug_mode: ", g_ctl.is_debug_mode
    camera_info_path = laser_mark_pkg_path + "/data/usb_camera.yml"

    print "laser_mark_pkg_path: ", laser_mark_pkg_path
    locator = LocatorMain(laser_mark_pkg_path, "data/usb_camera.yml")

    subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    service = rospy.Service("/laser/box", GetLaserBoxLocator, box_callback)


    rospy.spin()

    cv2.destroyAllWindows()
