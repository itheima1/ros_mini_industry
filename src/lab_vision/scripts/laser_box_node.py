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
    print "-------------------------------------------image_callback!1", type(msg)
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
    print "-------------------------------------------image_callback!2", type(msg)

    # cv2.imshow("img_raw", img)
    rst = locator.run(img)

    # print("-----------", cv2.__version__, sys.version)
    # half_mat = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    # cv2.imshow("image", half_mat)
    print "-------------------------------------------image_callback!3", type(msg)
    cv2.waitKey(10)


def box_callback(req):
    print "收到定位请求---------box_callback"
    while rst is None:
        rospy.sleep(0.1)

    center_offset = rst[0]
    angle_degree = rst[1]

    response = GetLaserBoxLocatorResponse()

    response.center_offset = center_offset
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

    print "laser_mark_pkg_path: ", camera_info_path
    locator = LocatorMain(camera_info_path)

    subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    service = rospy.Service("/laser/box", GetLaserBoxLocator, box_callback)

    # threading.Thread(target=capture_image, args=(locator, )).start()

    # global rst
    # capture = cv2.VideoCapture(2)
    # if not capture.isOpened():
    #     print "相机无法打开"
    # else:
    #     capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #     capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    #     while True:
    #         _, frame = capture.read()
    #         rst = locator.run(frame)
    #         print "-----------------"
    #
    #         action = cv2.waitKey(10) & 0xFF
    #         # if action == ord("q") or action == 27:
    #         #     break
    # capture.release()

    rospy.spin()

    cv2.destroyAllWindows()
