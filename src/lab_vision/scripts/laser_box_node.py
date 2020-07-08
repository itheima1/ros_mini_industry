#! /usr/bin/env python
# coding=utf-8

import rospy
import sys
from sensor_msgs.msg import Image
from locator.locator_main import LocatorMain

from cv_bridge import CvBridge
import cv2
import common.global_ctl as g_ctl
from itheima_msgs.srv import GetLaserBoxLocator, GetLaserBoxLocatorRequest, GetLaserBoxLocatorResponse


bridge = CvBridge()

rst = None


def image_callback(msg):
    if not isinstance(msg, Image): return

    mat = bridge.imgmsg_to_cv2(msg, "bgr8")

    img = mat.copy()

    global rst
    rst = locator.run(img)

    # print("-----------", cv2.__version__, sys.version)
    # half_mat = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    # cv2.imshow("image", half_mat)
    cv2.waitKey(10)


def box_callback(req):
    while rst is None:
        rospy.sleep(0.1)

    center_offset = rst[0]
    angle_degree = rst[1]

    response = GetLaserBoxLocatorResponse()

    response.center_offset = center_offset
    response.angle_degree = angle_degree
    return response


if __name__ == '__main__':
    rospy.init_node("laser_box_node")

    subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    service = rospy.Service("/laser/box", GetLaserBoxLocator, box_callback)

    g_ctl.update_debug_mode(True)
    print "debug_mode: ", g_ctl.is_debug_mode

    locator = LocatorMain()

    rospy.spin()

    cv2.destroyAllWindows()
