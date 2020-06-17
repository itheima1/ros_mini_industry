#! /usr/bin/env python
# coding=utf-8

from lib.robotcontrol import *
from time import *
from math import degrees
from Connector import Connector

import rospy


def parse_point(point):
    joints = []

    for j in point['joint']:
        joints.append(degrees(j))

    return joints


if __name__ == '__main__':
    rospy.init_node("aubo_ui_sync_node")

    ip = rospy.get_param("aubo_host", "192.168.0.117")
    port = rospy.get_param("aubo_port", 8899)

    ui_host = rospy.get_param("ui_host", "192.168.0.137")
    ui_port = rospy.get_param("ui_port", 10008)

    connector = Connector(ip=ui_host, port=ui_port)
    connector.connect()

    # 初始化Aubo系统
    Auboi5Robot.initialize()

    # 创建aubo机器人对象
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    result = robot.connect(ip, port)

    if result == RobotErrorType.RobotError_SUCC:
        # 连接成功

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            point = robot.get_current_waypoint()

            joints = parse_point(point)
            # print joints

            connector.send_aubo_joints(joints)

            rate.sleep()

    # 系统退出
    Auboi5Robot.uninitialize()

    connector.disconnect()
