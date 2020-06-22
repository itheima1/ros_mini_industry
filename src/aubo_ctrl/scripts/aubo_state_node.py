#! /usr/bin/env python
# coding=utf-8

from lib.robotcontrol import *
from time import *
from math import degrees
from std_msgs.msg import Float32MultiArray


import rospy


def parse_point(point):
    joints = []

    for j in point['joint']:
        joints.append(degrees(j))

    return joints


if __name__ == '__main__':
    rospy.init_node("aubo_state_node")

    ip = rospy.get_param("~aubo_host", "192.168.1.102")
    port = rospy.get_param("~aubo_port", 8899)

    publisher = rospy.Publisher("/aubo/joints", Float32MultiArray, queue_size=1000)

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

            msg = Float32MultiArray()
            msg.data = joints
            publisher.publish(msg)

            rate.sleep()

    # 系统退出
    Auboi5Robot.uninitialize()

