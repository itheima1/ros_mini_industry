#!/usr/bin/env python
# coding:utf-8

import actionlib
import rospy


def goal_cb(goal_handle):
    rospy.loginfo("---------------goal cb---------------")


def cancel_cb(goal_handle):
    rospy.loginfo("---------------cancel cb---------------")


if __name__ == '__main__':
    rospy.init_node("laser_mark_node")

    # 创建Action的服务端
    server = actionlib.ActionServer("/laser/mark", CountNumberAction, goal_cb=goal_cb, cancel_cb=cancel_cb,auto_start=False)
    server.start()

    # 开启轮询器阻塞线程
    rospy.spin()