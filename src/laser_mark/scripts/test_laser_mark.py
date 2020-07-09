#!/usr/bin/env python
# coding:utf-8

import rospy
import actionlib
from actionlib import GoalStatus
from actionlib import ClientGoalHandle
from itheima_msgs.msg import LaserMarkAction, LaserMarkGoal, LaserMarkFeedback, LaserMarkResult


def transition_cb(goal_handle):
    rospy.loginfo("---------------transition cb---------------")


def feedback_cb(goal_handle, feedback):
    rospy.loginfo("---------------feedback cb---------------")
    rospy.loginfo(feedback)


if __name__ == '__main__':
    # 创建节点
    rospy.init_node("test_laser_mark", anonymous=True)

    # 创建客户端
    client = actionlib.ActionClient("/laser/mark", LaserMarkAction)
    # 等待连接服务器
    client.wait_for_server()

    # 发送请求
    goal = LaserMarkGoal()
    goal.name = "你好Hello"
    goal.type = 4
    goal.id = 1
    goal_handle = client.send_goal(goal, transition_cb=transition_cb, feedback_cb=feedback_cb)

    client.wait_for_server(rospy.Duration(10))
