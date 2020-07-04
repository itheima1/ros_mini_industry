#!/usr/bin/env python
# coding:utf-8

import actionlib
import rospy
import Queue
import threading

from actionlib import ServerGoalHandle

from itheima_msgs.msg import LaserMarkAction, LaserMarkGoal, LaserMarkFeedback, LaserMarkResult
from driver import LaserDriver

# 请求队列
goal_queue = Queue.Queue()


def goal_cb(goal_handle):
    goal_queue.put(goal_handle)


def cancel_cb(goal_handle):
    rospy.loginfo("---------------cancel cb---------------")


def do_work(goal_handle):
    if not isinstance(goal_handle, ServerGoalHandle): return

    goal_handle.set_accepted()

    response = driver.send(id=1, type=1, name="hello")
    print "get result ========="
    print response

    result = LaserMarkResult()
    result.result = "success"
    goal_handle.set_succeeded(result)


def do_goal():
    while not rospy.is_shutdown():
        goal = goal_queue.get()
        if goal is None:
            continue

        do_work(goal)


if __name__ == '__main__':
    rospy.init_node("laser_mark_node")

    host = rospy.get_param("~host", "192.168.1.10")
    port = rospy.get_param("~port", 8899)

    threading.Thread(target=do_goal).start()

    driver = LaserDriver(host, port)
    driver.connect()

    # 创建Action的服务端
    server = actionlib.ActionServer("/laser/mark", LaserMarkAction, goal_cb=goal_cb, cancel_cb=cancel_cb,
                                    auto_start=False)
    server.start()

    # 开启轮询器阻塞线程
    rospy.spin()
    driver.disconnect()
    goal_queue.put(None)

