#!/usr/bin/env python
# -*- coding: utf-8 -*-


import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose,PoseWithCovarianceStamped,Point,Quaternion,Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

from random import  sample
from math import pow, sqrt, trunc


class NavigationTest():

    def __init__(self):
        rospy.init_node("exploring",anonymous=True);
        rospy.on_shutdown(self.shutdown)

        # 在每个目标位置暂停的时间
        self.rest_time = rospy.get_param("~rest_time",2)

        # 当前是否为仿真环境
        self.fake_test = rospy.get_param("~fake_test",True);

        # 到达目标的状态
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # 设置目标点的位置
        # 在rviz中点击2D NAV Goal 按键，然后单击地图中一点
        # 在终端中就会看到该店的坐标信息
        # locations = [Pose(Point(1.13, 0.26, 0.000),  Quaternion(0.000, 0.000, -0.447, 0.894)),
        #              Pose(Point(6.07, -4.75, 0.000),  Quaternion(0.000, 0.000, -0.847, 0.532)),
        #              Pose(Point(6.22, 4.08, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)),
        #              Pose(Point(0.99, 1.86, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)),
        #              Pose(Point(-1.46, 4.09, 0.000), Quaternion(0.000, 0.000, 0.340, 0.940)),
        #              Pose(Point(-6.53, -2.57, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))]
        locations = [Pose(Point(0.5, 0, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000)),
                     Pose(Point(0, 0.5, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000)),
                     ]


        # 发布机器人控制信息
        self.cmd_vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=5);

        # 订阅move_base服务信息
        self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction);

        # 打印日志
        rospy.loginfo("等待连接move base 服务...")

        # 设置等待时间显示
        self.move_base.wait_for_server(rospy.Duration(60))

        # 保存机器人在rviz中的初始位置
        initial_pose = PoseWithCovarianceStamped()

        # 保存机器人运行信息：成功率，运行时间，运行距离
        n_locations = len(locations);
        n_goals = 0
        n_successes = 0
        i = 0
        distance_traveled = 0
        start_time = rospy.Time.now();
        running_time=0
        location=0
        last_location=0;

        # 确保已经有初始位置
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("开始自动导航测试...")

        # 开启主循环，随机导航
        while not rospy.is_shutdown():
            # # 如果已经走完所有的点，再重新开始走
            # if i == n_locations:
            #     i = 0 ;
            #     sequence = sample(locations,n_locations)
            #
            #     # 如果最后一个点和第一个点相同，则调过
            #     if sequence[0] == last_location:
            #         i = 1
            #
            # # 在当前的排序中获取下一个目标点
            # location = sequence[i]
            location = i%len(locations)

            # 跟踪行驶距离
            # 使用初始位置
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x-locations[last_location].position.x,2)
                                +
                                pow(locations[location].position.y-locations[last_location].position.y,2))

            else:
                 rospy.loginfo("更新当前位姿...")
                 distance = sqrt(pow(locations[location].position.x-initial_pose.pose.pose.position.x,2)
                                 +
                                 pow(locations[location].position.y-initial_pose.pose.pose.position.y,2))
                 initial_pose.header.stamp = ""

            # 存储上一个位置，计算距离
            last_location = location

            # 计算器加1
            i += 1
            n_goals += 1

            # 设定下一个目标点
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now();

            # 打印目标位置
            rospy.loginfo("移动到:"+str(location))
            self.move_base.send_goal(self.goal)

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

            # 判断是否成功到达
            if not finished_within_time:
                self.move_base.cancel_goal();
                rospy.loginfo("超时还未到达目标点，取消了当前移动")
            else:
                state = self.move_base.get_state();
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("成功抵达目标点～:"+str(state))
                    n_successes +=1;
                    distance_traveled += distance
                else:
                    rospy.loginfo("移动到目标点失败啦："+str(state))


            # 运行所用的时间
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs/60

            # 输出本次导航的所有信息
            rospy.loginfo("成功的概率:{}/{}={}%".format(str(n_successes),str(n_goals),str(n_successes/n_goals*100)))

            # 运行时间和距离
            rospy.loginfo("运行时间：{}，运行距离：{}m".format(str(trunc(running_time)),str(trunc(distance_traveled))))

            #
            rospy.sleep(self.rest_time);



    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        NavigationTest()
        rospy.spin()
    except Exception as e:
        rospy.loginfo("导航功能已停止：{}".format(e))





