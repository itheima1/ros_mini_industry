#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Point,Quaternion
from sensor_msgs.msg import LaserScan,Image
import tf
from math import copysign, sqrt, pow,radians
from collections import deque
# from heimarobot_nav.transform_utils import quat_to_angle, normalize_angle
from LineFollower import LineDetect
import cv2 as cv
from cv_bridge import CvBridge


import PyKDL
from math import pi
from itheima_msgs.msg import CarMoveAction, CarMoveGoal, CarMoveFeedback, CarMoveResult
import threading
import Queue
from actionlib import ServerGoalHandle


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


bridge = CvBridge()


class HeimaRobot:

    def __init__(self):
        # Give the node a name
        # rospy.init_node('heimarobot_move_distance', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        # rospy.on_shutdown(self.shutdown)
        # How fast will we check the odometry values?
        # self.rate = rospy.get_param('~rate', 20)
        self.rate = rospy.Rate(20)

        # Set the distance to travel
        self.test_distance = 0.5 # meters
        self.speed_vel = 0.15 # meters per second
        self.speed_angle = 0.15
        self.safe_distance = 0.20;
        self.tolerance = rospy.get_param('~tolerance', 0.01) # meters
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test_distance = False
        self.start_test_angle = False

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.sub = rospy.Subscriber("scan_filtered", LaserScan, self.laser_detect)
        self.pub = rospy.Publisher("scan_front", LaserScan, queue_size=10)
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        self.pub_img = rospy.Publisher('/opencv_img', Image, queue_size=2);

        self.sub_vision = rospy.Subscriber('/car_img', Image, self.vision_detect);
        self.vision_mat = None

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

        self.is_moving = False

        self.goal_queue = Queue.Queue()
        threading.Thread(target=self.do_goal).start()

        self.server = actionlib.ActionServer("/slam/move", CarMoveAction, self.goal_cb, self.cancel_cb, False)
        self.server.start()

    def goal_cb(self, handle):
        self.goal_queue.put(handle)

    def cancel_cb(self, handle):
        pass

    def do_goal(self):
        while not rospy.is_shutdown():
            handle = self.goal_queue.get()
            if handle is None:
                continue
            self.do_move(handle)

    def do_move(self, handle):
        if not isinstance(handle, ServerGoalHandle):
            return
        goal = handle.get_goal()
        if not isinstance(goal, CarMoveGoal):
            return
        if goal.type == 0:
            # 移动上料
            self.move_feeding(handle)
        elif goal.type == 1:
            # 移动下料
            self.move_blanding(handle)

    def move_feeding(self, handle):
        if not isinstance(handle, ServerGoalHandle):
            return
        handle.set_accepted()

        # paths = [
        #     {"type":0,"pose":[1.519342,0.425970 ,0,     0,0,-0.674415,0.738352]},
        #     {"type":2,"distance":1.15}
        # ]
        paths = [
            {"type":1,"distance":1.35},
            {"type":3,"angle":radians(-90)},
            {"type":1,"distance":0.15},
        ]


        # for p in paths:
        #     if p["type"] == 0:
        #         rospy.loginfo("================= slam goal ========================")
        #         goal = MoveBaseGoal()
        #         # 指定地图参考系
        #         goal.target_pose.header.frame_id = "map"
        #         goal.target_pose.header.stamp = rospy.Time.now()
        #         # 移动目标设定位姿 xyz和四元数
        #         goal.target_pose.pose.position = Point(p["pose"][0], p["pose"][1], p["pose"][2]);
        #         goal.target_pose.pose.orientation.x = p["pose"][3]
        #         goal.target_pose.pose.orientation.y = p["pose"][4]
        #         goal.target_pose.pose.orientation.z = p["pose"][5]
        #         goal.target_pose.pose.orientation.w = p["pose"][6]
        #         self.move_to_goal(goal);
        #
        #         target = quat_to_angle(Quaternion(p["pose"][3], p["pose"][4], p["pose"][5], p["pose"][6]))
        #         cur_angel = self.get_odom_angle();
        #         print(target, cur_angel, target - cur_angel)
        #         # self.move_rotation(target-cur_angel);
        #     elif p["type"] == 1:
        #         rospy.loginfo("================= slam distance ========================")
        #         self.move_distance(p["distance"])
        #     elif p["type"] == 2:
        #         rospy.loginfo("================= slam line ========================")
        #         self.move_follower_line();
        #     elif p["type"] == 3:
        #         rospy.loginfo("================= slam angle ========================")
        #         self.move_rotation(p["angle"]);

        result = CarMoveResult()
        result.result = "success"
        handle.set_succeeded(result)

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def move_blanding(self, handle):
        if not isinstance(handle, ServerGoalHandle):
            return
        handle.set_accepted()

        paths=[
            {"type":1,"distance":-0.15},
            {"type":3,"angle":radians(90)},
            {"type":1,"distance":-1.35}
        ]

        for p in paths:
            if p["type"] == 0:
                goal = MoveBaseGoal()
                # 指定地图参考系
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                # 移动目标设定位姿 xyz和四元数
                goal.target_pose.pose.position = Point(p["pose"][0], p["pose"][1], p["pose"][2]);
                goal.target_pose.pose.orientation.x = p["pose"][3]
                goal.target_pose.pose.orientation.y = p["pose"][4]
                goal.target_pose.pose.orientation.z = p["pose"][5]
                goal.target_pose.pose.orientation.w = p["pose"][6]
                self.move_to_goal(goal);

                target = quat_to_angle(Quaternion(p["pose"][3], p["pose"][4], p["pose"][5], p["pose"][6]))
                cur_angel = self.get_odom_angle();
                print(target, cur_angel, target - cur_angel)
                # self.move_rotation(target-cur_angel);
            elif p["type"] == 1:
                self.move_distance(p["distance"])
            elif p["type"] == 2:
                self.move_follower_line();
            elif p["type"] == 3:
                rospy.loginfo("================= slam angle ========================")
                self.move_rotation(p["angle"]);


        result = CarMoveResult()
        result.result = "success"
        handle.set_succeeded(result)

        # Stop the robot
        self.cmd_vel.publish(Twist())


    # 导航移动api
    def move_to_goal(self,goal):
        self.start_test_angle = False
        self.start_test_distance = False

        #simpleactionclient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #等待5秒 ,actionserver启动
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("等待move_base actionserver启动")

        rospy.loginfo("发送目标到actionserver ...")

        ac.send_goal(goal)
        #设置超时时间为60s
        ac.wait_for_result(rospy.Duration(60))
        if(ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("成功到达")
            return True
        else:
            rospy.loginfo("未在规定时间内到达目的地,失败了")
            return False


    def move_distance(self,meter):
        print("直线移动")
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()

        x_start = self.position.x
        y_start = self.position.y

        move_cmd = Twist()

        self.test_distance = meter;
        self.start_test_distance = True;
        # Stop the robot by default
        move_cmd = Twist()
        isMoving = False;
        while(self.start_test_distance):

            # Get the current position from the tf transform between the odom and base frames
            self.position = self.get_position()

            # Compute the Euclidean distance from the target point
            distance = sqrt(pow((self.position.x - x_start), 2) +
                            pow((self.position.y - y_start), 2))

            # Correct the estimated distance by the correction factor
            # distance *= self.odom_linear_scale_correction

            # How close are we?
            error =  distance - abs(self.test_distance)


            # Are we close enough?
            if not self.start_test_distance or abs(error) <  self.tolerance:
                self.start_test_distance = False
                move_cmd.linear.x = 0;
                move_cmd.angular.z = 0;
                for i in range(10):
                    self.cmd_vel.publish(move_cmd)
                print("到达目的地")
                isMoving = False;
            else:
                # print("移动")
                # if not isMoving:
                    # If not, move in the appropriate direction
                    # move_cmd.linear.x = copysign(self.speed, -1 * error)
                    move_cmd.linear.x = copysign(self.speed_vel, self.test_distance)
                    self.cmd_vel.publish(move_cmd)
                    # isMoving = True;

            self.rate.sleep();

        rospy.sleep(0.5)
    def verify_angle(self,target):

        self.start_test_angle = True;
        self.angle_tolerance = 0.002;

        isMoving = False


        if self.start_test_angle:
            # Get the current rotation angle from tf
            self.odom_angle = self.get_odom_angle()

            last_angle = self.odom_angle
            turn_angle = 0
            speed_angle = self.speed_angle
            # 当前的度数 和 目标之间的差值
            if self.odom_angle > target:
                speed_angle = - self.speed_angle;

            error = target - self.odom_angle;



            while abs(error) > self.angle_tolerance and self.start_test_angle:
                if rospy.is_shutdown():
                    return


                # Rotate the robot to reduce the error
                move_cmd = Twist()
                move_cmd.angular.z = copysign(self.speed_angle, error)

                if not isMoving:
                    self.cmd_vel.publish(move_cmd)
                    isMoving = True
                self.rate.sleep()

                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()

                # Compute how far we have gone since the last measurement
                delta_angle = normalize_angle(self.odom_angle - last_angle)




                # Add to our total angle so far
                turn_angle += delta_angle

                # Compute the new error
                error = target - self.odom_angle;
                # Store the current angle for the next comparison
                last_angle = self.odom_angle

                print("error：{}，cur:{},target:{}".format(error,self.odom_angle,target))
                self.rate.sleep();
            print("矫正角度已经结束");
            # Stop the robot
            self.cmd_vel.publish(Twist())

            # Update the status flag
            self.start_test_angle = False
            params = {'start_test': False}

        #rospy.sleep(0.5)

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def move_rotation(self,angle):

        self.start_test_angle = True;
        self.angle_tolerance = 0.5;

        self.is_moving = False


        if self.start_test_angle:
            # Get the current rotation angle from tf
            self.odom_angle = self.get_odom_angle()

            last_angle = self.odom_angle
            turn_angle = 0
            # 当前的度数 和 目标之间的差值
            self.test_angle = angle;
            error = self.test_angle - turn_angle



            while abs(error) > self.angle_tolerance and self.start_test_angle:
                if rospy.is_shutdown():
                    return


                # # Rotate the robot to reduce the error
                # move_cmd = Twist()
                # move_cmd.angular.z = copysign(self.speed_angle, error)
                #
                # if not self.is_moving:
                #     self.cmd_vel.publish(move_cmd)
                #     self.is_moving = True
                # self.rate.sleep()
                #
                # # Get the current rotation angle from tf
                # self.odom_angle = self.get_odom_angle()
                #
                # # Compute how far we have gone since the last measurement
                # delta_angle = normalize_angle(self.odom_angle - last_angle)
                #
                #
                # # Add to our total angle so far
                # turn_angle += delta_angle
                #
                # # Compute the new error
                # error = self.test_angle - turn_angle
                # # Store the current angle for the next comparison
                # last_angle = self.odom_angle
                #
                # print("turn_angle：{}，cur:{},error:{}".format(turn_angle,self.odom_angle,error))


                # Rotate the robot to reduce the error
                move_cmd = Twist()
                move_cmd.angular.z = copysign(self.speed_angle, error)
                self.cmd_vel.publish(move_cmd)
                self.rate.sleep()

                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()

                # Compute how far we have gone since the last measurement
                delta_angle = normalize_angle(self.odom_angle - last_angle)

                # Add to our total angle so far
                turn_angle += delta_angle

                # Compute the new error
                error = self.test_angle - turn_angle

                # Store the current angle for the next comparison
                last_angle = self.odom_angle


            print("移动角度结束");
            # Stop the robot
            self.cmd_vel.publish(Twist())

            # Update the status flag
            self.start_test_angle = False
            params = {'start_test': False}



        # Stop the robot
        self.cmd_vel.publish(Twist())

    def vision_detect(self, msg):
        if not isinstance(msg, Image):
            return

        mat = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.vision_mat = mat

    def move_follower_line(self):
        line_detect = LineDetect();
        move_cmd = Twist();
        move_cmd.linear.x = 0;
        move_cmd.angular.z = 0;

        # capture = cv.VideoCapture(2)
        # self.start_test_distance = True
        # if not capture.isOpened():
        #     print("摄像头没有打开")


        while self.start_test_distance:

            if not self.start_test_distance :
                self.rate.sleep()
                continue

            # # 获取图像
            # ok,img = capture.read();
            if self.vision_mat is None:
                self.rate.sleep()
                continue

            img = self.vision_mat

            # 判断方向
            DIRECTION = line_detect.detect_line(img);

            # 将图片数据发送出去
            img_msg = bridge.cv2_to_imgmsg(img,"bgr8");
            self.pub_img.publish(img_msg);


            if DIRECTION == LineDetect.FRONT:
                move_cmd.linear.x = self.speed_vel;
                move_cmd.angular.z = 0

            elif DIRECTION == LineDetect.TURN_LEFT:
                move_cmd.linear.x = self.speed_vel;
                move_cmd.angular.z = self.speed_angle

            elif DIRECTION == LineDetect.TURN_RIGHT:
                move_cmd.linear.x = self.speed_vel;
                move_cmd.angular.z = -self.speed_angle

            else:
                move_cmd.linear.x = 0;
                move_cmd.angular.z = 0;

            if self.start_test_angle or self.start_test_distance:
                self.cmd_vel.publish(move_cmd);
            self.rate.sleep();
            #cv.waitKey(125)

        self.cmd_vel.publish(Twist());
        rospy.sleep(0.5)


    def laser_detect(self, data):

        newdata = data
        #从消息中读取的距离和强度数据是tuple，需要转成list以便操作
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)

        #通过清除不需要的扇区的数据来保留有效的数据
        # for x in range(120,240):
        #     newdata.ranges[x]=0
        #     newdata.intensities[x]=0

        #前方180°的扇区
        #for x in range(90,270):
        #    newdata.ranges[x]=0
        #    newdata.intensities[x]=0

        #正前方60°的扇区
        for x in range(0,30):
           if (self.start_test_distance or self.start_test_angle) and newdata.ranges[x] < self.safe_distance :
                self.cmd_vel.publish(Twist())
                self.start_test_angle = False;
                self.start_test_distance = False;
                print("激光雷达检测到前方即将发生碰撞，主动停止运动！")
        for x in range(330,360):
            if (self.start_test_distance or self.start_test_angle) and newdata.ranges[x] < self.safe_distance :
                self.cmd_vel.publish(Twist())
                self.start_test_angle = False;
                self.start_test_distance = False;
                print("激光雷达检测到前方即将发生碰撞，主动停止运动！")


        # 后方发生碰撞
        for x in range(175,185):
            if (self.start_test_distance or self.start_test_angle) and newdata.ranges[x] < 0.6 :
                self.cmd_vel.publish(Twist())
                self.start_test_angle = False;
                self.start_test_distance = False;
                print("激光雷达检测到 后方 即将发生碰撞，主动停止运动！")

        self.pub.publish(newdata)

    def isMoving(self):
        return self.is_moving

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)

    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))

    def stop(self):
        self.cmd_vel.publish(Twist())





class HeimaRobotNode:
    def __init__(self):
        # Give the node a name
        rospy.init_node('heimarobot_move_distance', anonymous=False)
        rospy.loginfo("Stopping the robot... abcd")
        self.robot = HeimaRobot();
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(20)


    def run(self):

        quene = deque();
        # 3.174344 Y:0.055265 orientation x:0.000000 y:0.000000 z:-0.118238 w:0.992985
        # X:2.252358 Y:0.263060 orientation x:0.000000 y:0.000000 z:-0.105617 w:0.994407
        # X:1.519342 Y:0.425970 orientation x:0.000000 y:0.000000 z:-0.674415 w:0.738352
        # quene.append({"type":0,"pose":[3.174344,0.055265,0,     0,0,-0.118238,0.992985]});
        quene.append({"type":0,"pose":[1.519342,0.425970 ,0,     0,0,-0.674415,0.738352]});
        quene.append({"type":2,"distance":1.15});


        quene.append({"type":1,"distance":-0.6});
        quene.append({"type":0,"pose":[1.519342,0.425970 ,0,     0,0,-0.674415,0.738352]});



        while not rospy.is_shutdown():

            # if len(quene)>0:
            #     p = quene.popleft();
            #
            #     if p["type"] == 0:
            #         goal = MoveBaseGoal()
            #         #指定地图参考系
            #         goal.target_pose.header.frame_id = "map"
            #         goal.target_pose.header.stamp = rospy.Time.now()
            #         #移动目标设定位姿 xyz和四元数
            #         goal.target_pose.pose.position = Point(p["pose"][0],p["pose"][1],p["pose"][2]);
            #         goal.target_pose.pose.orientation.x = p["pose"][3]
            #         goal.target_pose.pose.orientation.y = p["pose"][4]
            #         goal.target_pose.pose.orientation.z = p["pose"][5]
            #         goal.target_pose.pose.orientation.w = p["pose"][6]
            #         self.robot.move_to_goal(goal);
            #
            #         target = quat_to_angle(Quaternion(p["pose"][3],p["pose"][4],p["pose"][5],p["pose"][6]))
            #         cur_angel = self.robot.get_odom_angle();
            #
            #         # print(target,cur_angel,target - cur_angel);
            #         #
            #         #
            #         # self.robot.move_rotation(radians(-90));
            #         # self.verify_angle(1.094828244674173)
            #
            #     elif p["type"] == 1:
            #         self.robot.move_distance(p["distance"]);
            #
            #     elif p["type"] == 2:
            #         # 根据黄色的图标进行移动
            #         self.robot.move_follower_line();
            self.rate.sleep()

        # Stop the robot
        self.robot.stop()




    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.robot.stop()
        rospy.sleep(1)




if __name__ == '__main__':
    try:
        robotnode = HeimaRobotNode();

        # robotnode.run();

        # for p in path1:
        #     goal = MoveBaseGoal()
        #     #指定地图参考系
        #     goal.target_pose.header.frame_id = "map"
        #     goal.target_pose.header.stamp = rospy.Time.now()
        #     #移动目标设定位姿 xyz和四元数
        #     goal.target_pose.pose.position = Point(p[0],p[1],p[2]);
        #     goal.target_pose.pose.orientation.x = p[3]
        #     goal.target_pose.pose.orientation.y = p[4]
        #     goal.target_pose.pose.orientation.z = p[5]
        #     goal.target_pose.pose.orientation.w = p[6]
        #     robot.move_to_goal(goal);
        rospy.spin()
    except Exception as e:
        print(e);


