#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Point,Quaternion
import tf
from math import copysign, sqrt, pow
from collections import deque
from heimarobot_nav.transform_utils import quat_to_angle, normalize_angle

class HeimaRobot:

    def __init__(self):
        # Give the node a name
        rospy.init_node('heimarobot_move_distance', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        # self.rate = rospy.get_param('~rate', 20)
        self.rate = rospy.Rate(20)

        # Set the distance to travel
        self.test_distance = 0.5 # meters
        self.speed = rospy.get_param('~speed', 0.15) # meters per second
        self.tolerance = rospy.get_param('~tolerance', 0.01) # meters
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', False)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

    def run(self):
        # paths = [
        #     {"type":0,"pose":[-1.822970,0.453311,0,     0,0,-0.175287,0.98451]},
        #     {"type":1,"distance":0.5},
        #     {"type":1,"distance":-0.5},
        #     {"type":0,"pose":[-1.822970,0.453311,0,     0,0,-0.175287,0.98451]}
        # ]
        quene = deque();
        # 3.174344 Y:0.055265 orientation x:0.000000 y:0.000000 z:-0.118238 w:0.992985
        # X:2.252358 Y:0.263060 orientation x:0.000000 y:0.000000 z:-0.105617 w:0.994407

        # quene.append({"type":0,"pose":[3.174344,0.055265,0,     0,0,-0.118238,0.992985]});
        quene.append({"type":0,"pose":[2.252358,0.263060 ,0,     0,0,-0.105617,0.994407]});
        quene.append({"type":1,"distance":1.03});
        # quene.append({"type":1,"distance":-1.3});
        # quene.append({"type":0,"pose":[-1.036286,0.257732,0,     0,0,-0.232517,0.972592]});

        while not rospy.is_shutdown():

            if len(quene)>0:
                p = quene.popleft();

                if p["type"] == 0:
                    goal = MoveBaseGoal()
                    #指定地图参考系
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    #移动目标设定位姿 xyz和四元数
                    goal.target_pose.pose.position = Point(p["pose"][0],p["pose"][1],p["pose"][2]);
                    goal.target_pose.pose.orientation.x = p["pose"][3]
                    goal.target_pose.pose.orientation.y = p["pose"][4]
                    goal.target_pose.pose.orientation.z = p["pose"][5]
                    goal.target_pose.pose.orientation.w = p["pose"][6]
                    self.move_to_goal(goal);

                    target = quat_to_angle(Quaternion(p["pose"][3],p["pose"][4],p["pose"][5],p["pose"][6]))
                    cur_angel = self.get_odom_angle();

                    print(target,cur_angel,target - cur_angel);

                    #self.move_rotation(target-cur_angel);


                elif p["type"] == 1:
                    self.move_distance(p["distance"]);

            self.rate.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())


    # 导航移动api
    def move_to_goal(self,goal):
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
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()

        x_start = self.position.x
        y_start = self.position.y

        move_cmd = Twist()

        self.test_distance = meter;
        self.start_test = True;
        # Stop the robot by default
        move_cmd = Twist()
        isMoving = False;
        while(self.start_test):

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
            if not self.start_test or abs(error) <  self.tolerance:
                self.start_test = False
                move_cmd.linear.x = 0;
                move_cmd.angular.z = 0;
                for i in range(10):
                    self.cmd_vel.publish(move_cmd)
                print("到达目的地")
                isMoving = False;
            else:
                if not isMoving:
                    # If not, move in the appropriate direction
                    # move_cmd.linear.x = copysign(self.speed, -1 * error)
                    move_cmd.linear.x = copysign(self.speed, self.test_distance)
                    self.cmd_vel.publish(move_cmd)
                    isMoving = True;


    def move_rotation(self,angle):

        self.start_test_angle = True;
        self.angle_tolerance = 0.1;

        isMoving = True


        if self.start_test_angle:
            # Get the current rotation angle from tf
            self.odom_angle = self.get_odom_angle()

            last_angle = self.odom_angle
            turn_angle = 0
            self.test_angle = angle;
            error = self.test_angle - turn_angle



            while abs(error) > self.angle_tolerance and self.start_test_angle:
                if rospy.is_shutdown():
                    return


                # Rotate the robot to reduce the error
                move_cmd = Twist()
                move_cmd.angular.z = copysign(self.speed, error)

                if not isMoving:
                    self.cmd_vel.publish(move_cmd)
                self.rate.sleep()

                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()

                # Compute how far we have gone since the last measurement
                delta_angle = 1* normalize_angle(self.odom_angle - last_angle)

                # Add to our total angle so far
                turn_angle += delta_angle

                # Compute the new error
                error = self.test_angle - turn_angle
                print("error:",error)
                # Store the current angle for the next comparison
                last_angle = self.odom_angle

            # Stop the robot
            self.cmd_vel.publish(Twist())

            # Update the status flag
            self.start_test_angle = False
            params = {'start_test': False}

        rospy.sleep(0.5)

        # Stop the robot
        self.cmd_vel.publish(Twist())

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



    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)






path1 = [
    [-1.822970,0.453311,0,     0,0,-0.175287,0.98451],
    [-0.109315,-0.315679,0,     0,0,-0.344191,0.938900],
    [-1.822970,0.453311,0,     0,0,-0.175287,0.98451]
]


if __name__ == '__main__':
    try:
        robot = HeimaRobot();

        robot.run();

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
        # rospy.spin()
    except Exception as e:
        print(e);


