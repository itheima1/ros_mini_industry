#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
根据物体坐标点和像素坐标点估算物体姿态，并建立坐标系

1. 加载相机内参矩阵、畸变系数
2. 加载图片
3. 查找每个图片的角点
4. 查找角点亚像素
5. 计算对象姿态solvePnpRansac
6. 投影3D点到图像平面
7. 在图片上坐标系并显示图片
"""
import cv2
import rospy
from pose_estimator import PoseEstimator
from geometry_msgs.msg import Twist, Point,Quaternion
from move_path import HeimaRobot
from math import radians

class PoseEstimatorRobot:

    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        # Give the node a name
        rospy.init_node('heimarobot_move_distance', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        # self.rate = rospy.get_param('~rate', 20)
        self.rate = rospy.Rate(20)

        self.robot = HeimaRobot();


    def run(self):
        estimator = PoseEstimator("./usb_camera.yml")

        # 2. 加载图片
        capture = cv2.VideoCapture(2)
        if not capture.isOpened():
            print("摄像头打开失败！")
            return

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

        try:
            while not rospy.is_shutdown():
                _, frame = capture.read()
                if self.robot.isMoving():
                    continue
                rst, image, pose = estimator.estimate(frame)
                cv2.imshow("img", image)

                if rst:
                    print(("xyz-rpy: " + "{:>8.3f}," * 6).format(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))

                    self.robot.move_rotation(radians(-pose[4]))


                else:
                    print("未发现目标")
                    #self.robot.move_rotation(radians(90))
                    self.robot.stop()


                action = cv2.waitKey(30) & 0xff
                if action == ord('s'):
                    cv2.imwrite("save_line.jpg", image)
                elif action == 27:
                    break
                self.rate.sleep();
        except KeyboardInterrupt as e:
            print("Ctrl + c 主动停止了程序")
        finally:
            capture.release()
            cv2.destroyAllWindows()


    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    pose = PoseEstimatorRobot();
    pose.run()