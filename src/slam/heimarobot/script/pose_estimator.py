#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
from common.transform_tools import *

class PoseEstimator(object):

    def __init__(self, camera_params_path):

        # 1. 加载相机内参矩阵、畸变系数
        fs = cv2.FileStorage(camera_params_path, cv2.FileStorage_READ)
        self.cameraMatrix = fs.getNode("cameraMatrix").mat()
        self.distCoeffs = fs.getNode("distCoeffs").mat()
        print("cameraMatrix: \n", self.cameraMatrix)
        print("distCoeffs: \n", self.distCoeffs)

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        self.find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        self.pattern_size = (9, 6)
        self.object_points = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.object_points[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * 20
        self.axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, 3]]).reshape(-1, 3) * 20

    def estimate(self, frame):
        # img = cv2.flip(frame, 1)
        img = frame.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 3. 查找每个图片的角点
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None, flags=self.find_flags)
        if ret:
            # 4. 查找角点亚像素
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            cv2.drawChessboardCorners(img, self.pattern_size, corners, ret)

            # 5. 计算对象姿态solvePnpRansac
            retval, rvecs, tvecs, inliners = cv2.solvePnPRansac(self.object_points, corners2, self.cameraMatrix,
                                                                self.distCoeffs)

            if retval:
                # 6. 投影3D点到图像平面
                image_points, jacobian = cv2.projectPoints(self.axis, rvecs, tvecs, self.cameraMatrix, self.distCoeffs)

                # 7. 在图片上坐标系并显示图片
                img = self.draw(img, corners2, image_points)

                mat, jac = cv2.Rodrigues(rvecs)
                euler_angles = rotationMatrixToEulerAngles(mat)
                # print("tvecs: ", tvecs.ravel())
                # print("euler: ", np.rad2deg(euler_angles))
                pose = np.hstack([tvecs.ravel(), np.rad2deg(euler_angles)])

                return True, img, pose

        return False, img, None

    def draw(self, img, corners, image_points):
        corner = tuple(corners[0].ravel())
        cv2.line(img, corner, tuple(image_points[0].ravel()), (0, 0, 255), 5)
        cv2.line(img, corner, tuple(image_points[1].ravel()), (0, 255, 0), 5)
        cv2.line(img, corner, tuple(image_points[2].ravel()), (255, 0, 0), 5)
        return img
