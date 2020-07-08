#!/usr/bin/env python
# encoding:utf-8

import numpy as np
import cv2
import glob


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)

    # 将地板绘制成绿色，参数2为多个轮廓的列表，故需要多套一层[]
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-1)

    print("-------------------")
    # 柱子绘制成蓝色, 参数必须是tuple类型
    for i,j in zip(range(4),range(4,8)):
        print("{}.{} -> {}.{}".format(i, tuple(imgpts[i]), j,tuple(imgpts[j])))
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255,0,0),3)

    # 顶部框用红色
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)

    return img


if __name__ == '__main__':
    fs = cv2.FileStorage("../../data/usb_camera.yml", cv2.FILE_STORAGE_READ)
    mtx = fs.getNode("cameraMatrix").mat()
    dist = fs.getNode("distCoeffs").mat()

    print(mtx)
    print(dist)
    fs.release()

    # 构建对象矩阵， Z=0
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)
    """
    [
        [0,0],[1,0],[2,0] ...
        [0,1],[1,1],[2,2] ...
               ...
    ]
    """

    # 3D立方体的8个角点
    # axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
    axis = np.float32([[0, 0, 0],  [0, 3,  0], [3, 3,  0], [3, 0,  0],
                       [0, 0, -3], [0, 3, -3], [3, 3, -3], [3, 0, -3]])
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    index = 0
    capture = cv2.VideoCapture(2)
    while True:
        _, frame = capture.read()
        img = cv2.flip(frame, 1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (6, 9), None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # 查找旋转向量和平移向量
            retval, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

            # 将3D点投影到图像平面
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            img = draw(img, corners2, imgpts)
        cv2.imshow('img', img)

        k = cv2.waitKey(100) & 0xff
        if k == ord('s'):
            cv2.imwrite('image_pose_{}.png'.format(index), img)
            index += 1
        elif k == 27 or k == ord('q') or k == ord('Q'):
            break

    cv2.destroyAllWindows()