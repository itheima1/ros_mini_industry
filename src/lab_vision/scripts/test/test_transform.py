#!/usr/bin/env python
# encoding:utf-8

import cv2
import numpy as np

camera_info_path = "../../data/usb_camera.yml"

depth = 280.0 / 1000.0
def point_to_3d(p):
    z = depth
    x = (p[0] - cx) * z / fx
    y = (p[1] - cy) * z / fy
    return [x, y, z]


if __name__ == '__main__':

    fs = cv2.FileStorage(camera_info_path, cv2.FILE_STORAGE_READ)
    mtx = fs.getNode("cameraMatrix").mat()
    dist = fs.getNode("distCoeffs").mat()
    print "相机内参", mtx
    print "畸变系数", dist
    fs.release()

    fx = mtx[0, 0]
    fy = mtx[1, 1]
    cx = mtx[0, 2]
    cy = mtx[1, 2]
    print fx, fy, cx, cy

    np.set_printoptions(precision=6, suppress=True)

    # TODO: 计算三维空间偏移量
    # 1. 将两个坐标系点、开始和结束点转成3维
    laser_rect = np.array([[417, 397], [579, 410], [393, 709]])  # center, x, y
    points = np.array([[486, 560], [437, 544]])

    a_array = np.zeros((0, 3))
    # 2. 构建坐标系
    for point in laser_rect:
        c_point = point_to_3d(point)
        a_array = np.row_stack((a_array, c_point))

    point_array = np.zeros((0, 3))
    for point in points:
        c_point = point_to_3d(point)
        point_array = np.row_stack((point_array, c_point))

    vector_x = a_array[1] - a_array[0]
    vector_x = vector_x / np.sqrt(np.sum(vector_x**2))

    vector_y = a_array[2] - a_array[0]
    vector_y = vector_y / np.sqrt(np.sum(vector_y**2))

    vector_z = np.array([0, 0, 1])
    vector_z = vector_z / np.sqrt(np.sum(vector_z**2))

    R = np.array([
        [vector_x[0], vector_y[0], vector_z[0]],
        [vector_x[1], vector_y[1], vector_z[1]],
        [vector_x[2], vector_y[2], vector_z[2]]
    ])
    translation = a_array[0]

    transform_a2o = np.eye(4)
    transform_a2o[:3, :3] = R
    transform_a2o[:3,  3] = translation

    transform_o2a = np.eye(4)
    Rt = R.T
    transform_o2a[:3, :3] = Rt
    transform_o2a[:3,  3] = - Rt.dot(translation.T)

    print point_array
    print "R" + "-" * 30
    print R
    print "a_array " + "-" * 30
    print a_array
    print "transform_a2o" + "-" * 30
    print transform_a2o
    print "transform_o2a" + "-" * 30
    print transform_o2a

    # 3. 计算偏移的向量，映射到到激光坐标系B
    vector = np.hstack(((point_array[1] - point_array[0]), 1))
    print vector * 1000

    rst = transform_o2a.dot(np.hstack((point_array[1], 1))) - transform_o2a.dot(np.hstack((point_array[0], 1)))
    offset_rst = rst[:2] * 1000
    print offset_rst