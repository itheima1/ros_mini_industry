#!/usr/bin/env python
# encoding:utf-8
import cv2
import os
import json
from locator.laser_locator import LaserRectLocator
from locator.box_locator import BoxLaserLocator
from common.geometry_util import *


class LocatorMain():

    def __init__(self,node_path, camera_info, env_name="env1"):
        camera_info_path = os.path.join(node_path, camera_info)
        print "-------------------------------------------------------------camera_info_path: ", camera_info_path

        self.laser_locator = LaserRectLocator()
        self.box_laser_locator = BoxLaserLocator()
        self.output_writer = None

        # 激光中心从传送带到盒子表面的偏移量
        self.offset = np.array([-10.0, 40.0])
        # 参考的激光缩放比例
        self.rect_scale_factor = 8.5
        self.laser_rect_area = None
        self.rect_center = None
        self.frame_count = 0

        self.depth = 280.0 / 1000.0

        fs = cv2.FileStorage(camera_info_path, cv2.FILE_STORAGE_READ)
        self.mtx = fs.getNode("cameraMatrix").mat()
        self.dist = fs.getNode("distCoeffs").mat()
        print "相机内参", self.mtx
        print "畸变系数", self.dist
        fs.release()
        self.node_path = node_path
        self.env_name = env_name
        self.init_params()

    def init_params(self):
        print "init locator_main params ing.................."
        self.laser_locator.load_params(self.node_path, self.env_name)
        self.box_laser_locator.load_params(self.node_path, self.env_name)

        try:
            file_path = os.path.join(self.node_path, "config", self.env_name, '{}.json'.format("locator_offset"))
            if not os.path.exists(file_path):
                print "加载locator_offset配置文件失败. --------- 文件不存在： ", file_path
                return
            with open(file_path, 'r') as f:
                obj = json.load(f)  # 此时a是一个字典对
                if obj is None:
                    print "加载locator_offset配置文件失败. --------- "
                    return
                print"加载locator_offset配置文件 success. ---------", obj
                self.offset[0] = obj["laser_offset_x"]
                self.offset[1] = obj["laser_offset_y"]
        except Exception as e:
            print e

    def save_params(self):
        print "save locator_main params ing.................."
        self.laser_locator.save_params(self.node_path, self.env_name)
        self.box_laser_locator.save_params(self.node_path, self.env_name)

        try:
            config_dir = os.path.join(self.node_path, "config", self.env_name)
            if not os.path.exists(config_dir):
                os.makedirs(config_dir)
            obj = {
                "laser_offset_x": self.offset[0],
                "laser_offset_y": self.offset[1],
            }
            file_path = os.path.join(config_dir, '{}.json'.format("locator_offset"))
            # json_str = json.dumps(obj)
            with open(file_path, 'w') as f:
                json.dump(obj, f)
                print "保存locator_offset配置文件 success ---------", file_path, obj
        except Exception as e:
            print e

    def handle_action(self, key):
        print "action: ", key
        if key == ord('a'): # left
            self.offset[0] -= 1
        elif key == ord('d'): # right
            self.offset[0] += 1
        elif key == ord('w'): # up
            self.offset[1] -= 1
        elif key == ord('s'): # down
            self.offset[1] += 1

        print "offset[x: {},y: {}]".format(self.offset[0], self.offset[1])

    def point_to_3d(self, p):
        fx, fy = self.mtx[0, 0], self.mtx[1, 1]
        cx, cy = self.mtx[0, 2], self.mtx[1, 2]
        z = self.depth
        x = (p[0] - cx) * z / fx
        y = (p[1] - cy) * z / fy
        return [x, y, z]

    def run(self, frame):
        # 尝试定位盒子
        box_rst = self.box_laser_locator.detect(frame)

        img_show = frame.copy()
        h, w, c = img_show.shape
        # 中心画一条线，保证在盒子在中心位置附近
        cv2.line(img_show, (w / 2, 0), (w / 2, h), (50, 255, 50), 1, cv2.LINE_AA)
        cv2.line(img_show, (0, h / 2), (w, h / 2), (255, 50, 255), 1, cv2.LINE_AA)

        screen_center = (w / 2, h / 2)
        refer_rect_width = 100 * 0.2 * self.rect_scale_factor
        refer_rect_height = 100 * 0.4 * self.rect_scale_factor
        # 绘制黄色的矩形框, 要求和目标对齐
        cv2.rectangle(img_show,
                      (int(screen_center[0] - refer_rect_width / 2), int(screen_center[1] - refer_rect_height / 2)),
                      (int(screen_center[0] + refer_rect_width / 2), int(screen_center[1] + refer_rect_height / 2)),
                      (120, 60, 230), 1)

        center_offset = None
        angle_degree = None

        if box_rst is None:
            # print "-----------没找到盒子，矩形框定位------------"
            # 2. 如果没检测到盒子，先计算矩形框的包容盒，确定中心位置（根据二值化并查找边缘）
            # 根据矩形框和中心构建坐标系，绘制
            # 计算其偏移后的预测位置，绘制
            laser_rect_rst = self.laser_locator.detect(frame)
            if laser_rect_rst is None:
                print "--------未检测到矩形激光标定目标, 请及时调整激光视觉----------"
            else:
                if self.laser_rect_area is None:
                    print "--------矩形激光标目标定位成功--------"
                laser_rect_area, rect_center = laser_rect_rst
                # 更新最新的位置信息
                self.laser_rect_area = laser_rect_area
                self.rect_center = rect_center

            if self.laser_rect_area is not None:
                # 绘制中心和圆环
                cv2.circle(img_show, tuple(np.int0(self.rect_center)), 4, (0, 0, 255), -1)
                cv2.circle(img_show, tuple(np.int0(self.rect_center)), 10, (0, 255, 255), 1)
                points = np.int0(self.laser_rect_area)
                # 绘制最小有向包容盒
                cv2.drawContours(img_show, [points], 0, (0, 255, 255), 2)

        else:
            print "------激光打标机发现盒子，计算偏移量------：",
            # 1. 检测到盒子，并计算中点（注意用离中心的偏移量来修正x位置） 如果没有最新的标定数据，则要求其有激光实施标定
            target_rect_area, box_center_float = box_rst
            # 绘制最小有向包容盒
            cv2.drawContours(img_show, [np.int0(target_rect_area)], 0, (0, 0, 255), 2)

            box_center = np.int0(box_center_float)
            # rst_dict 保存了两个点： box_center 是计算出来的盒子中心， laser_circle 激光点中心
            cv2.circle(img_show, tuple(box_center), 10, (0, 200, 0), 2)  # 圆环
            cv2.circle(img_show, tuple(box_center), 3, (100, 100, 0), -1)  # 圆心

            # 计算并绘制预测中点（根据得到的坐标轴）
            if self.rect_center is None:
                print "目前暂未获得激光打标机默认的中心和打印方向信息"
            else:
                r_center = self.rect_center + self.offset

                # 传送带上的中心
                cv2.circle(img_show, tuple(np.int0(self.rect_center)), 4, (0, 0, 255), -1)
                cv2.circle(img_show, tuple(np.int0(self.rect_center)), 10, (0, 255, 255), 1)
                # 绘制中心和圆环, 盒子上的中心
                cv2.circle(img_show, tuple(np.int0(r_center)), 3, (0, 0, 255), -1)
                cv2.circle(img_show, tuple(np.int0(r_center)), 10, (0, 255, 255), 2)
                # 箭头
                cv2.arrowedLine(img_show, tuple(self.rect_center), tuple(np.int0(r_center)), (0, 0, 255), 2)

                # 绘制最小有向包容盒
                laser_rect_area_offset = self.laser_rect_area + self.offset
                r_points = np.int0(laser_rect_area_offset)
                cv2.drawContours(img_show, [r_points], 0, (0, 255, 255), 2)

                # 然后计算偏移后的预测位置和盒子中点的偏移量 ------------------------------------------   1
                cv2.arrowedLine(img_show, tuple(np.int0(r_center)), tuple(np.int0(box_center_float)), (255, 0, 0), 2)

                # 先把像素转成空间坐标，再计算x, y偏差
                # center_offset = box_center_float - r_center
                # 计算激光矩形的旋转角度，然后让偏移向量追加此旋转

                # 计算旋转角度            --------------------------------------------------------- 2
                # 激光预测矩形的四个点 laser_rect_area_offset
                # 盒子矩形的四个顶点      target_rect_area

                # 对激光预测矩形进行排序, 绘制向量
                laser_rect_area_offset = sort_rect(laser_rect_area_offset)
                laser_rect_vector_x, laser_rect_vector_y = calc_vector(laser_rect_area_offset)
                cv2.arrowedLine(img_show, tuple(np.int0(laser_rect_area_offset[0])),
                                tuple(np.int0(laser_rect_area_offset[0] + laser_rect_vector_x)),
                                (0, 0, 255), 2, cv2.LINE_AA)
                cv2.arrowedLine(img_show, tuple(np.int0(laser_rect_area_offset[0])),
                                tuple(np.int0(laser_rect_area_offset[0] + laser_rect_vector_y)),
                                (0, 255, 0), 2, cv2.LINE_AA)

                # 对盒子矩形的顶点进行排序显示
                target_rect_area = sort_rect(target_rect_area)
                target_rect_vector_x, target_rect_vector_y = calc_vector(target_rect_area)

                cv2.arrowedLine(img_show, tuple(np.int0(target_rect_area[0])),
                                tuple(np.int0(target_rect_area[0] + target_rect_vector_x)),
                                (0, 0, 255), 2, cv2.LINE_AA)
                cv2.arrowedLine(img_show, tuple(np.int0(target_rect_area[0])),
                                tuple(np.int0(target_rect_area[0] + target_rect_vector_y)),
                                (0, 255, 0), 2, cv2.LINE_AA)

                # 计算Y向量的夹角 laser_rect_vector_y -> target_rect_vector_y
                norm_l = np.linalg.norm(laser_rect_vector_y)
                norm_t = np.linalg.norm(target_rect_vector_y)
                cos_angle = laser_rect_vector_y.dot(target_rect_vector_y) / (norm_l * norm_t)

                cross_rst = np.cross(laser_rect_vector_y, target_rect_vector_y)
                angle_radius = np.arccos(cos_angle)
                angle_degree = np.rad2deg(angle_radius)

                if cross_rst < 0:
                    angle_degree = -angle_degree

                # 计算三维空间偏移量
                # 1. 将两个坐标系点、开始和结束点转成3维
                rst = self.calc_offset(r_center, laser_rect_vector_x, laser_rect_vector_y, box_center)
                center_offset = rst[:2] * 1000
                center_offset[0] = -center_offset[0]

                print "偏移：[x: {0[0]}, y: {0[1]}], 夹角：{1}".format(center_offset, angle_degree)
                # TODO: 最近的一波都记录下来，取一下加权平均数

                # TODO: 将像素单位转成物理单位mm

        cv2.imshow("image_final", img_show)

        if self.output_writer is not None:
            self.output_writer.write(img_show)

        # https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html?highlight=getrotationmatrix2d
        # M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
        # M = cv2.getAffineTransform(pts1,pts2)
        # cv2.warpAffine(img,M,(cols,rows))

        if center_offset is None or angle_degree is None:
            return None

        # 最后返回偏移量(x,y)和旋转角度Θ
        return center_offset, angle_degree

    def calc_offset(self, laser_rect_center, laser_rect_vector_x, laser_rect_vector_y, box_center):
        laser_rect = np.array([np.int0(laser_rect_center), np.int0(laser_rect_center + laser_rect_vector_x),
                               np.int0(laser_rect_center + laser_rect_vector_y)])  # center, x, y
        points = np.array([np.int0(laser_rect_center), box_center])
        a_array = np.zeros((0, 3))
        # 2. 构建坐标系
        for point in laser_rect:
            c_point = self.point_to_3d(point)
            a_array = np.row_stack((a_array, c_point))
        point_array = np.zeros((0, 3))
        for point in points:
            c_point = self.point_to_3d(point)
            point_array = np.row_stack((point_array, c_point))
        vector_x = a_array[1] - a_array[0]
        vector_x = vector_x / np.sqrt(np.sum(vector_x ** 2))
        vector_y = a_array[2] - a_array[0]
        vector_y = vector_y / np.sqrt(np.sum(vector_y ** 2))
        vector_z = np.array([0, 0, 1])
        vector_z = vector_z / np.sqrt(np.sum(vector_z ** 2))
        R = np.array([
            [vector_x[0], vector_y[0], vector_z[0]],
            [vector_x[1], vector_y[1], vector_z[1]],
            [vector_x[2], vector_y[2], vector_z[2]]
        ])
        translation = a_array[0]
        # transform_a2o = np.eye(4)
        # transform_a2o[:3, :3] = R
        # transform_a2o[:3,  3] = translation
        transform_o2a = np.eye(4)
        Rt = R.T
        transform_o2a[:3, :3] = Rt
        transform_o2a[:3, 3] = - Rt.dot(translation.T)
        # print point_array
        # print "R" + "-" * 30
        # print R
        # print "a_array " + "-" * 30
        # print a_array
        # print "transform_o2a" + "-" * 30
        # print transform_o2a
        # 3. 计算偏移的向量，映射到到激光坐标系B
        # vector = np.hstack(((point_array[1] - point_array[0]), 1))
        # print vector * 1000
        rst = transform_o2a.dot(np.hstack((point_array[1], 1))) - transform_o2a.dot(np.hstack((point_array[0], 1)))
        return rst

    def set_writer(self, output):
        self.output_writer = output
