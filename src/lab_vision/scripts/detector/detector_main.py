#!/usr/bin/env python
# encoding:utf-8
"""
图片感兴趣区域提取
根据颜色提取目标区域提取正方形区域
确定四个区域
     下料区 1 （传送带）
     产品区 2 （小车）
     原料区 3 （小车）
     上料区 4 （传送带）

定位下料区盒子位置
发布位置&姿态（二维）
"""
import cv2
import numpy as np
from detector.assembly_line_detector import AssemblyLineDetector
from detector.agv_detector import AgvDetector
from detector.box_detector import find_box
from common.global_ctl import is_debug_mode
from common.curve_tools import get_rect_range
import common.global_ctl as g_ctl


class DetectorMain:
    font_face = cv2.FONT_HERSHEY_COMPLEX

    def __init__(self, node_path, env_name = "env1"):
        if is_debug_mode:
            cv2.namedWindow("image")
            cv2.setMouseCallback("image", self.on_mouse_event)
        self.line_detector = AssemblyLineDetector()
        self.agv_detector = AgvDetector()
        # self.spliter_line_in_x = 790
        # self.spliter_agv_in_x = 600
        self.spliter_in_x_line = 950
        self.spliter_percent_in_x_line = 0.4
        self.spliter_percent_in_x_agv = 0.6
        self.node_path = node_path
        self.env_name = env_name
        self.init_params()

    def init_params(self):
        self.line_detector.load_params(self.node_path, self.env_name)
        self.agv_detector.load_params(self.node_path, self.env_name)

    def save_params(self):
        print "save detector_main params ing.................."
        self.line_detector.save_params(self.node_path, self.env_name)
        self.agv_detector.save_params(self.node_path, self.env_name)

    def start_find(self, img):
        # img = pic.copy()
        # image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        h, w, c = img.shape
        # print("width: {}, height: {}, channel: {}".format(w, h, c))
        # 提取流水线区域

        # print("----------------开始查找传送带盒子--------------")
        assembly_line_box_lst = []
        line_rst = self.line_detector.detect(img)
        if not line_rst:
            print("未发现流水线，请检查并确认！ <<<<<<<<<<")
        else:
            img_masked, img_color_masked, target_area = line_rst
            # cv2.imshow("img_masked", img_masked)
            # 在流水线指定区域找盒子， img_color_masked是数据源
            assembly_line_box_lst = find_box(img_masked, img_color_masked, "line")

            target_area_range = get_rect_range(target_area)

            # 绘制分割线
            spliter_in_x_line = self.draw_split_line(img_color_masked, target_area_range, self.spliter_in_x_line, False)

            # 把盒子根据当前的绝对位置推算个类型
            # 组装线 x < 790 , type=0原材料，type=1成品，type=2上料空位
            assembly_line_box_lst = [ (box[0], box[1], 1 if box[0][0] < spliter_in_x_line else 0)
                for box in assembly_line_box_lst]

            # 把传送带原材料区的空白位置构建出点和x向量，且type=2
            # 取出竖直方向的中点，在水平方向0.6的位置，设置为上料区
            y_start, y_end, x_start, x_end = target_area_range
            radius = 60

            x_end[0] = min(spliter_in_x_line + 600, 1920)
            for i in range(3):
                center = tuple(np.int0(x_start + (x_end - x_start) * (1.0 - i * 0.1)))

                # 如果上料区的半径区域没点，就确认当前区域可以放盒子，并且不再往左找了 （看看上料区目前有没有盒子）
                filter_rst_lst = [box for box in assembly_line_box_lst
                                  if box[2] == 0 and (center[0] - radius) < box[0][0] < (center[0] + radius)]

                if len(filter_rst_lst) == 0:
                    vect_y = (-1, 0)
                    # vect_x = x_end - x_start
                    # vect_x = vect_x / np.linalg.norm(vect_x)
                    assembly_line_box_lst.append([center, vect_y, 2])
                    # 此区域目前没盒子，可以放
                    cv2.circle(img_color_masked, center, radius, (230, 80, 160), 2)
                    cv2.circle(img_color_masked, center, 5, (0,0,255), -1)
                    cv2.arrowedLine(img_color_masked, center, tuple(center + np.array(vect_y) * 60),
                                    (0, 255, 0), 2, cv2.LINE_AA)
                    break

            else:
                print ">>>>>>>>>>> 当前传送带上没有合适的位置放置盒子 <<<<<<<<<<<<<<<<<"


            img_color_masked_half = cv2.resize(img_color_masked, None, fx=0.5, fy=0.5)
            cv2.imshow("img_color_masked-line", img_color_masked_half)
            cv2.moveWindow("img_color_masked-line", 0, 540)

        # print("----------------开始查找AVG盒子--------------")
        # 提取AGV小车区域
        agv_box_lst = []
        agv_rst = self.agv_detector.detect(img)
        if not agv_rst:
            print("未发现AGV小车，请检查并确认！ <<<<<<<<<<")
        else:
            agv_img_masked, agv_img_color_masked, agv_target_area = agv_rst
            # 在AGV小车上查找盒子： agv_img_color_masked是数据源, [center(x, y), vector_y(x, y)]
            agv_box_lst = find_box(agv_img_masked, agv_img_color_masked, "agv")

            agv_target_area_range = get_rect_range(agv_target_area)

            spliter_in_x_agv = self.draw_split_line(agv_img_color_masked, agv_target_area_range,
                                                    self.spliter_percent_in_x_agv)

            # 把盒子根据当前的绝对位置推算个类型
            # 小车线 x < 600 , type=0原材料，type=1产品，type=2产品区空位
            agv_box_lst = [(box[0], box[1], 1 if box[0][0] < spliter_in_x_agv else 0)
                           for box in agv_box_lst]


            y_start, y_end, x_start, x_end = agv_target_area_range
            # 绘制2个水平分割线
            # y_spliters = [0, 0.333, 0.666, 1.0]
            # 绘制1个水平分割线
            y_spliters = [0, 0.5, 1.0]

            # 把AVG产品区的空白位置构建出点和y向量，且type=2
            # 遍历每个产品矩形区域，看看有没有落在其中的产品中心点，没有则构建 空白位置数据-------这里是一行一行遍历的
            for s_index in range(len(y_spliters) - 1):
                spliter = y_spliters[s_index]

                # 右上角
                y_point = np.int0(y_start + (y_end - y_start) * spliter)
                # 右下角
                y_point_next = np.int0(y_start + (y_end - y_start) * y_spliters[s_index + 1])

                # ----------------------------------------------------------原料区
                top_right = np.array([x_end[0], y_start[1]])
                bottom_right = np.array([x_end[0], y_end[1]])

                raw_area = np.array([
                    y_point,
                    y_point_next,
                    np.int0(top_right + (bottom_right - top_right) * y_spliters[s_index + 1]),
                    np.int0(top_right + (bottom_right - top_right) * spliter),
                ])

                # 在原料区判断, 是否有空位
                filter_raw_lst = [box for box in agv_box_lst
                                  if box[2] == 0 and y_point[1] < box[0][1] < y_point_next[1]]

                if len(filter_raw_lst) == 0:
                    # 如果没有点
                    center = (raw_area[0] + raw_area[2]) * 0.5
                    center[0] += ((raw_area[3] - raw_area[0]) * 0.1)[0]
                    center = np.int0(center)
                    vect_y = (0, 1)
                    agv_box_lst.append([tuple(center), tuple(vect_y), 2])
                    cv2.circle(agv_img_color_masked, tuple(center), 60, (230, 80, 160), 2)
                    cv2.circle(agv_img_color_masked, tuple(center), 5, (0,0,255), -1)
                    cv2.arrowedLine(agv_img_color_masked, tuple(center), tuple(center + (np.array(vect_y) * 60)),
                                    (0, 255, 0), 2, cv2.LINE_AA)

                # ----------------------------------------------------------产品区

                # 矩形区域
                pro_area = np.array([
                    [x_start[0], y_point[1]],       # 左上
                    [x_start[0], y_point_next[1]],  # 左下
                    y_point_next,                   # 右下
                    y_point                         # 右上
                ])
                # 在Pro产品区判断, 是否有产品空位
                filter_pro_lst = [box for box in agv_box_lst
                                  if box[2] == 1 and y_point[1] < box[0][1] < y_point_next[1]]

                if len(filter_pro_lst) == 0:
                    # 如果没有点
                    center = (pro_area[0] + pro_area[2]) * 0.5
                    center[0] += ((pro_area[3] - pro_area[0]) * 0.26)[0]
                    center = np.int0(center)
                    # vect_y = y_end - y_start
                    # vect_y = vect_y / np.linalg.norm(vect_y)
                    # 目前只能传整形数据，就不自己算向量了。
                    vect_y = (0, 1)
                    agv_box_lst.append([tuple(center), tuple(vect_y), 2])

                    cv2.circle(agv_img_color_masked, tuple(center), 60, (230, 80, 160), 2)
                    cv2.circle(agv_img_color_masked, tuple(center), 5, (0,0,255), -1)
                    cv2.arrowedLine(agv_img_color_masked, tuple(center), tuple(center + (np.array(vect_y) * 60)),
                                    (0, 255, 0), 2, cv2.LINE_AA)
                    # cv2.fillPoly(agv_img_color_masked, [pro_area], (200,25,100), cv2.LINE_AA)




            # 绘制分割线
            for spliter in y_spliters:
                y_point = np.int0(y_start + (y_end - y_start) * spliter)
                p_start = (x_start[0], y_point[1])
                p_end = (x_end[0], y_point[1])
                cv2.line(agv_img_color_masked, p_start, p_end, (30, 150, 255), 2, cv2.LINE_AA)

            agv_img_color_masked_half = cv2.resize(agv_img_color_masked, None, fx=0.5, fy=0.5)
            cv2.imshow("img_color_masked-agv", agv_img_color_masked_half)
            cv2.moveWindow("img_color_masked-agv", 960, 540)


        if g_ctl.is_debug_mode:
            # 中心画一条线，保证在盒子在中心位置附近
            cv2.line(img, (w / 2, 0), (w / 2, h), (50, 255, 50), 2, cv2.LINE_AA)
            line_offset_x = -30
            cv2.line(img, (w / 2 + line_offset_x, 0), (w / 2 + line_offset_x, h), (50, 255, 50), 2, cv2.LINE_AA)
            line_offset_x = -530
            cv2.line(img, (w / 2 + line_offset_x, 0), (w / 2 + line_offset_x, h), (50, 255, 50), 2, cv2.LINE_AA)
            line_offset_x = 444
            cv2.line(img, (w / 2 + line_offset_x, 0), (w / 2 + line_offset_x, h), (50, 255, 50), 2, cv2.LINE_AA)

            cv2.line(img, (0, h / 2), (w, h / 2), (50, 50, 255), 2, cv2.LINE_AA)
            line_offset_y = 48
            cv2.line(img, (0, h / 2 + line_offset_y), (w, h / 2 + line_offset_y), (50, 50, 255), 2, cv2.LINE_AA)
            line_offset_y = 316
            cv2.line(img, (0, h / 2 + line_offset_y), (w, h / 2 + line_offset_y), (50, 50, 255), 2, cv2.LINE_AA)
            line_offset_y = 360
            cv2.line(img, (0, h / 2 + line_offset_y), (w, h / 2 + line_offset_y), (50, 50, 255), 2, cv2.LINE_AA)

            cv2.putText(img, "Center", (w / 2 + 50, h / 2 - 50), DetectorMain.font_face, fontScale=1.3, color=(255, 100, 255), thickness=2, lineType=cv2.LINE_AA)
            cv2.arrowedLine(img, (w / 2 + 50, h / 2 - 50), (w / 2 + 5, h / 2 - 5), (0,255,255), 2)

            cv2.imshow("image", img)
        # 打印盒子列表 [(center, vector_x),(center, vector_x) ...]
        return assembly_line_box_lst, agv_box_lst

    def draw_split_line(self, img_color_masked, target_area_range, spliter_value_in_x, is_percent = True):
        y_start, y_end, x_start, x_end = target_area_range

        if is_percent:
            x = int((x_end[0] - x_start[0]) * spliter_value_in_x + x_start[0])
        else:
            x = spliter_value_in_x
        # 绘制竖分线
        cv2.line(img_color_masked,
                 (x, y_start[1]),
                 (x, y_end[1]), (200, 60, 255), 2, cv2.LINE_AA)
        line_start = (x, y_start[1])
        cv2.putText(img_color_masked, "Raw", (line_start[0] + 50, line_start[1] - 20),
                    DetectorMain.font_face, fontScale=1, color=(255, 255, 255), thickness=2, lineType=cv2.LINE_AA)
        cv2.putText(img_color_masked, "Pro", (line_start[0] - 120, line_start[1] - 20),
                    DetectorMain.font_face, fontScale=1, color=(255, 255, 255), thickness=2, lineType=cv2.LINE_AA)
        return x

    def on_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("坐标----------------------({}, {})------------------".format(x, y))

