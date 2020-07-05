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
from detector.assembly_line_detector import AssemblyLineDetector
from detector.agv_detector import AgvDetector
from detector.box_detector import find_box

class DetectorMain:

    def __init__(self):
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.on_mouse_event)
        self.line_detector = AssemblyLineDetector()
        self.agv_detector = AgvDetector()

    def start_find(self, img):
        cv2.imshow("image", img)
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
            img_masked, img_color_masked = line_rst
            # cv2.imshow("img_masked", img_masked)
            # 在流水线指定区域找盒子
            assembly_line_box_lst = find_box(img_masked, img_color_masked, "line")

        # print("----------------开始查找AVG盒子--------------")
        # 提取AGV小车区域
        agv_box_lst = []
        agv_rst = self.agv_detector.detect(img)
        if not agv_rst:
            print("未发现AGV小车，请检查并确认！ <<<<<<<<<<")
        else:
            agv_img_masked, agv_img_color_masked = agv_rst
            agv_box_lst = find_box(agv_img_masked, agv_img_color_masked, "agv")

        # 打印盒子列表 [(center, vector_x),(center, vector_x) ...]

        # 把盒子的索引根据当前的绝对位置推算个类型
        # 组装线 x < 790 , type = 1 成品
        # 小车线 x < 600 , type = 1 成品
        assembly_line_box_lst = [ (box[0],box[1], 1 if box[0][0] < 790 else 0) for box in assembly_line_box_lst ]
        agv_box_lst           = [ (box[0],box[1], 1 if box[0][0] < 600 else 0) for box in           agv_box_lst ]

        return assembly_line_box_lst, agv_box_lst

    def on_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("坐标----------------------({}, {})------------------".format(x, y))


"""
启动Action服务，等待调用者发起请求
开启盒子坐标发布者

从相机读取数据
发布盒子位姿信息
"""
if __name__ == '__main__':
    # 输入
    pic = cv2.imread("./imgs/camera_rgb_0.jpg", cv2.IMREAD_UNCHANGED)

    detector = DetectorMain()

    rst_lst = detector.start_find(pic)

    # 输出
    print(rst_lst)

    while True:
        action = cv2.waitKey(30) & 0xFF
        if action == ord("q") or action == 27:
            break

    cv2.destroyAllWindows()
