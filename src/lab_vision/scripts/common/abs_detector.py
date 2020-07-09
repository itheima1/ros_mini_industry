#!/usr/bin/env python
# encoding:utf-8
import cv2
import json
from os import path

class AbsDetector(object):

    def __init__(self):
        self.h_min = 0
        self.h_max = 180
        self.s_min = 0
        self.s_max = 25
        self.v_min = 210
        self.v_max = 255
        self.win_name = ""

    def save_params(self, node_path):
        if self.win_name:
           try:
               obj = {
                   "h_min": self.h_min,
                   "h_max": self.h_max,
                   "s_min": self.s_min,
                   "s_max": self.s_max,
                   "v_min": self.v_min,
                   "v_max": self.v_max
               }
               file_path = path.join(node_path, "config", '{}.json'.format(self.win_name))
               # json_str = json.dumps(obj)
               with open(file_path, 'w') as f:
                   json.dump(obj, f)
                   print "保存配置文件 success ---------", file_path, obj
           except Exception as e:
               print e
        else:
            print "保存配置失败 --------- 没有窗口名称"

    def load_params(self, node_path):
        if self.win_name:
            try:
                file_path = path.join(node_path, "config", '{}.json'.format(self.win_name))
                if not path.exists(file_path):
                    print "加载配置文件失败. --------- 文件不存在： ", file_path
                    return
                with open(file_path, 'r') as f:
                    obj = json.load(f)  # 此时a是一个字典对
                    if obj is None:
                        print "加载配置文件失败. --------- "
                        return
                    print"加载配置文件 success. ---------", obj
                    self.h_min = obj["h_min"]
                    self.h_max = obj["h_max"]
                    self.s_min = obj["s_min"]
                    self.s_max = obj["s_max"]
                    self.v_min = obj["v_min"]
                    self.v_max = obj["v_max"]
            except Exception as e:
                print e
        else:
            print "保存配置失败 --------- 没有窗口名称"

    def init_win_name(self, name):
        self.win_name = "mask-" + name

    def init_track_bar(self, name):
        self.win_name = "mask-" + name
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("h_min:", self.win_name, self.h_min, 255, lambda x: self.update_hsv_args("h_min", x))
        cv2.createTrackbar("h_max:", self.win_name, self.h_max, 255, lambda x: self.update_hsv_args("h_max", x))
        cv2.createTrackbar("s_min:", self.win_name, self.s_min, 255, lambda x: self.update_hsv_args("s_min", x))
        cv2.createTrackbar("s_max:", self.win_name, self.s_max, 255, lambda x: self.update_hsv_args("s_max", x))
        cv2.createTrackbar("v_min:", self.win_name, self.v_min, 255, lambda x: self.update_hsv_args("v_min", x))
        cv2.createTrackbar("v_max:", self.win_name, self.v_max, 255, lambda x: self.update_hsv_args("v_max", x))

    def get_win_name(self):
        return self.win_name

    def get_trackbar_pos(self, track_bar_name):
        return cv2.getTrackbarPos(track_bar_name, self.get_win_name())

    def detect(self, image):
        raise NotImplementedError

    def update_hsv_args(self, arg_name, value):
        if arg_name == "h_min":
            self.h_min = value
        elif arg_name == "h_max":
            self.h_max = value
        elif arg_name == "s_min":
            self.s_min = value
        elif arg_name == "s_max":
            self.s_max = value
        elif arg_name == "v_min":
            self.v_min = value
        elif arg_name == "v_max":
            self.v_max = value


if __name__ == '__main__':
    detector = AbsDetector()

    print(detector.h_min, detector.h_max)
    detector.update_hsv_args("h_min", 100)
    detector.update_hsv_args("h_max", 200)
    print(detector.h_min, detector.h_max)
