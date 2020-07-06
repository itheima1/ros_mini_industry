#!/usr/bin/env python
# encoding:utf-8
import cv2


class AbsDetector(object):

    def __init__(self):
        self.h_min = 0
        self.h_max = 180
        self.s_min = 0
        self.s_max = 25
        self.v_min = 210
        self.v_max = 255
        self.win_name = ""

    def on_button_click(self, state):
        print("button click !", state)


    def init_track_bar(self, name):
        self.win_name = "bin_img-"+ name
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("h_min:", self.win_name, self.h_min, 255, lambda x: self.update_hsv_args("h_min", x))
        cv2.createTrackbar("h_max:", self.win_name, self.h_max, 255, lambda x: self.update_hsv_args("h_max", x))
        cv2.createTrackbar("s_min:", self.win_name, self.s_min, 255, lambda x: self.update_hsv_args("s_min", x))
        cv2.createTrackbar("s_max:", self.win_name, self.s_max, 255, lambda x: self.update_hsv_args("s_max", x))
        cv2.createTrackbar("v_min:", self.win_name, self.v_min, 255, lambda x: self.update_hsv_args("v_min", x))
        cv2.createTrackbar("v_max:", self.win_name, self.v_max, 255, lambda x: self.update_hsv_args("v_max", x))

    def get_track_bar_name(self):
        return self.win_name

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
