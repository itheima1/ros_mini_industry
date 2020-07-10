#!/usr/bin/env python
# encoding:utf-8

import cv2
from detector.detector_main import DetectorMain
import common.global_ctl as g_ctl
"""
启动Action服务，等待调用者发起请求
开启盒子坐标发布者

从相机读取数据
发布盒子位姿信息
"""
if __name__ == '__main__':
    g_ctl.update_debug_mode(True)
    print "debug_mode: ", g_ctl.is_debug_mode

    detector = DetectorMain("../", "env")

    # 输入
    pic = cv2.imread("./imgs/test_aubo2.png", cv2.IMREAD_UNCHANGED)

    rst_lst = detector.start_find(pic)

    # 输出
    print(rst_lst)

    while True:
        action = cv2.waitKey(30) & 0xFF
        if action == ord("q") or action == 27:
            break
        elif action == ord('s') or action == ord('S'):
            detector.save_params()

    cv2.destroyAllWindows()