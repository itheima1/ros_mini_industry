#! /usr/bin/env python
# coding=utf-8

import cv2
import numpy as np
from locator.locator_main import LocatorMain


def detect_from_camera():
    capture = cv2.VideoCapture(0)
    # capture = cv2.VideoCapture("/home/ty/Videos/Webcam/2020-07-07-045038.webm")
    if not capture.isOpened():
        print "相机无法打开"
        return

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    # capture.set(cv2.)
    # img = cv2.imread("./imgs/laser_0.jpg", cv2.IMREAD_COLOR)
    # img = cv2.imread("./imgs/laser_1.jpg", cv2.IMREAD_COLOR)
    locator_main = LocatorMain()
    # img = cv2.imread("./imgs/laser_2.jpg", cv2.IMREAD_COLOR)
    # cv2.imshow("image", img)
    while True:

        _, frame = capture.read()
        locator_main.run(frame)
        # cv2.imshow("image", frame)

        action = cv2.waitKey(30) & 0xFF
        if action == ord("q") or action == 27:
            break
    capture.release()
    cv2.destroyAllWindows()


def detect_from_video():
    # capture = cv2.VideoCapture("/home/ty/Videos/Webcam/2020-07-07-045038.webm")
    # capture = cv2.VideoCapture("/home/ty/Videos/Webcam/2020-07-07-045245.webm")
    # capture = cv2.VideoCapture("/home/ty/Videos/Webcam/2020-07-07-051725.webm")
    capture = cv2.VideoCapture("/home/ty/Videos/Webcam/rect.webm")
    if not capture.isOpened():
        print "相机无法打开"
        return

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    totalFrames = capture.get(cv2.CAP_PROP_FRAME_COUNT)
    currentFrames = 0

    locator = LocatorMain()

    try:
        while True:

            _, frame = capture.read()

            rst = locator.run(frame)
            # cv2.imshow("image", frame)

            action = cv2.waitKey(30) & 0xFF
            if action == ord("q") or action == 27:
                break

            currentFrames += 1

            # 循环
            if currentFrames == totalFrames - 1:
                currentFrames = 0
                capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

    except KeyboardInterrupt:
        print("Ctrl + c 主动停止程序")
    # except Exception as e:
    #     print('发生异常:', e)
    finally:
        capture.release()
        cv2.destroyAllWindows()


def detect_from_image():
    img = cv2.imread("./imgs/laser_1.jpg", cv2.IMREAD_COLOR)
    # img = cv2.imread("./imgs/laser_test.jpg", cv2.IMREAD_COLOR)
    locator = LocatorMain()
    locator.run(img)

    while True:
        action = cv2.waitKey(100) & 0xFF
        if action == ord("q") or action == 27:
            break

    cv2.destroyAllWindows()


def detect_from_image_loop():
    img_emtpy = cv2.imread("./imgs/laser_0.jpg", cv2.IMREAD_COLOR)
    img_rect = cv2.imread("./imgs/laser_rect1.jpg", cv2.IMREAD_COLOR)
    img_box = cv2.imread("./imgs/laser_1.jpg", cv2.IMREAD_COLOR)
    locator = LocatorMain()

    try:
        counter = 0
        while True:

            if counter == 0:
                locator.run(img_emtpy)
            elif counter % 3 == 0:
                locator.run(img_box)
            else:
                locator.run(img_rect)

            counter += 1
            action = cv2.waitKey(0) & 0xFF
            if action == ord("q") or action == 27:
                break

        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        print("Ctrl + c 主动停止程序")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # detect_from_camera()
    detect_from_video()
    # detect_from_image_loop()