#!/usr/bin/python
#encoding: utf-8

import cv2
import numpy as np
from tools.curve_tools import shrink_polygon

if __name__ == '__main__':
    img = np.zeros((480, 640, 3), np.uint8)

    target_area = np.array([
        [30, 100],
        [230, 100],
        [210, 180],
        [10, 180],
    ])

    cv2.drawContours(img, [target_area], 0, (0, 0, 255), 2)

    new_area = shrink_polygon(target_area, 0.8)
    new_area = np.int0(new_area)
    print new_area
    cv2.drawContours(img, [new_area], 0, (0, 255, 255), 2)

    new_area = shrink_polygon(new_area, 1.5)
    new_area = np.int0(new_area)
    cv2.drawContours(img, [new_area], 0, (255, 255, 255), 2)

    cv2.imshow("image", img)

    cv2.waitKey(0)

    cv2.destroyAllWindows()
