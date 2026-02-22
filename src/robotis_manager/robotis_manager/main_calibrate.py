#!/usr/bin/python

import numpy as np
import cv2
import os
import rclpy
from rclpy.node import Node

from robotis_manager.vision_image import Vision
from robotis_manager.vision_utils import *


class CalibrateImage:
    def __init__(self, filename=None):
        self.colorInstance = ColorBased()
        self.filenames = filename
        self.val_load = self.colorInstance.load(self.filenames)
        self.val_load_lower = self.val_load[0]
        self.val_load_upper = self.val_load[1]

    def nothing(self, x):
        pass

    def write(self, path, lower, upper):
        val = {path + '_lower_val': lower, path + '_upper_val': upper}
        with open(path + '.txt', 'w') as f:
            f.write(str(val))

    def createTrackbar(self):
        cv2.namedWindow(self.filenames, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.filenames, 600, 400)
        cv2.createTrackbar('Low Hue', self.filenames, self.val_load_lower[0], 255, self.nothing)
        cv2.createTrackbar('Low Sat', self.filenames, self.val_load_lower[1], 255, self.nothing)
        cv2.createTrackbar('Low Val', self.filenames, self.val_load_lower[2], 255, self.nothing)
        cv2.createTrackbar('Max Hue', self.filenames, self.val_load_upper[0], 255, self.nothing)
        cv2.createTrackbar('Max Sat', self.filenames, self.val_load_upper[1], 255, self.nothing)
        cv2.createTrackbar('Max Val', self.filenames, self.val_load_upper[2], 255, self.nothing)

    def readTrackbar(self):
        lh = cv2.getTrackbarPos('Low Hue', self.filenames)
        ls = cv2.getTrackbarPos('Low Sat', self.filenames)
        lv = cv2.getTrackbarPos('Low Val', self.filenames)
        uh = cv2.getTrackbarPos('Max Hue', self.filenames)
        us = cv2.getTrackbarPos('Max Sat', self.filenames)
        uv = cv2.getTrackbarPos('Max Val', self.filenames)
        self.write(self.filenames, [lh, ls, lv], [uh, us, uv])


def main(args=None):
    rclpy.init(args=args)
    node = Node('vision_calibrate_handler')

    # Set working directory ke folder data agar file .txt bisa dibaca/ditulis
    from ament_index_python.packages import get_package_share_directory
    data_dir = os.path.join(get_package_share_directory('robotis_manager'), 'data')
    os.chdir(data_dir)

    calGreenField = CalibrateImage('robotis_green_field')
    calOrangeBall = CalibrateImage('robotis_orange_ball')

    calGreenField.createTrackbar()
    calOrangeBall.createTrackbar()

    try:
        while rclpy.ok():
            calGreenField.readTrackbar()
            calOrangeBall.readTrackbar()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
