#!/usr/bin/python

import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
from std_msgs.msg import Int16MultiArray, Int32MultiArray
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
import cv2
import time

# MACROS
KF_FOCAL_LENS       = 347.10
KF_REAL_BALL_WIDTH  = 14.6
MIN_RADIUS_OF_BALL  = 5


class RosHandler(Node):
    def __init__(self):
        super().__init__('vision_ros_handler')

        self.bridge = CvBridge()

        # Publishers
        self.ballFilter     = self.create_publisher(Float32MultiArray, 'ball/coordinate_filter', 10)
        self.ballCoordinate = self.create_publisher(Int16MultiArray,   'ball/coordinate',         10)
        self.ballState      = self.create_publisher(Int8MultiArray,    'ball/ball_state',         10)
        self.frameSize      = self.create_publisher(Int32MultiArray,   'frame/frame_size',        10)

        # ball last position
        self.ball_last_pos_x = -1
        self.ball_last_pos_y = -1

        self.get_logger().info("VisionRosHandler initialized")

    def imgMsg2Frame(self, data):
        frame = []
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
        return frame

    def compressedImgMsg2Frame(self, data):
        frame = []
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
        return frame

    def ball2ros(self, data, shape):
        try:
            x_filter, y_filter, distance = 0.0, 0.0, 0.0
            flag, pos_y, pos_x, pos_kick = 0, -1, -1, -1

            if data != {}:
                x_filter = (data["x_pos"] / shape[1]) * 2 - 1
                y_filter = (data["y_pos"] / shape[0]) * 2 - 1
                distance = (KF_REAL_BALL_WIDTH * KF_FOCAL_LENS) / data["size"]
                flag = 1
            else:
                flag = 0

            if flag:
                pos_y    = 2 if data["y_pos"] > shape[0] - 85 else 1 if data["y_pos"] < 85 else 0
                pos_x    = 2 if data["x_pos"] < 100 else 1 if data["x_pos"] > shape[1] - 100 else 0
                pos_kick = 0 if data["x_pos"] < (shape[1] // 2) else 1
                self.ball_last_pos_y = pos_y
                self.ball_last_pos_x = pos_x

                coordinate = Float32MultiArray()
                position   = Int16MultiArray()
                position.layout.dim.append(MultiArrayDimension())
                position.layout.dim[0].size   = 3
                position.layout.dim[0].stride = 1
                coordinate.data = [x_filter, y_filter, distance]
                position.data   = [int(data["x_pos"]), int(data["y_pos"]), int(data["size"] // 2)]

                self.ballFilter.publish(coordinate)
                self.ballCoordinate.publish(position)
            else:
                pos_y = -1
                pos_x = -1

            state      = Int8MultiArray()
            state.data = [pos_x, pos_y, pos_kick,
                          self.ball_last_pos_x,
                          self.ball_last_pos_y,
                          flag]

            frame_size      = Int32MultiArray()
            frame_size.data = [shape[0], shape[1]]

            self.frameSize.publish(frame_size)
            self.ballState.publish(state)

        except KeyError as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RosHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
