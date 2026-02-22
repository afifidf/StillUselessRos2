#!/usr/bin/python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from robotis_manager.vision_main import BallDetectorNode
from robotis_manager.vision_ros_handler import RosHandler


class ImageNode(Node):
    def __init__(self):
        super().__init__('robotis_vision')
        self.get_logger().info("Vision Initialize")
        self.rh = RosHandler()

        self.create_subscription(
            Image,
            'usb_cam_node/image_raw',
            lambda data: BallDetectorNode(data, self.rh),
            10
        )
        # self.create_subscription(
        #     CompressedImage,
        #     'usb_cam/image_raw/compressed',
        #     Node,
        #     10
        # )


def main(args=None):
    rclpy.init(args=args)
    node = ImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.rh.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
