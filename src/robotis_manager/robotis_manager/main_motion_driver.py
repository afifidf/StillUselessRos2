#!/usr/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

from robotis_manager.motion_data_flow_handler import Data, Ticks, TASKSTA_STANDBY
from robotis_manager.motion_module import MotionModule
from robotis_manager.motion_utility import MotionUtility


class MotionDriverNode(Node):
    def __init__(self):
        super().__init__('motion_driver')

        self.motion = MotionModule()
        self.utils  = MotionUtility()

        self.get_logger().info("Motion Driver Initialize")

        # Publish initial status
        msg = String()
        msg.data = Data.Motion.taskStatus
        self.motion.pub_TaskStatus.publish(msg)

        Data.Motion.taskStatus = TASKSTA_STANDBY
        msg2 = String()
        msg2.data = Data.Motion.taskStatus
        self.motion.pub_TaskStatus.publish(msg2)

        # Timer loop @ 10 Hz
        self.create_timer(0.1, self.loop)

    def loop(self):
        if Data.Motion.walkingPubUpdate:
            Data.Motion.walkingPubUpdate = False
            # WalkingParam belum tersedia sebagai custom msg
            # self.motion.pub_WalkingParams.publish(Data.Motion.WalkingParam_msg)
            self.get_logger().info("Walking Parameters Set Mode 1")
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = MotionDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
