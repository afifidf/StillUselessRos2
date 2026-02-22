#!/usr/bin/python

import rclpy
import time
import sys
import signal
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import JointState
from robotis_manager.motion_data_flow_handler import Data, RosHandler, Ticks
from robotis_manager.motion_module import MotionModule

# MACROS
HEAD_PAN  = 0
HEAD_TILT = 1
BODY_PAN  = 2
CTRL_NUM  = 2

HEAD_PAN_MIN  = -1.0
HEAD_PAN_MAX  =  1.0
HEAD_TILT_MIN = -1.2
HEAD_TILT_MAX =  1.0

CENTER_X_FILTERED = 0.0004444444444444695
CENTER_Y_FILTERED = -0.008902077

# exponential filter
KF_BALL_POS_X = 2
KF_BALL_POS_Y = 2


class HeadTracking(RosHandler):
    def __init__(self):
        super().__init__('head_tracking_handler')
        self.motion = MotionModule('head_tracking_motion')

    def range(self, value, start, end):
        return start <= value <= end

    def limit(self, value, min_val, max_val):
        return max_val if value > max_val else min_val if value < min_val else value

    def MotionHeadControl(self, pan, tilt):
        if self.range(pan, HEAD_PAN_MIN, HEAD_PAN_MAX):
            pan_msg = Float64(); pan_msg.data = pan
            self.pub_HeadPan.publish(pan_msg)
            time.sleep(0.1)
        if self.range(tilt, HEAD_TILT_MIN, HEAD_TILT_MAX):
            tilt_msg = Float64(); tilt_msg.data = tilt
            self.pub_HeadTilt.publish(tilt_msg)
            time.sleep(0.1)

    def MotionHeadScan(self, enable):
        msg = Bool(); msg.data = enable
        self.pub_HeadScan.publish(msg)
        time.sleep(0.1)

    def signalHandler(self, sig, frame):
        self.get_logger().info("You pressed ctrl+c! Exiting...")
        self.NormalizeKamera()
        sys.exit()

    def NormalizeKamera(self, Pan=-0.0, Tilt=0.0):
        self.MotionHeadControl(Pan, Tilt)
        self.motion.HeadJoint(Data.Head.headJointData)

    def Ticks(self):
        return int(time.time() * 1000)


def mapF(x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) // (inMax - inMin) + outMin


def main(args=None):
    rclpy.init(args=args)
    node = HeadTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
