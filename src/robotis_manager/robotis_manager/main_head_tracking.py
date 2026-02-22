#!/usr/bin/python

import rclpy
from rclpy.node import Node
import time

from robotis_manager.motion_data_flow_handler import Data, Ticks
from robotis_manager.motion_PID import PID, PIDControl
from robotis_manager.motion_head_control_module import HeadTracking, HEAD_TILT, HEAD_PAN, CTRL_NUM, KF_BALL_POS_Y, KF_BALL_POS_X
from robotis_manager.motion_module import MotionModule

# constants
TILT_THRESHOLD = 0.025
PAN_THRESHOLD  = 0.025
INIT_HEAD_TILT = -0.3


class HeadTrackingNode(Node):
    def __init__(self):
        super().__init__('head_tracking')

        self.headTrack = HeadTracking()
        self.motion    = MotionModule()

        self.resetTime = Ticks()
        self.outTilt   = 0
        self.outPan    = 0

        self.pid = [PIDControl() for _ in range(CTRL_NUM)]

        self.pid[HEAD_TILT].setConstant(Kp=1.2, Ki=0.8, Kd=1.2)
        self.pid[HEAD_TILT].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_TILT].setRange(InMin=0, InMax=337, OutMin=-1.2, OutMax=-0.2)
        self.pid[HEAD_TILT].setSetPoints(168.5)

        self.pid[HEAD_PAN].setConstant(Kp=0.75, Ki=0.0, Kd=0.0)
        self.pid[HEAD_PAN].setTime(Ti=10.0, Td=10.0)
        self.pid[HEAD_PAN].setRange(InMin=0, InMax=450, OutMin=-1.2, OutMax=1.2)
        self.pid[HEAD_PAN].setSetPoints(225.0)

        for i in range(CTRL_NUM):
            self.pid[i].Init()
            self.pid[i].setError(0)
            self.pid[i].setEnableWindUpCrossing()
            self.pid[i].setEnableWindUpLimit()

        self.headTrack.NormalizeKamera()
        self.get_logger().info("Head Tracking Initialize")

        # Timer loop @ 50 Hz
        self.create_timer(0.02, self.loop)

    def loop(self):
        self.motion.EnableCtrlModule("head_control_module")

        Data.Head.ballFilterY = (Data.Head.ballFilterY * KF_BALL_POS_Y) + Data.Head.ballCY
        Data.Head.ballFilterY /= KF_BALL_POS_Y + 1
        Data.Head.ballFilterX = (Data.Head.ballFilterX * KF_BALL_POS_X) + Data.Head.ballCX
        Data.Head.ballFilterX /= KF_BALL_POS_X + 1

        if Data.Head.headStatus:
            if Data.Head.ballFlag:
                self.pid[HEAD_TILT].calculate(Data.Head.ballFilterY)
                self.outTilt = self.pid[HEAD_TILT].getOutput()

                self.pid[HEAD_PAN].calculate(Data.Head.ballFilterX)
                self.outPan = self.pid[HEAD_PAN].getOutput()

                self.headTrack.MotionHeadControl(self.outPan, self.outTilt)
            else:
                pass  # ball not found, bisa tambahkan scanning logic


def main(args=None):
    rclpy.init(args=args)
    node = HeadTrackingNode()
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
