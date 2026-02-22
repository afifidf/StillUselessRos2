#!/usr/bin/python

import time
import sys
import math
import numpy as np
from op3_walking_module_msgs.msg import WalkingParam
from std_msgs.msg import String


class MotionUtility:
    # static var
    init_x_offset        = -0.02
    init_y_offset        = 0.014
    init_z_offset        = 0.030
    init_roll_offset     = -0.0015
    init_pitch_offset    = 0
    init_yaw_offset      = 0.0
    hip_pitch_offset     = 0.22

    dsp_ratio            = 0.2
    step_fb_ratio        = 0.3
    z_move_amplitude     = 0.09
    move_aim_on          = False
    balance_enable       = True
    balance_hip_roll_gain    = 0.50
    balance_knee_gain        = 0.80
    balance_ankle_roll_gain  = 1.2
    balance_ankle_pitch_gain = 1.0
    y_swap_amplitude     = 0.015
    z_swap_amplitude     = 0.010
    arm_swing_gain       = 0.80
    pelvis_offset        = 0.009
    p_gain               = 10
    i_gain               = 1
    d_gain               = 2

    def __init__(self):
        pass

    @staticmethod
    def EnableCtrlModule(publisher, module):
        msg = String(); msg.data = module
        publisher.publish(msg)
        time.sleep(0.1)

    @staticmethod
    def InitPose(publisher):
        msg = String(); msg.data = 'ini_pose'
        publisher.publish(msg)
        time.sleep(4)

    @staticmethod
    def WalkingCommand(publisher, data):
        msg = String(); msg.data = data
        publisher.publish(msg)
        time.sleep(0.1)

    @staticmethod
    def Walking(move_x, move_y, move_yaw, period):
        msg = WalkingParam()
        msg.init_x_offset        = MotionUtility.init_x_offset
        msg.init_y_offset        = MotionUtility.init_y_offset
        msg.init_z_offset        = MotionUtility.init_z_offset
        msg.init_roll_offset     = MotionUtility.init_roll_offset
        msg.init_pitch_offset    = MotionUtility.init_pitch_offset
        msg.init_yaw_offset      = MotionUtility.init_yaw_offset
        msg.hip_pitch_offset     = MotionUtility.hip_pitch_offset
        msg.period_time          = period
        msg.dsp_ratio            = MotionUtility.dsp_ratio
        msg.step_fb_ratio        = MotionUtility.step_fb_ratio
        msg.x_move_amplitude     = move_x
        msg.y_move_amplitude     = move_y
        msg.z_move_amplitude     = MotionUtility.z_move_amplitude
        msg.angle_move_amplitude = (move_yaw + -0.5) * math.pi / 180
        msg.move_aim_on          = MotionUtility.move_aim_on
        msg.balance_enable       = MotionUtility.balance_enable
        msg.balance_hip_roll_gain    = MotionUtility.balance_hip_roll_gain
        msg.balance_knee_gain        = MotionUtility.balance_knee_gain
        msg.balance_ankle_roll_gain  = MotionUtility.balance_ankle_roll_gain
        msg.balance_ankle_pitch_gain = MotionUtility.balance_ankle_pitch_gain
        msg.y_swap_amplitude     = MotionUtility.y_swap_amplitude
        msg.z_swap_amplitude     = MotionUtility.z_swap_amplitude
        msg.arm_swing_gain       = MotionUtility.arm_swing_gain
        msg.pelvis_offset        = MotionUtility.pelvis_offset
        msg.p_gain               = MotionUtility.p_gain
        msg.i_gain               = MotionUtility.i_gain
        msg.d_gain               = MotionUtility.d_gain
        return msg, True

    @staticmethod
    def Quat2Euler(imu):
        w = imu.orientation.w
        x = imu.orientation.x
        y = imu.orientation.y
        z = imu.orientation.z
        ysqr = y * y
        t0    = +2.0 * (w * x + y * z)
        t1    = +1.0 - 2.0 * (x * x + ysqr)
        pitch = np.degrees(np.arctan2(t0, t1))
        t2    = np.clip(+2.0 * (w * y - z * x), -1.0, 1.0)
        roll  = np.degrees(np.arcsin(t2))
        t3    = +2.0 * (w * z + x * y)
        t4    = +1.0 - 2.0 * (ysqr + z * z)
        yaw   = np.degrees(np.arctan2(t3, t4))
        return yaw, pitch, roll
