#!/usr/bin/python

import time
import math
import numpy as np

from robotis_manager.motion_PID import PID, PIDControl
from robotis_manager.motion_module import MotionModule, ACTION_GETUPFRONT, ACTION_GETUPBACK
from robotis_manager.motion_data_flow_handler import Data, Ticks
from robotis_manager.motion_utility import MotionUtility as util
from robotis_manager.motion_head_control_module import HeadTracking

# constant macros
BODY_REF        = 0
BODY_PAN        = 1
STEP_CORRECTION = 2
TASK_CTRL_NUM   = 3

KF_BALL_SIZE = 2
KF_BALL_POS  = 1
KF_YAW_ERROR = 10

BALL_SETPOINT  = 0
STEP_SPEED_X   = 0.042
INIT_HEAD_TILT = -0.3


def mapF(x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin


class BaseControl:
    def __init__(self):
        self.module    = MotionModule()
        self.headTrack = HeadTracking()

        # pid control outputs
        self.outBodyReference  = 0
        self.outBodyPanning    = 0
        self.outStepCorrection = 0

        self.pid = [PIDControl() for _ in range(TASK_CTRL_NUM)]
        self.pidSetConstant()

        # head scanning state
        self.headTiltScan  = -0.3
        self.headPanScan   =  1.0
        self.headScanState = -1
        self.headScanCount =  0
        self.findWithWalk  = True

        # variables
        self.yawOffset        = 0
        self.yawErrorFiltered = 0
        self.ballPosFiltered  = 0
        self.ballSizeFiltered = 200
        self.xError           = 0
        self.xSpeed           = 0

        self.taskTimer  = 0
        self.fallStatus = False

    def pidSetConstant(self):
        self.pid[BODY_REF].setConstant(Kp=5.0, Ki=0.05, Kd=0.0)
        self.pid[BODY_REF].setTime(Ti=10.0, Td=10.0)
        self.pid[BODY_REF].setRange(InMin=-500, InMax=500, OutMin=-500, OutMax=500)
        self.pid[BODY_REF].setSetPoints(0)

        self.pid[BODY_PAN].setConstant(Kp=4.5, Ki=0.05, Kd=0.0)
        self.pid[BODY_PAN].setTime(Ti=10.0, Td=10.0)
        self.pid[BODY_PAN].setRange(InMin=-1.2, InMax=1.2, OutMin=-500, OutMax=500)
        self.pid[BODY_PAN].setSetPoints(0)

        self.pid[STEP_CORRECTION].setConstant(Kp=5.0, Ki=0.05, Kd=0.0)
        self.pid[STEP_CORRECTION].setTime(Ti=10.0, Td=10.0)
        self.pid[STEP_CORRECTION].setRange(InMin=-500, InMax=500, OutMin=-0.035, OutMax=0.035)
        self.pid[STEP_CORRECTION].setSetPoints(0)

        for i in range(TASK_CTRL_NUM):
            self.pid[i].Init()
            self.pid[i].setError(0)
            self.pid[i].setEnableWindUpCrossing()
            self.pid[i].setEnableWindUpLimit()

    def isFalling(self):
        if abs(Data.Motion.imuData.linear_acceleration.x) > 8.0:
            self.module.Motion_Stop()
            if Data.Motion.imuData.linear_acceleration.x > 8.0:  # back
                self.module.Motion_InitAction()
                self.module.MotionActionNum(ACTION_GETUPBACK)
                time.sleep(1.0)
            elif Data.Motion.imuData.linear_acceleration.x < 8.0:  # front
                self.module.Motion_InitAction()
                self.module.MotionActionNum(ACTION_GETUPFRONT)
                time.sleep(1.0)
            self.module.Motion_InitWalking()
            self.module.Motion_InitHead()
            return True
        elif -2.0 < Data.Motion.imuData.linear_acceleration.x < 2.0:
            return False

    def isBallFound(self):
        if Data.Head.ballFlag:
            self.module.headEnable(True)
            self.headScanCount = 0
            self.taskTimer     = Ticks()
            return True
        else:
            if self.findWithWalk:
                if self.headScanCount < 1:
                    self.module.Motion_WalkingParams(0.00, 0.00, 0.0, 0.55)
                    self.module.Motion_Start()
                else:
                    if Data.Head.ball_Last_x == 1:
                        self.module.Motion_WalkingParams(0.00, 0.00, -500, 0.55)
                        self.module.Motion_Start()
                    elif Data.Head.ball_Last_x == 2:
                        self.module.Motion_WalkingParams(0.00, 0.00, 500, 0.55)
                        self.module.Motion_Start()

            if Ticks() - self.taskTimer >= 2000:
                self.module.headEnable(False)
                self.headTrack.MotionHeadControl(self.headPanScan, self.headTiltScan)
                self.module.HeadJoint(Data.Head.headJointData)

                if self.headPanScan == 1.0 and self.headScanState == -1:
                    self.headScanState = 0
                elif self.headPanScan == -1.0 and self.headScanState == 0:
                    self.headScanState = 1
                if self.headTiltScan == -1.0 and self.headScanState == 1:
                    self.headScanState = 2
                elif self.headPanScan == 1.0 and self.headScanState == 2:
                    self.headScanState = 3
                elif self.headTiltScan == -0.3 and self.headScanState == 3:
                    self.headScanState = 0
                    self.headScanCount += 1

                if self.headScanState == 0:
                    self.headPanScan -= 0.25
                elif self.headScanState == 1:
                    self.headTiltScan -= 0.25
                elif self.headScanState == 2:
                    self.headPanScan += 0.25
                elif self.headScanState == 3:
                    self.headTiltScan += 0.25

                self.headPanScan  = max(-1.0, min(1.0,  self.headPanScan))
                self.headTiltScan = max(-1.0, min(-0.3, self.headTiltScan))
            return False

    def HeadScan(self):
        self.headTrack.MotionHeadControl(self.headPanScan, self.headTiltScan)

    def SystemPreStop(self):
        self.module.headEnable(False)
        self.module.Motion_Stop()
        self.module.LED_RGB(0, 0, 0)
        self.module.LED_Status(0, 0, 1)
        self.module.Buzzer(2000)
        time.sleep(0.2)
        self.module.Buzzer(0)
        self.module.headEnable(0)
        self.module.LED_RGB(0, 0, 0)
        self.module.LED_Status(1, 0, 0)
        Data.Systems.state = 'STOP'

    def SystemStop(self):
        self.outBodyReference = 0
        self.yawErrorFiltered = 0
        self.pid[BODY_REF].reset()
        self.module.headEnable(True)
        self.taskTimer = Ticks()

    def SystemPreIdle(self):
        self.module.Motion_InitPose()
        self.module.Buzzer(3000)
        time.sleep(0.2)
        self.module.Buzzer(2000)
        time.sleep(0.2)
        self.module.Buzzer(0)
        Data.Systems.state = 'IDLE'

    def SystemIdle(self):
        if Ticks() - self.taskTimer < 500:
            self.module.LED_Status(1, 0, 0)
            self.module.LED_RGB(0, 0, 0)
        elif Ticks() - self.taskTimer < 1000:
            self.module.LED_Status(0, 0, 0)
            self.module.LED_RGB(31, 0, 0)
        else:
            self.taskTimer = Ticks()

    def SystemPreStandby(self):
        self.module.LED_RGB(0, 0, 0)
        self.module.LED_Status(0, 0, 1)
        self.module.Motion_InitWalking()
        self.module.Motion_InitHead()
        self.module.Buzzer(2000)
        self.module.Motion_HeadControl(0.0, INIT_HEAD_TILT)
        self.module.Buzzer(3000)
        time.sleep(0.1)
        self.module.LED_Status(1, 0, 0)
        self.module.LED_RGB(0, 0, 0)
        self.module.Buzzer(0)
        Data.Systems.state = 'STANDBY'

    def SystemStandby(self):
        Data.Systems.state = 'STOP'

    def SystemPreRun(self):
        self.module.LED_Status(0, 1, 0)
        self.module.Motion_HeadControl(0.0, INIT_HEAD_TILT)
        self.module.EnableCtrlModule("head_control_module")
        self.module.Motion_InitWalking()
        self.module.Motion_InitHead()
        self.module.headEnable(True)
        self.taskTimer     = Ticks()
        Data.Systems.state = 'RUN'
        Data.Systems.task  = 'GO'
