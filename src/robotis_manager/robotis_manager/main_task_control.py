#!/usr/bin/python

import rclpy
from rclpy.node import Node
import sys
import math
import time
import socket
import numpy as np

from robotis_manager.motion_PID import PID, PIDControl
from robotis_manager.motion_module import MotionModule, ACTION_RKICK, ACTION_GETUPFRONT, ACTION_GETUPBACK
from robotis_manager.motion_data_flow_handler import Data, Ticks, TASKSTA_STANDBY, TASKSTA_STOP, TASKSTA_RUN
from robotis_manager.motion_utility import MotionUtility as util
from robotis_manager.motion_head_control_module import HeadTracking, HEAD_PAN, CTRL_NUM, KF_BALL_POS_X, KF_BALL_POS_Y
from robotis_manager.motion_control_module import BaseControl, BODY_REF, BODY_PAN, STEP_CORRECTION, mapF
from robotis_manager.motion_control_module import INIT_HEAD_TILT, KF_YAW_ERROR, BALL_SETPOINT, STEP_SPEED_X, KF_BALL_SIZE, KF_BALL_POS
from robotis_manager.robocup_game_controller_handler import GameControllerListener
from robotis_manager.robocup_udp_controller_handler import UDPClient


class TuningModule:
    def __init__(self):
        self.kfBallSizeToKick            = 30.25
        self.kfBallSizeToSlowDown        = 44.25
        self.kfStepSpeed                 = 0.02
        self.kfReadyStepSpeed            = 0.0250
        self.kfStepPeriod                = 0.60
        self.kfStepCorrection            = 0.020
        self.kfBodyPanCorrection         = 0.15
        self.kfOffSetYKickBall           = 0.0090
        self.kfOffSetXKickBall           = 0.0060
        self.kfBallLostTime              = 7500
        self.kfMaxPositionTime           = 15000
        self.kfPositionBallLostTime      = 5000
        self.kfPositionBallDisappearTime = 1500
        self.headScanTimmerTresh         = 1500


class TaskControl(Node, BaseControl, TuningModule):
    def __init__(self, isUsingGameController=False, isUsingSSH=False,
                 isKeeperRobot=False, isUsingUDController=False):
        Node.__init__(self, 'task_control')
        BaseControl.__init__(self)
        TuningModule.__init__(self)

        self.isKeeperRobot      = isKeeperRobot
        self.setYawOffset       = True
        self.stateSSH           = "NONE"
        self.stateSSHBefore     = "NONE"
        self.isUsingSSH         = isUsingSSH

        self.udp_client              = None
        self.isUsingUDPController    = isUsingUDController

        if self.isUsingUDPController:
            robot_address    = ("192.168.0.186", 9979)
            self.udp_client  = UDPClient(robot_address)
            self.system_state_udp = {
                'PLAY':   self.SystemPreRun,
                'FINISH': self.SystemPreStop,
                'READY':  self.SystemPreReady
            }

        self.listener            = None
        self.isUsingGameController = isUsingGameController
        if self.isUsingGameController:
            self.listener        = GameControllerListener()
            self.system_state_game = {
                'PLAY':   self.SystemPreRun,
                'FINISH': self.SystemPreStop,
                'READY':  self.SystemPreReady
            }

        self.system_state_functions = {
            'PRE_RUN':     self.SystemPreRun,
            'PRE_STOP':    self.SystemPreStop,
            'PRE_IDLE':    self.SystemPreIdle,
            'PRE_STANDBY': self.SystemPreStandby,
            'PRE_READY':   self.SystemPreReady,
            'STANDBY':     self.SystemStandby,
            'IDLE':        self.SystemIdle,
            'STOP':        self.SystemStop,
            'RUN':         self.SystemRun,
            'READY':       self.SystemReady
        }

        self.system_state_ssh = {
            'PRE_RUN':     self.SystemPreRun,
            'PRE_STOP':    self.SystemPreStop,
            'PRE_IDLE':    self.SystemPreIdle,
            'PRE_STANDBY': self.SystemPreStandby,
            'PRE_READY':   self.SystemPreReady,
            'READY':       self.SystemReady
        }

        self.get_logger().info('Bismillah Jalan && Checking Gyroscope...')
        self.yawOffset = Data.Motion.gyroYawIntegrator
        if abs(Data.Motion.gyroYawIntegrator - self.yawOffset) > 1:
            self.get_logger().warn('Gyroscope NOT STABLE!')
        else:
            self.get_logger().info('Gyroscope OK')
        time.sleep(0.5)

        # Timer loop @ 50 Hz
        self.create_timer(0.02, self.loop)

    def loop(self):
        try:
            # gyro pid for correction
            self.yawErrorFiltered  = self.yawOffset - Data.Motion.gyroYawIntegrator
            self.yawErrorFiltered  = (self.yawErrorFiltered * KF_YAW_ERROR) + self.yawErrorFiltered
            self.yawErrorFiltered /= KF_YAW_ERROR + 1

            # limit angle
            if self.yawErrorFiltered > 1100:
                self.yawErrorFiltered = 1100
            elif self.yawErrorFiltered < -1100:
                self.yawErrorFiltered = -1100

            self.pid[BODY_REF].calculate(self.yawErrorFiltered)
            self.outBodyReference = self.pid[BODY_REF].getOutput()
            self.outBodyReference = mapF(self.outBodyReference, -500, 500, 500, -500)

            self.pid[BODY_PAN].calculate(Data.Head.headPan)
            self.outBodyPanning = self.pid[BODY_PAN].getOutput()
            self.outBodyPanning = mapF(self.outBodyPanning, -500, 500, 500, -500)

            self.pid[STEP_CORRECTION].calculate(self.yawErrorFiltered)
            self.outStepCorrection = self.pid[STEP_CORRECTION].getOutput()

            self.fallStatus = self.isFalling()
            self.SystemSSHSetState()

            if self.setYawOffset or Data.Systems.state == "PRE_STOP":
                self.yawErrorFiltered            = 0
                Data.Motion.gyroYawIntegrator    = 0
                self.yawOffset                   = Data.Motion.gyroYawIntegrator
                self.get_logger().warn("Set yawOffset Reference: {}".format(self.yawOffset))
                self.setYawOffset                = False

        except socket.error as e:
            if e.errno == 4:
                pass
            else:
                self.get_logger().error("[ERROR] {}".format(e))

    def SystemSSHSetState(self):
        # using open-cr button
        state_func = self.system_state_functions.get(Data.Systems.state, lambda: None)
        state_func()
        # using ssh
        if self.stateSSH != self.stateSSHBefore:
            self.stateSSH = self.stateSSHBefore
            state_ssh = self.system_state_ssh.get(self.stateSSH, lambda: None)
            state_ssh()
        self.stateSSHBefore = Data.Systems.ssh_control
        # using game controller
        if self.isUsingGameController:
            Data.Com.gameState = self.listener.receive()
            if Data.Com.gameState is not None:
                state_game = self.system_state_game.get(Data.Com.gameState, lambda: None)
                state_game()
        # using udp
        if self.isUsingUDPController:
            state_udp = self.udp_client.run()
            state_udp = self.system_state_udp.get(state_udp, lambda: None)
            state_udp()

    def SystemPreReady(self):
        self.module.Motion_WalkingParams(0.00, 0.0, self.outBodyReference, 0.60)
        self.module.Motion_Start()
        self.taskTimer      = Ticks()
        Data.Systems.state  = 'READY'
        Data.Systems.task   = 'READY'

    def SystemReady(self):
        if Data.Systems.task == 'READY':
            self.module.Motion_WalkingParams(0.00, 0.00, self.outBodyReference, 0.60)
            self.module.Motion_Start()
            if Ticks() - self.taskTimer > 1000:
                self.taskTimer      = Ticks()
                Data.Systems.task   = 'GO'
        elif Data.Systems.task == 'GO':
            self.module.Motion_WalkingParams(self.kfReadyStepSpeed, 0.00, self.outBodyReference, 0.60)
            self.module.Motion_Start()
            if Ticks() - self.taskTimer > 12000:
                self.taskTimer = Ticks()
                self.module.Motion_Stop()
                Data.Systems.state = 'PRE_STOP'

    def SystemRun(self):
        # calculating and filtering for distance ball from robot
        self.ballSizeFiltered  = (self.ballSizeFiltered * KF_BALL_SIZE) + Data.Head.ballCZ
        self.ballSizeFiltered /= KF_BALL_SIZE + 1
        self.ballPosFiltered   = (self.ballPosFiltered * KF_BALL_POS) + Data.Head.ballCXFiltered
        self.ballPosFiltered  /= KF_BALL_POS + 1

        self.get_logger().info("ballSizeFiltered: {}".format(self.ballSizeFiltered))

        # y step correction for walking towards the ball
        self.xError = (self.ballPosFiltered * -1 - BALL_SETPOINT)
        self.xSpeed = STEP_SPEED_X / (1 + abs(self.xError / 5))
        self.xSpeed = self.xSpeed / (2 + (1 / 5))
        self.xError /= 5
        self.xError = max(-0.005, min(0.005, self.xError))
        self.xSpeed = max(-0.0335, min(0.0335, self.xSpeed))

        if Data.Systems.task == 'GO':
            if not self.isKeeperRobot:
                if not self.isBallFound():
                    self.HeadScan()
                    self.module.Motion_WalkingParams(0.00, 0.0, self.outBodyReference, 0.60)
                    self.module.Motion_Start()
                    if (Ticks() - self.taskTimer >= 1100 and Data.Head.ballFlag) or (
                            Ticks() - self.taskTimer >= self.headScanTimmerTresh):
                        self.taskTimer      = Ticks()
                        Data.Systems.task   = 'SCAN'
                else:
                    if self.pid[BODY_REF].getError() > 0.1:
                        self.module.Motion_WalkingParams(0.0, 0.0, self.outBodyReference, 0.60)
                        self.module.Motion_Start()
                    else:
                        if self.isBallFound():
                            if Data.Head.headPan > 0.225:
                                self.module.Motion_WalkingParams(0.00, 0.020, self.outBodyReference, 0.60)
                                self.module.Motion_Start()
                            elif Data.Head.headPan < -0.225:
                                self.module.Motion_WalkingParams(0.00, -0.020, self.outBodyReference, 0.60)
                                self.module.Motion_Start()
                            else:
                                self.module.Motion_Stop()
                            if self.ballSizeFiltered < self.kfBallSizeToKick or Data.Head.headTilt < -0.7:
                                self.CorrectionStateToKick()
                                self.taskTimer = Ticks()

        elif Data.Systems.task == 'SCAN':
            if self.isBallFound():
                if abs(self.pid[BODY_PAN].getError()) > self.kfBodyPanCorrection:
                    self.module.Motion_WalkingParams(0.00, 0.00, self.outBodyPanning, 0.60)
                    self.module.Motion_Start()
                else:
                    if self.ballSizeFiltered < self.kfBallSizeToKick:
                        Data.Systems.task = 'POSITION'
                        self.taskTimer    = Ticks()
                    elif self.kfBallSizeToKick <= self.ballSizeFiltered < self.kfBallSizeToSlowDown:
                        self.module.Motion_WalkingParams(0.010, 0.0, self.outBodyPanning, 0.60)
                        self.module.Motion_Start()
                    else:
                        self.module.Motion_WalkingParams(self.kfStepSpeed, 0.0, self.outBodyPanning, self.kfStepPeriod)
                        self.module.Motion_Start()

        elif Data.Systems.task == 'POSITION':
            if (not Data.Head.ballFlag and Ticks() - self.taskTimer >= self.kfBallLostTime) or (
                    Ticks() - self.taskTimer >= self.kfMaxPositionTime):
                Data.Systems.task = 'RETRY'
            else:
                if self.ballSizeFiltered < self.kfBallSizeToSlowDown and Data.Head.ballFlag:
                    if self.pid[BODY_REF].getError() > 0.1:
                        self.module.Motion_WalkingParams(0.0, self.kfStepCorrection, self.outBodyReference, 0.60)
                        self.module.Motion_Start()
                    elif self.pid[BODY_REF].getError() < -0.1:
                        self.module.Motion_WalkingParams(0.0, -self.kfStepCorrection, self.outBodyReference, 0.60)
                        self.module.Motion_Start()
                    else:
                        self.CorrectionStateToKick()
                else:
                    if Ticks() - self.taskTimer >= self.kfPositionBallDisappearTime:
                        Data.Systems.task = 'RETRY'

        elif Data.Systems.task == 'RETRY':
            self.module.Motion_InitWalking()
            Data.Systems.task = 'GO'
            self.taskTimer    = Ticks()

    def CorrectionStateToKick(self):
        if Data.Head.ballPos == 0:
            self.CorrectionWalk(self.kfOffSetXKickBall, self.kfOffSetYKickBall)
        elif Data.Head.ballPos == 1 and self.ballSizeFiltered < 30.25:
            self.KickBall("LEFT" if Data.Head.ballCX < 225 else "RIGHT")
        elif Data.Head.ballPos in [2, 3, 6]:
            self.CorrectionWalk(self.kfOffSetXKickBall, -self.kfOffSetYKickBall)
        elif Data.Head.ballPos == 4:
            self.CorrectionWalk(self.kfOffSetXKickBall, self.kfOffSetYKickBall)
        elif Data.Head.ballPos == 5 and self.ballSizeFiltered < 30.25:
            self.KickBall("LEFT" if Data.Head.ballCX < 225 else "RIGHT")
        elif Data.Head.ballPos == -1:
            if Ticks() - self.taskTimer >= self.kfPositionBallLostTime:
                Data.Systems.task = 'RETRY'

    def CorrectionWalk(self, x_step, y_step):
        self.taskTimer = Ticks()
        self.module.Motion_WalkingParams(x_step, y_step, self.outBodyReference, 0.60)

    def KickBall(self, direction):
        self.module.Motion_Stop()
        self.module.Motion_InitAction()
        self.module.MotionActionNum(80)
        if Ticks() - self.taskTimer >= 1000:
            self.module.MotionActionNum(134 if direction == "LEFT" else ACTION_RKICK)
            Data.Systems.task = 'RETRY'
            self.taskTimer    = Ticks()

    @staticmethod
    def ExponentialFilter(value, factor, source, lower_limit, upper_limit):
        value  = (value * factor) + source
        value /= factor + 1
        value  = min(max(value, lower_limit), upper_limit)
        return value


def main(args=None):
    rclpy.init(args=args)
    node = TaskControl()
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
