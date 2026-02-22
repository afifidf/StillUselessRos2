#!/usr/bin/python

import rclpy
import time
import math
import sys
import signal
import random
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Bool, Float64, Float32MultiArray, String
from std_msgs.msg import Int32MultiArray, Int32, Int8, Int8MultiArray, MultiArrayDimension
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState, Imu
from robotis_controller_msgs.msg import SyncWriteItem
from op3_walking_module_msgs.msg import WalkingParam

from robotis_manager.motion_utility import MotionUtility

# MACROS
TASKSTA_STARTUP = "startup"
TASKSTA_ERROR   = "error"
TASKSTA_ESTOP   = "estop"
TASKSTA_STOP    = "stop"
TASKSTA_STANDBY = "standby"
TASKSTA_RUN     = "run"

MTNMODE_NONE   = "none"
MTNMODE_WALK   = "walk"
MTNMODE_ACTION = "action"
MTNMODE_HEAD   = "head"


class Data(object):  # static data class
    class Head:
        ballCX = 0.0
        ballCY = 0.0
        ballCZ = 0.0

        ballCXFiltered = 0.0
        ballCYFiltered = 0.0
        ballPan  = None
        ballTilt = None
        ballKick = None
        ballFlag = None
        ballPos  = None
        ball_Last_x = None
        ball_Last_y = None

        ballFilterX = 0.0
        ballFilterY = 0.0

        bodyPan = 0.0

        headTilt = 0.0
        headPan  = 0.0

        frameXsize = 0
        frameYsize = 0
        headStatus = 0

        headJointData = JointState()
        headJointData.name = ['head_pan', 'head_tilt']
        headJointData.position = [0.0, 0.0]
        headPubUpdate = False

    class Motion:
        gyroYawIntegrator = 0
        accYawIntegrator  = 0
        imuData    = Imu()
        orientData = Vector3()

        motionMode       = MTNMODE_NONE
        taskStatus       = TASKSTA_STARTUP
        WalkingParam_msg = WalkingParam()
        walkingPubUpdate = False

    class Systems:
        taskTimer   = 0
        state       = "PRE_STANDBY"
        task        = "NONE"
        ssh_control = "NONE"

    class Com:
        gameState     = None
        previousState = None


class RosHandler(Node):
    def __init__(self, node_name='ros_handler'):
        super().__init__(node_name)

        # --- Publishers ---
        self.pub_WalkingCommand   = self.create_publisher(String,       '/robotis/walking/command',                  1)
        self.pub_InitPose         = self.create_publisher(String,       '/robotis/base/ini_pose',                    1)
        self.pub_TaskStatus       = self.create_publisher(String,       '/manuvering/status',                        1)
        self.pub_EnableCtrlModule = self.create_publisher(String,       '/robotis/enable_ctrl_module',               1)
        self.pub_ManWalkingParams  = self.create_publisher(WalkingParam, '/manuvering/walking_params',               1)
        self.pub_HeadPan          = self.create_publisher(Float64,      '/manuvering/head_pan',                      1)
        self.pub_HeadTilt         = self.create_publisher(Float64,      '/manuvering/head_tilt',                     1)
        self.pub_ManHeadEnable    = self.create_publisher(Bool,         '/manuvering/head_enable',                   1)
        self.pub_ManActionNum     = self.create_publisher(Int32,        '/manuvering/action_num',                    1)

        self.pub_HeadScanCtrl  = self.create_publisher(Bool,            '/manuvering/head_scan',                     1)
        self.pub_HeadScan      = self.create_publisher(String,          'robotis/head_control/scan_command',         1)
        self.pub_HeadJoint     = self.create_publisher(JointState,      'robotis/head_control/set_joint_states',     1)
        self.pub_PageNum       = self.create_publisher(Int32,           '/robotis/action/page_num',                  1)
        self.pub_WalkingParams = self.create_publisher(WalkingParam,    '/robotis/walking/set_params',               1)

        self.pub_YawPos        = self.create_publisher(Float64,         '/manuvering/sensor/gyro/yaw',               1)
        self.pub_YawLin        = self.create_publisher(Float64,         '/manuvering/sensor/accel/yaw',              1)

        self.pub_ManCommand    = self.create_publisher(String,          '/manuvering/command',                       1)
        self.pub_ManMotionMode = self.create_publisher(String,          '/manuvering/motion_mode',                   1)
        self.pub_Mode          = self.create_publisher(Int32,           '/manuvering/mode',                          1)
        self.pub_BodyPanMove   = self.create_publisher(Float64,         '/manuvering/body_pan_move',                 1)
        self.pub_SyncWrite     = self.create_publisher(SyncWriteItem,   '/robotis/sync_write_item',                  1)

        # --- Subscribers ---
        self.create_subscription(Int8MultiArray,   '/ball/ball_state',           self.__ballState,              10)
        self.create_subscription(Int16MultiArray,  '/ball/coordinate',           self.__coor_ball,              10)
        self.create_subscription(Float32MultiArray,'/ball/coordinate_filter',    self.__coor_ball_filtered,     10)
        self.create_subscription(Int32MultiArray,  '/frame/frame_size',          self.__frame_size,             10)
        self.create_subscription(Bool,             '/manuvering/head_enable',    self.__head_state,             10)
        self.create_subscription(Float64,          '/manuvering/head_pan',       self.__head_pan_callback,      10)
        self.create_subscription(Float64,          '/manuvering/head_tilt',      self.__head_tilt_callback,     10)
        self.create_subscription(Bool,             '/manuvering/head_scan',      self.__head_scan_callback,     10)
        self.create_subscription(String,           '/robotis/open_cr/button',    self.__opencr_button_callback, 10)
        self.create_subscription(Bool,             '/manuvering/e_stop',         self.__estop_callback,         10)
        self.create_subscription(String,           '/manuvering/motion_mode',    self.__motion_mode_callback,   10)
        self.create_subscription(String,           '/manuvering/command',        self.__command_mode_callback,  10)
        self.create_subscription(Imu,              '/robotis/open_cr/imu',       self.__imu_callback,           10)
        self.create_subscription(WalkingParam,     '/manuvering/walking_params', self.__walking_param_callback, 10)
        self.create_subscription(Int32,            '/manuvering/action_num',     self.__action_num_callback,    10)
        self.create_subscription(String,           '/robotis/ssh_control',       self.__ssh_control_callback,   10)

        self.get_logger().info("RosHandler initialized")

    # ---- Callbacks ----

    def __ballState(self, data):
        Data.Head.ballPan     = data.data[0]
        Data.Head.ballTilt    = data.data[1]
        Data.Head.ballKick    = data.data[2]
        Data.Head.ball_Last_x = data.data[3]
        Data.Head.ball_Last_y = data.data[4]
        Data.Head.ballFlag    = data.data[5]

    def __coor_ball(self, data):
        Data.Head.ballCX = data.data[0]
        Data.Head.ballCY = data.data[1]
        if not Data.Head.ballFlag:
            Data.Head.ballPos = -1
        else:
            if Data.Head.ballCX < 100:
                Data.Head.ballPos = 0
            elif 100 <= Data.Head.ballCX < 180:
                Data.Head.ballPos = 1
            elif 180 <= Data.Head.ballCX < 205:
                Data.Head.ballPos = 2
            elif 205 <= Data.Head.ballCX < 245:
                Data.Head.ballPos = 3
            elif 245 <= Data.Head.ballCX < 270:
                Data.Head.ballPos = 4
            elif 270 <= Data.Head.ballCX < 350:
                Data.Head.ballPos = 5
            elif Data.Head.ballCX >= 350:
                Data.Head.ballPos = 6

    def __coor_ball_filtered(self, data):
        Data.Head.ballCXFiltered = data.data[0]
        Data.Head.ballCYFiltered = data.data[1]
        Data.Head.ballCZ         = data.data[2]

    def __frame_size(self, data):
        Data.Head.frameYsize = data.data[0]
        Data.Head.frameXsize = data.data[1]

    def __head_state(self, data):
        Data.Head.headStatus = 1 if data.data else 0

    def __head_pan_callback(self, data):
        Data.Head.headPubUpdate = True
        Data.Head.headPan = data.data
        if data.data > 1.5:
            Data.Head.headJointData.position = [1.5, Data.Head.headJointData.position[1] if len(Data.Head.headJointData.position) > 1 else 0.0]
        elif data.data < -1.5:
            Data.Head.headJointData.position = [-1.5, Data.Head.headJointData.position[1] if len(Data.Head.headJointData.position) > 1 else 0.0]
        else:
            Data.Head.headJointData.position = [data.data, Data.Head.headJointData.position[1] if len(Data.Head.headJointData.position) > 1 else 0.0]

    def __head_tilt_callback(self, data):
        Data.Head.headPubUpdate = True
        Data.Head.headTilt = data.data
        if len(Data.Head.headJointData.position) < 2:
            Data.Head.headJointData.position.append(data.data)
        else:
            Data.Head.headJointData.position[1] = data.data

    def __head_scan_callback(self, data):
        pass

    def __opencr_button_callback(self, data):
        if data.data == "mode":
            if Data.Systems.state == 'STOP':
                Data.Systems.state = 'PRE_RUN'
            elif Data.Systems.state == 'RUN':
                Data.Systems.state = 'PRE_STOP'
        elif data.data == "start":
            if Data.Systems.state == 'PRE_STANDBY':
                Data.Systems.state = 'STANDBY'
        elif data.data == "user":
            if Data.Systems.state == 'STOP':
                Data.Systems.state = 'PRE_IDLE'
            elif Data.Systems.state == 'IDLE':
                Data.Systems.state = 'PRE_STANDBY'

    def __estop_callback(self, data):
        pass

    def __ssh_control_callback(self, data):
        Data.Systems.ssh_control = data.data

    def __motion_mode_callback(self, data):
        if data.data == MTNMODE_WALK:
            if Data.Motion.taskStatus in [TASKSTA_STANDBY, TASKSTA_STOP]:
                MotionUtility.EnableCtrlModule(self.pub_EnableCtrlModule, "walking_module")
                time.sleep(2)
                self.get_logger().info("Walking Module Enabled")
                Data.Motion.motionMode = MTNMODE_WALK
                Data.Motion.taskStatus = TASKSTA_STOP
        elif data.data == MTNMODE_ACTION:
            if Data.Motion.taskStatus in [TASKSTA_STANDBY, TASKSTA_STOP]:
                MotionUtility.EnableCtrlModule(self.pub_EnableCtrlModule, "action_module")
                time.sleep(0.1)
                self.get_logger().info("Action Module Enabled")
                Data.Motion.motionMode = MTNMODE_ACTION
                Data.Motion.taskStatus = TASKSTA_STOP
        elif data.data == MTNMODE_HEAD:
            MotionUtility.EnableCtrlModule(self.pub_EnableCtrlModule, "head_control_module")
            time.sleep(0.1)
            self.get_logger().info("Head Control Module Enabled")
        msg = String(); msg.data = Data.Motion.taskStatus
        self.pub_TaskStatus.publish(msg)

    def __command_mode_callback(self, data):
        if data.data == "start":
            if Data.Motion.taskStatus == TASKSTA_STOP:
                if Data.Motion.motionMode == MTNMODE_WALK:
                    MotionUtility.WalkingCommand(self.pub_WalkingCommand, "start")
                    self.get_logger().info("Walking Started")
                Data.Motion.taskStatus = TASKSTA_RUN
        elif data.data == "stop":
            if Data.Motion.motionMode == MTNMODE_WALK:
                MotionUtility.WalkingCommand(self.pub_WalkingCommand, "stop")
                self.get_logger().info("Walking Stopped")
                Data.Motion.taskStatus = TASKSTA_STOP
        elif data.data == "reset":
            if Data.Motion.taskStatus == TASKSTA_STOP:
                MotionUtility.InitPose(self.pub_InitPose)
                self.get_logger().info("Reset to Init Pose")
            elif Data.Motion.taskStatus == TASKSTA_RUN:
                MotionUtility.WalkingCommand(self.pub_WalkingCommand, "stop")
                self.get_logger().info("Walking Stopped")
                MotionUtility.InitPose(self.pub_InitPose)
                self.get_logger().info("Reset to Init Pose")
            Data.Motion.taskStatus = TASKSTA_STANDBY
        msg = String(); msg.data = Data.Motion.taskStatus
        self.pub_TaskStatus.publish(msg)

    def __imu_callback(self, data):
        Data.Motion.imuData = data
        Data.Motion.gyroYawIntegrator += Data.Motion.imuData.angular_velocity.z
        Data.Motion.accYawIntegrator  += Data.Motion.imuData.linear_acceleration.x

    def __walking_param_callback(self, data):
        Data.Motion.WalkingParam_msg, Data.Motion.walkingPubUpdate = MotionUtility.Walking(
            data.x_move_amplitude, data.y_move_amplitude,
            data.angle_move_amplitude, data.period_time)

    def __action_num_callback(self, data):
        self.pub_PageNum.publish(data)
        time.sleep(0.1)
        self.get_logger().info("Action Start")


def Ticks():
    return int(time.time() * 1000)


def main(args=None):
    rclpy.init(args=args)
    node = RosHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
