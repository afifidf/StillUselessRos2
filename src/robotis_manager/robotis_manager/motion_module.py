#!/usr/bin/python

import time
import sys
from std_msgs.msg import String, Int32, Bool, Float64
from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import SyncWriteItem
from op3_walking_module_msgs.msg import WalkingParam
from robotis_manager.motion_data_flow_handler import RosHandler, Data

# macros / constant
ACTION_STAND            = 1
ACTION_KEEPER           = 60
ACTION_STANDINIT        = 80
ACTION_SIT              = 15
ACTION_DEFENSERIGHT     = 61
ACTION_DEFENSELEFT      = 62
ACTION_RIGHTKICK        = 83
ACTION_LEFTKICK         = 84
ACTION_RIGHTKICKSLOW    = 14
ACTION_LEFTKICKSLOW     = 15
ACTION_RIGHTKICKSLOWEST = 19
ACTION_LEFTKICKSLOWEST  = 20
ACTION_SIDEKICKRIGHT    = 12
ACTION_GETUPFRONT       = 122
ACTION_GETUPBACK        = 123
ACTION_LKICKHIGH        = 120
ACTION_RKICKHIGH        = 121
ACTION_RIGHTKICKMED     = 94
ACTION_LEFTKICKMED      = 97
ACTION_RKICK            = 133
ACTION_LKICK            = 134


class MotionModule(RosHandler):
    def __init__(self, node_name='motion_module'):
        super().__init__(node_name)

    def EnableCtrlModule(self, module):
        msg = String(); msg.data = module
        self.pub_EnableCtrlModule.publish(msg)
        time.sleep(0.1)

    def InitPose(self):
        msg = String(); msg.data = 'ini_pose'
        self.pub_InitPose.publish(msg)
        time.sleep(4)

    def WalkingCommand(self, data):
        msg = String(); msg.data = data
        self.pub_WalkingCommand.publish(msg)
        time.sleep(0.1)

    def MotionActionNum(self, num):
        msg = Int32(); msg.data = num
        self.pub_ManActionNum.publish(msg)
        time.sleep(1.0)

    def Motion_BodyPanControl(self, bodyPan):
        msg = Float64(); msg.data = bodyPan
        self.pub_BodyPanMove.publish(msg)
        time.sleep(0.1)

    def Motion_HeadControl(self, pan, tilt):
        pan_msg = Float64();  pan_msg.data = pan
        self.pub_HeadPan.publish(pan_msg)
        time.sleep(0.1)
        tilt_msg = Float64(); tilt_msg.data = tilt
        self.pub_HeadTilt.publish(tilt_msg)
        time.sleep(0.1)

    def Motion_WalkingParams(self, x, y, o, t):
        msg = WalkingParam()
        msg.x_move_amplitude     = x
        msg.y_move_amplitude     = y
        msg.angle_move_amplitude = o
        msg.period_time          = t
        self.pub_ManWalkingParams.publish(msg)

    def HeadJoint(self, data):
        self.pub_HeadJoint.publish(data)

    def headEnable(self, num):
        msg = Bool(); msg.data = bool(num)
        self.pub_ManHeadEnable.publish(msg)

    def MotionCommand(self, cmd):
        msg = String(); msg.data = cmd
        self.pub_ManCommand.publish(msg)

    def MotionMode(self, mode):
        msg = String(); msg.data = mode
        self.pub_ManMotionMode.publish(msg)
        time.sleep(0.1)

    def Mode_Driver(self, num):
        msg = Int32(); msg.data = num
        self.pub_Mode.publish(msg)
        time.sleep(0.1)

    def LED_Status(self, r, g, b):
        msg = SyncWriteItem()
        msg.joint_name = ['open-cr']
        msg.item_name  = 'LED'
        msg.value      = [r + g * 2 + b * 4]
        self.pub_SyncWrite.publish(msg)
        time.sleep(0.01)

    def LED_RGB(self, r, g, b):
        msg = SyncWriteItem()
        msg.joint_name = ['open-cr']
        msg.item_name  = 'LED_RGB'
        msg.value      = [(r) | (g << 5) | (b << 10)]
        self.pub_SyncWrite.publish(msg)
        time.sleep(0.01)

    def Buzzer(self, freq):
        msg = SyncWriteItem()
        msg.joint_name = ['open-cr']
        msg.item_name  = 'buzzer'
        msg.value      = [freq]
        self.pub_SyncWrite.publish(msg)
        time.sleep(0.01)

    def Motion_InitWalking(self):
        self.MotionMode('walk')
        time.sleep(2.0)

    def Motion_InitAction(self):
        self.MotionMode('action')
        time.sleep(0.3)

    def Motion_InitHead(self):
        self.MotionMode('head')
        time.sleep(0.3)

    def Motion_Start(self):
        self.MotionCommand('start')

    def Motion_Stop(self):
        self.MotionCommand('stop')
        time.sleep(0.5)

    def Motion_InitPose(self):
        self.MotionCommand('reset')
        time.sleep(4)


class WalkingModule(MotionModule):
    def __init__(self):
        super().__init__('walking_module')


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MotionModule()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
