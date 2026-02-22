#!/usr/bin/python
import sys
import math
import time
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String


class SshControl(Node):
    def __init__(self):
        super().__init__('ssh_control')
        self.pub_StateControl = self.create_publisher(String, 'robotis/ssh_control', 1)
        self.state = 'NONE'
        self.RunControl()

    def RunControl(self):
        try:
            self.state = 'NONE'
            while rclpy.ok():
                try:
                    print('CONTROL MENU')
                    print('0. PRE_STOP')
                    print('1. PRE_RUN')
                    print('2. PRE_STANDBY')
                    print('3. PRE_IDLE')
                    print('4. PRE_READY')
                    print('5. NONE')
                    input_state = str(input('INPUT CONTROL: '))
                    if input_state == '0':
                        self.state = 'PRE_STOP'
                    elif input_state == '1':
                        self.state = 'PRE_RUN'
                    elif input_state == '2':
                        self.state = 'PRE_STANDBY'
                    elif input_state == '3':
                        self.state = 'PRE_IDLE'
                    elif input_state == '4':
                        self.state = 'PRE_READY'
                    else:
                        self.state = 'NONE'
                    self.get_logger().warn('ROBOT_STATE: {}'.format(self.state))
                    msg = String()
                    msg.data = self.state
                    self.pub_StateControl.publish(msg)
                except (NameError, SyntaxError) as e:
                    self.get_logger().error('WRONG INPUT: {}'.format(e))
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SshControl()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
