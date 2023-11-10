#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
from parafoil.tool import map_value
from parafoil_msgs.msg import Control, RCIn


class Autopilot(Node):
    def __init__(self):
        super().__init__('autopilot')
        self.declare_parameter('control_rate', 20)
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().integer_value

        self.arm = False  # False: disarm; True: arm
        self.arm_status = 0  # 0: no change; 1: arming; -1: disarming
        self.last_arm = None
        self.last_ctrl = time.time()
        self.publisher = self.create_publisher(Control, 'parafoil/control', 10)
        self.subscription = self.create_subscription(RCIn, 'parafoil/rc/in', self.callback, 10)

    def callback(self, message):
        channels = message.channels
        channel1, channel2, channel3, channel4 = channels[:4]

        if channel1 > 1600 and channel2 > 1600 and channel3 < 400 and channel4 < 400:  # arm
            if self.arm_status != 1:
                self.arm_status = 1
                self.last_arm = time.time()
            else:
                if time.time() - self.last_arm > 2:
                    self.arm = True
        elif channel1 < 400 and channel2 > 1600 and channel3 < 400 and channel4 > 1600:  # disarm
            if self.arm_status != -1:
                self.arm_status = -1
                self.last_arm = time.time()
            else:
                if time.time() - self.last_arm > 2:
                    self.arm = False
        else:
            self.arm_status = 0

        if time.time() - self.last_ctrl > 1 / self.control_rate:
            control = Control()
            if self.arm:
                delta = map_value(channel1, 994 - 650, 994 + 650, -1, 1)
                sigma = map_value(channel2, 1002 - 650, 1002 + 650, -1, 1)

                throttle = map_value(channel3, 306, 1693, 0, 100)
                left_servo = map_value(sigma + delta, -1, 1, -100, 100)
                right_servo = map_value(sigma - delta, -1, 1, -100, 100)

                control.throttle = int(throttle)
                control.left_servo = int(left_servo)
                control.right_servo = int(right_servo)
            else:
                control.throttle = 0
                control.left_servo = 0
                control.right_servo = 0
            control.timestamp = time.time()
            self.publisher.publish(control)
            self.last_ctrl = time.time()


def main(args=None):
    rclpy.init(args=args)
    autopilot = Autopilot()
    rclpy.spin(autopilot)
    autopilot.destroy_node()
    rclpy.shutdown()
