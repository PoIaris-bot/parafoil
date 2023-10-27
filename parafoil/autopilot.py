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

        self.control_rate = 20
        self.last_pub = time.time()
        self.publisher = self.create_publisher(Control, 'control', 10)
        self.subscription = self.create_subscription(RCIn, 'rc_in', self.callback, 10)

    def callback(self, message):
        channels = message.channels
        channel1, channel2, channel3 = channels[:3]

        channel1 = map_value(channel1, 994 - 650, 994 + 650, -1, 1)
        channel2 = map_value(channel2, 1002 - 650, 1002 + 650, -1, 1)

        throttle = map_value(channel3, 306, 1693, 0, 100)
        left_servo_angle = map_value(channel2 + channel1, -1, 1, 0, 180)
        right_servo_angle = map_value(channel2 - channel1, -1, 1, 0, 180)

        if time.time() - self.last_pub > 1 / self.control_rate:
            control = Control()
            control.throttle = int(throttle)
            control.left_servo_angle = int(left_servo_angle)
            control.right_servo_angle = int(right_servo_angle)
            self.publisher.publish(control)
            self.last_pub = time.time()

    def stop(self):
        control = Control()
        control.throttle = 0
        control.left_servo_angle = 90
        control.right_servo_angle = 90
        self.publisher.publish(control)


def main(args=None):
    rclpy.init(args=args)
    autopilot = Autopilot()
    rclpy.spin(autopilot)
    autopilot.stop()
    autopilot.destroy_node()
    rclpy.shutdown()
