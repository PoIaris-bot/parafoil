#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import rclpy
import numpy as np
from rclpy.node import Node
from parafoil_msgs.msg import Pose, RCIn, Control


class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        self.declare_parameter('filepath', '/home/pi/ros2_ws/src/parafoil/log')
        self.filepath = self.get_parameter('filepath').get_parameter_value().string_value
        if not os.path.exists(self.filepath):
            os.makedirs(self.filepath)

        self.logging = False

        self.pose_raw = []
        self.pose = []
        self.control = []
        self.rc_in = []

        self.pose_raw_subscription = self.create_subscription(Pose, 'parafoil/pose/raw', self.pose_raw_callback, 10)
        self.pose_subscription = self.create_subscription(Pose, 'parafoil/pose', self.pose_callback, 10)
        self.control_subscription = self.create_subscription(Control, 'parafoil/control', self.control_callback, 10)
        self.rc_in_subscription = self.create_subscription(RCIn, 'parafoil/rc/in', self.rc_in_callback, 10)

    def pose_raw_callback(self, message):
        if self.logging:
            self.pose_raw.append([
                message.timestamp,
                message.id,
                *message.position,
                *message.angle,
                *message.quaternion,
                *message.velocity,
                *message.angular_velocity,
                *message.acceleration
            ])

    def pose_callback(self, message):
        if self.logging:
            self.pose.append([
                message.timestamp,
                message.id,
                *message.position,
                *message.angle,
                *message.quaternion,
                *message.velocity,
                *message.angular_velocity,
                *message.acceleration
            ])

    def control_callback(self, message):
        if self.logging:
            self.control.append([
                message.timestamp,
                message.throttle,
                message.left_servo,
                message.right_servo
            ])

    def rc_in_callback(self, message):
        if self.logging:
            self.rc_in.append([
                message.timestamp,
                *message.channels
            ])

        if message.channels[9] < 1000:  # channel 10 (A) up
            self.logging = True
        else:  # channel 10 (A) down
            self.logging = False
            saved = False
            if self.pose_raw:
                filename = os.path.join(
                    self.filepath, f'{time.strftime("pose_raw_%Y%m%d%H%M%S", time.localtime())}.txt'
                )
                np.savetxt(filename, np.array(self.pose_raw), delimiter=',')
                saved = True
                self.pose_raw = []
            if self.pose:
                filename = os.path.join(self.filepath, f'{time.strftime("pose_%Y%m%d%H%M%S", time.localtime())}.txt')
                np.savetxt(filename, np.array(self.pose), delimiter=',')
                saved = True
                self.pose = []
            if self.control:
                filename = os.path.join(self.filepath, f'{time.strftime("control_%Y%m%d%H%M%S", time.localtime())}.txt')
                np.savetxt(filename, np.array(self.control), delimiter=',')
                saved = True
                self.control = []
            if self.rc_in:
                filename = os.path.join(self.filepath, f'{time.strftime("rc_in_%Y%m%d%H%M%S", time.localtime())}.txt')
                np.savetxt(filename, np.array(self.rc_in), delimiter=',')
                saved = True
                self.rc_in = []
            if saved:
                self.get_logger().info(f'Log saved to {self.filepath}')


def main(args=None):
    rclpy.init(args=args)
    logger = Logger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()
