#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
import serial
from rclpy.node import Node
from parafoil_msgs.msg import Pose
from serial.serialutil import SerialException


class RadioTelemetry(Node):
    def __init__(self):
        super().__init__('radio_telemetry')
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 57600)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        try:
            self.serial = serial.Serial(self.port, self.baud_rate)
        except SerialException:
            self.serial = None

        self.declare_parameter('rate', 15)
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.timestamp = time.time()

        self.declare_parameter('topic', 'parafoil/pose/ekf')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(Pose, topic, self.callback, 10)

    def callback(self, message):
        if time.time() - self.timestamp > 1 / self.rate:
            data = [
                message.timestamp,
                message.id,
                *message.position,
                *message.velocity,
                *message.angle,
                *message.quaternion,
                *message.acceleration,
                *message.angular_velocity
            ]
            if self.serial is not None:
                try:
                    self.serial.write(str(data).encode())
                    self.timestamp = time.time()
                except SerialException:
                    self.serial.close()
                    self.serial = None
            else:
                try:
                    self.serial = serial.Serial(self.port, self.baud_rate)
                except SerialException:
                    self.serial = None


def main(args=None):
    rclpy.init(args=args)
    radio_telemetry = RadioTelemetry()
    rclpy.spin(radio_telemetry)
    radio_telemetry.destroy_node()
    rclpy.shutdown()
