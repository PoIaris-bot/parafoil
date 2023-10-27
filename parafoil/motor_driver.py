#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import pigpio
from rclpy.node import Node
from parafoil.tool import map_value
from parafoil_msgs.msg import Control

FREQUENCY = 50
RANGE = 10000

THROTTLE_PWM_MIN = 500
THROTTLE_PWM_MAX = 1000

ANGLE_PWM_MIN = 250
ANGLE_PWM_MAX = 1250

ANGLE_MIN = 75
ANGLE_MAX = 145
ANGLE_MID = (ANGLE_MIN + ANGLE_MAX) / 2
ANGLE_OFFSET = 10


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.declare_parameter('motor_pin', 18)
        self.declare_parameter('left_servo_pin', 23)
        self.declare_parameter('right_servo_pin', 24)

        self.motor_pin = self.get_parameter('motor_pin').get_parameter_value().integer_value
        self.left_servo_pin = self.get_parameter('left_servo_pin').get_parameter_value().integer_value
        self.right_servo_pin = self.get_parameter('right_servo_pin').get_parameter_value().integer_value

        self.pi = pigpio.pi()
        self.pi.set_mode(self.motor_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.left_servo_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.right_servo_pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.motor_pin, FREQUENCY)
        self.pi.set_PWM_frequency(self.left_servo_pin, FREQUENCY)
        self.pi.set_PWM_frequency(self.right_servo_pin, FREQUENCY)
        self.pi.set_PWM_range(self.motor_pin, RANGE)
        self.pi.set_PWM_range(self.left_servo_pin, RANGE)
        self.pi.set_PWM_range(self.right_servo_pin, RANGE)
        self.pi.set_PWM_dutycycle(self.motor_pin, THROTTLE_PWM_MIN)
        self.pi.set_PWM_dutycycle(self.left_servo_pin, map_value(180 - ANGLE_MID, 0, 180, ANGLE_PWM_MIN, ANGLE_PWM_MAX))
        self.pi.set_PWM_dutycycle(
            self.right_servo_pin, map_value(ANGLE_MID + ANGLE_OFFSET, 0, 180, ANGLE_PWM_MIN, ANGLE_PWM_MAX)
        )

        self.subscription = self.create_subscription(Control, 'control', self.callback, 10)

    def callback(self, message):
        left_servo_angle = 180 - map_value(message.left_servo_angle, 0, 180, ANGLE_MIN, ANGLE_MAX)
        right_servo_angle = map_value(
            message.right_servo_angle, 0, 180, ANGLE_MIN + ANGLE_OFFSET, ANGLE_MAX + ANGLE_OFFSET
        )

        self.pi.set_PWM_dutycycle(self.motor_pin,
                                  map_value(message.throttle, 0, 100, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX))
        self.pi.set_PWM_dutycycle(
            self.left_servo_pin, map_value(left_servo_angle, 0, 180, ANGLE_PWM_MIN, ANGLE_PWM_MAX)
        )
        self.pi.set_PWM_dutycycle(
            self.right_servo_pin, map_value(right_servo_angle, 0, 180, ANGLE_PWM_MIN, ANGLE_PWM_MAX)
        )

    def stop(self):
        self.pi.set_PWM_dutycycle(self.motor_pin, 500)
        self.pi.set_PWM_dutycycle(self.left_servo_pin, map_value(180 - ANGLE_MID, 0, 180, ANGLE_PWM_MIN, ANGLE_PWM_MAX))
        self.pi.set_PWM_dutycycle(
            self.right_servo_pin, map_value(ANGLE_MID + ANGLE_OFFSET, 0, 180, ANGLE_PWM_MIN, ANGLE_PWM_MAX)
        )


def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriver()
    rclpy.spin(motor_driver)
    motor_driver.stop()
    motor_driver.destroy_node()
    rclpy.shutdown()
