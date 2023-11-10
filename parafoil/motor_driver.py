#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import pigpio
from rclpy.node import Node
from parafoil.tool import map_value
from parafoil_msgs.msg import Control


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
        self.pi.set_PWM_frequency(self.motor_pin, 50)
        self.pi.set_PWM_frequency(self.left_servo_pin, 50)
        self.pi.set_PWM_frequency(self.right_servo_pin, 50)
        self.pi.set_PWM_range(self.motor_pin, 10000)
        self.pi.set_PWM_range(self.left_servo_pin, 10000)
        self.pi.set_PWM_range(self.right_servo_pin, 10000)
        self.pi.set_PWM_dutycycle(self.motor_pin, 500)
        self.pi.set_PWM_dutycycle(self.left_servo_pin, map_value(70, 0, 180, 250, 1250))
        self.pi.set_PWM_dutycycle(self.right_servo_pin, map_value(120, 0, 180, 250, 1250))

        self.subscription = self.create_subscription(Control, 'parafoil/control', self.callback, 10)

    def callback(self, message):
        motor_duty_cycle = map_value(message.throttle, 0, 100, 500, 1000)

        if message.left_servo >= 0:
            left_servo_angle = map_value(100 - message.left_servo, 0, 100, 35, 70)
        else:
            left_servo_angle = map_value(-message.left_servo, 0, 100, 70, 140)
        left_servo_duty_cycle = map_value(left_servo_angle, 0, 180, 250, 1250)

        if message.right_servo >= 0:
            right_servo_angle = map_value(message.right_servo, 0, 100, 120, 155)
        else:
            right_servo_angle = map_value(message.right_servo, -100, 0, 50, 120)
        right_servo_duty_cycle = map_value(right_servo_angle, 0, 180, 250, 1250)

        self.pi.set_PWM_dutycycle(self.motor_pin, motor_duty_cycle)
        self.pi.set_PWM_dutycycle(self.left_servo_pin, left_servo_duty_cycle)
        self.pi.set_PWM_dutycycle(self.right_servo_pin, right_servo_duty_cycle)


def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriver()
    rclpy.spin(motor_driver)
    motor_driver.destroy_node()
    rclpy.shutdown()
