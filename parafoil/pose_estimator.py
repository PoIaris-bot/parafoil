#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import numpy as np
from rclpy.node import Node
from parafoil_msgs.msg import Pose
from parafoil.tool import remap_angle


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.subscription = self.create_subscription(Pose, 'parafoil/pose/raw', self.callback, 10)
        self.publisher = self.create_publisher(Pose, 'parafoil/pose/ekf', 10)

        self.x = None
        self.A_tilde = np.block([
            [np.zeros((6, 6)), np.eye(6)],
            [np.zeros((6, 6)), np.zeros((6, 6))]
        ])
        self.A = None
        self.C = np.eye(12)
        self.P = np.eye(12)
        self.Q_tilde = 1e-2 * np.eye(12)
        self.Q = None
        self.R = 1e-2 * np.eye(12)
        self.timestamp = None

    def callback(self, message):
        y = np.array([[
            *message.position,
            *message.angle,
            *message.velocity,
            *message.angular_velocity
        ]]).T
        if self.x is None:
            self.x = y
        else:
            dt = message.timestamp - self.timestamp
            self.A = self.A_tilde * dt + np.eye(12)
            self.Q = self.Q_tilde * dt

            self.x = np.dot(self.A, self.x)
            self.x[3:6, 0] = remap_angle(self.x[3:6, 0])
            self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

            innovation = y - np.dot(self.C, self.x)
            lambda_t = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
            kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(lambda_t))
            self.x = self.x + np.dot(kalman_gain, innovation)
            self.x[3:6, 0] = remap_angle(self.x[3:6, 0])
            self.P = self.P - np.dot(np.dot(kalman_gain, self.C), self.P)
        pose = Pose()
        pose.timestamp = message.timestamp
        pose.id = message.id
        pose.position = self.x[0:3, 0].tolist()
        pose.angle = self.x[3:6, 0].tolist()
        pose.velocity = self.x[6:9, 0].tolist()
        pose.angular_velocity = self.x[9:12, 0].tolist()
        pose.quaternion = message.quaternion
        pose.acceleration = message.acceleration
        self.publisher.publish(pose)
        self.timestamp = message.timestamp


def main(args=None):
    rclpy.init(args=args)
    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    pose_estimator.destroy_node()
    rclpy.shutdown()
