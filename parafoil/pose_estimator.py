#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from parafoil_msgs.msg import Pose


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.subscription = self.create_subscription(Pose, 'parafoil/pose/raw', self.callback, 10)
        self.publisher = self.create_publisher(Pose, 'parafoil/pose', 10)

    def callback(self, message):
        pass


def main(args=None):
    rclpy.init(args=args)
    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    pose_estimator.destroy_node()
    rclpy.shutdown()
