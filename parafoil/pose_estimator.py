#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from parafoil_msgs.msg import UWB


class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.subscription = self.create_subscription(UWB, 'uwb', self.callback, 10)

    def callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    pose_estimator = PoseEstimatorNode()
    rclpy.spin(pose_estimator)
    pose_estimator.destroy_node()
    rclpy.shutdown()
