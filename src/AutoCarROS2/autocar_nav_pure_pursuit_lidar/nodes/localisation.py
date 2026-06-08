#!/usr/bin/env python3
"""Odometry localisation with optional map-based pose correction (lap 2+)."""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from autocar_msgs.msg import State2D
from autocar_nav_pure_pursuit.normalise_angle import normalise_angle


class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')

        self.state_pub = self.create_publisher(State2D, '/autocar/state2D', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/autocar/odom', self.odom_cb, 10)
        self.corr_sub = self.create_subscription(
            Pose2D, '/autocar/pose_correction', self.correction_cb, 10)

        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('update_frequency', 50.0)
        self.frequency = float(self.get_parameter('update_frequency').value)

        self.odom = None
        self.corr_x = 0.0
        self.corr_y = 0.0
        self.corr_yaw = 0.0

    def correction_cb(self, msg: Pose2D):
        self.corr_x = float(msg.x)
        self.corr_y = float(msg.y)
        self.corr_yaw = float(msg.theta)

    def odom_cb(self, msg: Odometry):
        self.odom = msg
        self._publish()

    def _publish(self):
        if self.odom is None:
            return

        theta = 2.0 * np.arctan2(
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w)
        theta = normalise_angle(theta)

        state = State2D()
        state.pose.x = self.odom.pose.pose.position.x + self.corr_x
        state.pose.y = self.odom.pose.pose.position.y + self.corr_y
        state.pose.theta = normalise_angle(theta + self.corr_yaw)
        state.twist.x = self.odom.twist.twist.linear.x
        state.twist.y = self.odom.twist.twist.linear.y
        state.twist.w = -self.odom.twist.twist.angular.z
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Localisation()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
