#!/usr/bin/env python3
"""Odometry localisation with optional SLAM TF correction (in-memory map, no save)."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from autocar_msgs.msg import State2D
from autocar_nav_pure_pursuit.normalise_angle import normalise_angle
from autocar_nav_pure_pursuit_lidar.slam_pose import slam_pose_in_odom


class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')

        self.state_pub = self.create_publisher(State2D, '/autocar/state2D', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/autocar/odom', self.odom_cb, 10)

        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('update_frequency', 50.0)
        self.declare_parameter('use_slam', True)
        self.frequency = float(self.get_parameter('update_frequency').value)
        self.use_slam = bool(self.get_parameter('use_slam').value)

        self.odom = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._slam_warned = False

        period = 1.0 / self.frequency if self.frequency > 0.0 else 0.02
        self.timer = self.create_timer(period, self._publish)

        if self.use_slam:
            self.get_logger().info(
                'SLAM localisation: pose from map/odom TF (no map save).')
        else:
            self.get_logger().info('Wheel odometry only (use_slam=false).')

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def _wheel_pose(self) -> tuple[float, float, float] | None:
        if self.odom is None:
            return None
        theta = 2.0 * math.atan2(
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w)
        return (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            normalise_angle(theta),
        )

    def _slam_pose(self) -> tuple[float, float, float] | None:
        try:
            stamp = rclpy.time.Time()
            timeout = Duration(seconds=0.05)
            map_to_odom = self._tf_buffer.lookup_transform(
                'map', 'odom', stamp, timeout=timeout)
            map_to_base = self._tf_buffer.lookup_transform(
                'map', 'base_link', stamp, timeout=timeout)
        except Exception:
            if not self._slam_warned:
                self.get_logger().warn(
                    'Waiting for SLAM TF (map->odom, map->base_link)...',
                    throttle_duration_sec=5.0)
                self._slam_warned = True
            return None
        return slam_pose_in_odom(map_to_odom, map_to_base)

    def _publish(self):
        if self.odom is None:
            return

        pose = self._slam_pose() if self.use_slam else self._wheel_pose()
        if pose is None:
            pose = self._wheel_pose()
        if pose is None:
            return

        x, y, theta = pose
        state = State2D()
        state.pose.x = x
        state.pose.y = y
        state.pose.theta = theta
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
