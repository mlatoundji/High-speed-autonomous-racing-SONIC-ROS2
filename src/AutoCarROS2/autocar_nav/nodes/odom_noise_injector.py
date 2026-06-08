#!/usr/bin/env python3
"""Odometry noise injector on the perception path.

Subscribes to `/autocar/state2D_mid` (after latency injection) and republishes
on `/autocar/state2D` with additive Gaussian noise when `odom_noise_std > 0`.

Topic in:  /autocar/state2D_mid  (autocar_msgs/State2D)
Topic out: /autocar/state2D      (autocar_msgs/State2D)

Parameter:
    odom_noise_std  float  (default 0.0)
        Standard deviation (m for x/y, rad for theta). 0 = pass-through.
"""

import random

import rclpy
from rclpy.node import Node

from autocar_msgs.msg import State2D


class OdomNoiseInjector(Node):

    def __init__(self):
        super().__init__('odom_noise_injector')

        self.declare_parameter('odom_noise_std', 0.0)
        self.odom_noise_std = float(self.get_parameter('odom_noise_std').value)

        self.create_subscription(State2D, '/autocar/state2D_mid',
                                 self._on_state, 10)
        self.pub = self.create_publisher(State2D, '/autocar/state2D', 10)

        if self.odom_noise_std <= 0.0:
            self.pass_through = True
            self.get_logger().info(
                'Odom noise injector in pass-through mode (odom_noise_std=0).'
            )
        else:
            self.pass_through = False
            self.get_logger().info(
                f'Odom noise injector active: sigma={self.odom_noise_std:.4f}.'
            )

    def _on_state(self, msg: State2D):
        if self.pass_through:
            self.pub.publish(msg)
            return

        out = State2D()
        out.pose.x = msg.pose.x + random.gauss(0.0, self.odom_noise_std)
        out.pose.y = msg.pose.y + random.gauss(0.0, self.odom_noise_std)
        out.pose.theta = msg.pose.theta + random.gauss(0.0, self.odom_noise_std)
        out.twist = msg.twist
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OdomNoiseInjector()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
