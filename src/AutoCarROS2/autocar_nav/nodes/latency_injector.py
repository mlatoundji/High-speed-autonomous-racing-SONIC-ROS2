#!/usr/bin/env python3
"""Latency injector for the perception->execution path.

Subscribes to `/autocar/state2D_raw` and republishes on `/autocar/state2D_mid`
after `latency_ms` milliseconds. When `latency_ms == 0`, forwards immediately.

Topic in:  /autocar/state2D_raw  (autocar_msgs/State2D)
Topic out: /autocar/state2D_mid  (autocar_msgs/State2D)

Parameter:
    latency_ms  int  (default 0)
"""

from collections import deque

import rclpy
from rclpy.node import Node

from autocar_msgs.msg import State2D

DISPATCH_HZ = 200.0


class LatencyInjector(Node):

    def __init__(self):
        super().__init__('latency_injector')

        self.declare_parameter('latency_ms', 0)
        self.latency_ms = int(self.get_parameter('latency_ms').value)
        self.latency_ns = self.latency_ms * 1_000_000

        self.create_subscription(State2D, '/autocar/state2D_raw',
                                 self._on_state, 10)
        self.pub = self.create_publisher(State2D, '/autocar/state2D_mid', 10)

        if self.latency_ms == 0:
            self.pass_through = True
            self.get_logger().info(
                'Latency injector in pass-through mode (latency_ms=0).'
            )
        else:
            self.pass_through = False
            self.buffer = deque()
            self.create_timer(1.0 / DISPATCH_HZ, self._dispatch)
            self.get_logger().info(
                f'Latency injector active: delaying state2D by {self.latency_ms} ms '
                f'(dispatch loop at {DISPATCH_HZ:.0f} Hz).'
            )

    def _on_state(self, msg: State2D):
        if self.pass_through:
            self.pub.publish(msg)
            return
        t_now = self.get_clock().now().nanoseconds
        self.buffer.append((t_now, msg))

    def _dispatch(self):
        t_now = self.get_clock().now().nanoseconds
        while self.buffer and (t_now - self.buffer[0][0]) >= self.latency_ns:
            _, msg = self.buffer.popleft()
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LatencyInjector()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
