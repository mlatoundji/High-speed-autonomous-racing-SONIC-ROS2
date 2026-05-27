#!/usr/bin/env python3
"""RViz status markers for interactive driving."""

import json

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from autocar_msgs.msg import State2D


class StatusVisualization(Node):
    def __init__(self):
        super().__init__('viz_status')
        self.marker_pub = self.create_publisher(
            MarkerArray, '/autocar/status_markers', 10)
        self.create_subscription(
            String, '/autocar/control_status', self.status_cb, 10)
        self.create_subscription(
            State2D, '/autocar/state2D', self.state_cb, 10)

        self.status = {}
        self.state = None
        self.timer = self.create_timer(0.1, self.timer_cb)

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status = {'mode': 'unknown', 'raw': msg.data}

    def state_cb(self, msg):
        self.state = msg

    def timer_cb(self):
        if self.state is None:
            return
        markers = MarkerArray()
        markers.markers.append(self._status_text_marker())
        markers.markers.append(self._speed_bar_marker())
        markers.markers.append(self._collision_marker())
        self.marker_pub.publish(markers)

    def _base_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'autocar_status'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _status_text_marker(self):
        marker = self._base_marker(0, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = self.state.pose.x
        marker.pose.position.y = self.state.pose.y
        marker.pose.position.z = 4.2
        marker.scale.z = 1.6
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = self._mode_color()

        mode = self.status.get('mode', 'unknown')
        speed = self.status.get('speed', 0.0)
        target = self.status.get('target_speed', 0.0)
        lat = self.status.get('lateral_error', 0.0)
        stopped = self.status.get('stopped', False)
        collision = self.status.get('collision', False)
        reason = self.status.get('collision_reason', '')
        state = 'STOP' if stopped else ('COLLISION' if collision else 'RUN')
        marker.text = (
            f'Mode: {mode} | {state}\n'
            f'v={speed:.2f} m/s target={target:.2f} m/s\n'
            f'lateral error={lat:+.2f} m'
        )
        if reason:
            marker.text += f'\n{reason}'
        return marker

    def _speed_bar_marker(self):
        marker = self._base_marker(1, Marker.LINE_STRIP)
        marker.scale.x = 0.18
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = self._mode_color()

        speed = max(0.0, float(self.status.get('speed', 0.0)))
        length = min(8.0, speed)
        start = Point()
        start.x = self.state.pose.x
        start.y = self.state.pose.y
        start.z = 2.5
        end = Point()
        end.x = self.state.pose.x
        end.y = self.state.pose.y + length
        end.z = 2.5
        marker.points = [start, end]
        return marker

    def _collision_marker(self):
        marker = self._base_marker(2, Marker.CUBE)
        marker.pose.position.x = self.state.pose.x
        marker.pose.position.y = self.state.pose.y
        marker.pose.position.z = 1.2
        marker.scale.x = 5.4
        marker.scale.y = 2.8
        marker.scale.z = 2.4
        marker.color.a = 0.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        if self.status.get('collision', False):
            marker.color.a = 0.35
        return marker

    def _mode_color(self):
        mode = self.status.get('mode', 'unknown')
        if self.status.get('collision', False):
            return 1.0, 0.0, 0.0
        if self.status.get('stopped', False):
            return 1.0, 0.6, 0.0
        if mode == 'manual':
            return 0.2, 0.6, 1.0
        if mode == 'semi':
            return 1.0, 0.9, 0.1
        return 0.1, 1.0, 0.2


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StatusVisualization()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
