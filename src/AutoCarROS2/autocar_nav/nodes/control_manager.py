#!/usr/bin/env python3
"""Control arbitration for manual, semi-automatic and autonomous driving."""

import json
from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from autocar_msgs.msg import State2D

VALID_MODES = {'manual', 'semi', 'auto'}


def zero_twist():
    return Twist()


def clamp(value, low, high):
    return max(low, min(high, value))


class ControlManager(Node):
    """Single authority that publishes to Gazebo's `/autocar/cmd_vel`."""

    def __init__(self):
        super().__init__('control_manager')

        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_frequency', 50.0, desc),
                ('initial_mode', 'auto', desc),
                ('manual_timeout_s', 0.35, desc),
                ('semi_override_timeout_s', 1.0, desc),
                ('max_accel_mps2', 4.0, desc),
                ('max_steer_rate_radps', 1.8, desc),
                ('max_steer', 0.85, desc),
                ('stuck_speed_threshold', 0.15, desc),
                ('stuck_command_threshold', 0.8, desc),
                ('stuck_time_s', 1.2, desc),
                ('collision_hold_s', 0.8, desc),
                ('footprint_length', 4.8, desc),
                ('footprint_width', 2.1, desc),
                ('track_inner_radius', 96.0, desc),
                ('track_outer_radius', 112.0, desc),
                ('track_boundary_margin', 0.8, desc),
            ],
        )

        self.frequency = float(self.get_parameter('update_frequency').value)
        self.dt = 1.0 / self.frequency
        self.mode = self._normalise_mode(str(self.get_parameter('initial_mode').value))
        self.manual_timeout_s = float(self.get_parameter('manual_timeout_s').value)
        self.semi_override_timeout_s = float(
            self.get_parameter('semi_override_timeout_s').value)
        self.max_accel = float(self.get_parameter('max_accel_mps2').value)
        self.max_steer_rate = float(self.get_parameter('max_steer_rate_radps').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.stuck_speed_threshold = float(
            self.get_parameter('stuck_speed_threshold').value)
        self.stuck_command_threshold = float(
            self.get_parameter('stuck_command_threshold').value)
        self.stuck_time_s = float(self.get_parameter('stuck_time_s').value)
        self.collision_hold_s = float(self.get_parameter('collision_hold_s').value)
        self.footprint_length = float(self.get_parameter('footprint_length').value)
        self.footprint_width = float(self.get_parameter('footprint_width').value)
        self.track_inner_radius = float(
            self.get_parameter('track_inner_radius').value)
        self.track_outer_radius = float(
            self.get_parameter('track_outer_radius').value)
        self.track_boundary_margin = float(
            self.get_parameter('track_boundary_margin').value)

        self.cmd_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/autocar/control_status', 10)
        self.mode_pub = self.create_publisher(String, '/autocar/current_mode', 10)
        self.collision_pub = self.create_publisher(Bool, '/autocar/collision', 10)

        self.create_subscription(Twist, '/autocar/auto_cmd_vel', self.auto_cmd_cb, 10)
        self.create_subscription(Twist, '/autocar/manual_cmd_vel', self.manual_cmd_cb, 10)
        self.create_subscription(String, '/autocar/control_mode', self.mode_cb, 10)
        self.create_subscription(Bool, '/autocar/stop', self.stop_cb, 10)
        self.create_subscription(Bool, '/autocar/resume_auto', self.resume_cb, 10)
        self.create_subscription(State2D, '/autocar/state2D', self.state_cb, 10)
        self.create_subscription(Float64, '/autocar/lateral_error', self.lateral_error_cb, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        self.auto_cmd = zero_twist()
        self.manual_cmd = zero_twist()
        self.output_cmd = zero_twist()
        self.last_auto_time = None
        self.last_manual_time = None
        self.last_output_time = self._now()

        self.state = None
        self.lateral_error = 0.0
        self.grid = None
        self.grid_info = None

        self.stop_latched = False
        self.collision_latched = False
        self.collision_reason = ''
        self.collision_since = None
        self.stuck_since = None

        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info(f'Control manager ready in {self.mode!r} mode.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _normalise_mode(self, mode):
        mode = mode.strip().lower()
        if mode not in VALID_MODES:
            self.get_logger().warn(
                f'Unknown control mode {mode!r}; falling back to manual.')
            return 'manual'
        return mode

    def auto_cmd_cb(self, msg):
        self.auto_cmd = deepcopy(msg)
        self.last_auto_time = self._now()

    def manual_cmd_cb(self, msg):
        self.manual_cmd = deepcopy(msg)
        self.last_manual_time = self._now()
        if self.stop_latched and abs(msg.linear.x) > 0.01:
            self.stop_latched = False

    def mode_cb(self, msg):
        self.mode = self._normalise_mode(msg.data)
        self.stop_latched = False
        if self.mode == 'manual':
            self.collision_latched = False
        self.get_logger().info(f'Control mode set to {self.mode}.')

    def stop_cb(self, msg):
        if msg.data:
            self.stop_latched = True
            self.get_logger().warn('Manual stop latched.')

    def resume_cb(self, msg):
        if msg.data:
            self.stop_latched = False
            self.collision_latched = False
            self.collision_reason = ''
            self.mode = 'auto'
            self.get_logger().info('Autonomous control resumed.')

    def state_cb(self, msg):
        self.state = msg

    def lateral_error_cb(self, msg):
        self.lateral_error = float(msg.data)

    def map_cb(self, msg):
        self.grid_info = msg.info
        self.grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)

    def timer_cb(self):
        now = self._now()
        desired = self._desired_command(now)
        desired = self._apply_recovery(desired, now)
        desired = self._rate_limit(desired, now)

        self.output_cmd = desired
        self.last_output_time = now
        self.cmd_pub.publish(desired)
        self._publish_status(now)

    def _desired_command(self, now):
        if self.stop_latched:
            return zero_twist()

        if self.mode == 'manual':
            if self._manual_recent(now, self.manual_timeout_s):
                return deepcopy(self.manual_cmd)
            return zero_twist()

        if self.mode == 'semi' and self._manual_recent(now, self.semi_override_timeout_s):
            return self._semi_command()

        return deepcopy(self.auto_cmd)

    def _manual_recent(self, now, timeout):
        return self.last_manual_time is not None and now - self.last_manual_time <= timeout

    def _semi_command(self):
        cmd = deepcopy(self.auto_cmd)
        manual = self.manual_cmd
        cmd.linear.x = manual.linear.x
        if abs(manual.angular.z) > 1e-3:
            cmd.angular.z = self.auto_cmd.angular.z + manual.angular.z
        return cmd

    def _apply_recovery(self, cmd, now):
        collision = self._detect_collision(cmd, now)
        if collision and not self.collision_latched:
            self.collision_latched = True
            self.collision_since = now
            self.get_logger().warn(f'Collision/recovery detected: {self.collision_reason}')

        if self.mode == 'manual':
            return cmd

        if self.collision_latched:
            if self.collision_since is None:
                self.collision_since = now
            if now - self.collision_since < self.collision_hold_s:
                return zero_twist()
            self.collision_latched = False
            self.collision_reason = ''
            self.stuck_since = None

        return cmd

    def _detect_collision(self, cmd, now):
        if self._outside_track_bounds():
            self.collision_reason = 'vehicle outside track bounds'
            return True

        footprint_blocked = self._footprint_occupied()
        if footprint_blocked:
            self.collision_reason = 'occupied footprint'
            return True

        if self.state is None:
            self.stuck_since = None
            return False

        speed = float(np.hypot(self.state.twist.x, self.state.twist.y))
        commanded = abs(cmd.linear.x)
        if commanded >= self.stuck_command_threshold and speed <= self.stuck_speed_threshold:
            if self.stuck_since is None:
                self.stuck_since = now
            if now - self.stuck_since >= self.stuck_time_s:
                self.collision_reason = 'commanded motion but vehicle is stuck'
                return True
        else:
            self.stuck_since = None
        return False

    def _outside_track_bounds(self):
        if self.state is None:
            return False

        radius = float(np.hypot(self.state.pose.x, self.state.pose.y))
        min_radius = self.track_inner_radius - self.track_boundary_margin
        max_radius = self.track_outer_radius + self.track_boundary_margin
        return radius < min_radius or radius > max_radius

    def _footprint_occupied(self):
        if self.grid is None or self.grid_info is None or self.state is None:
            return False

        x = self.state.pose.x
        y = self.state.pose.y
        theta = self.state.pose.theta
        samples_x = np.linspace(-0.4 * self.footprint_length, 0.4 * self.footprint_length, 5)
        samples_y = np.linspace(-0.5 * self.footprint_width, 0.5 * self.footprint_width, 3)
        c = np.cos(theta)
        s = np.sin(theta)
        for sx in samples_x:
            for sy in samples_y:
                wx = x + sx * c - sy * s
                wy = y + sx * s + sy * c
                cell = self._world_to_grid(wx, wy)
                if cell is None:
                    continue
                col, row = cell
                if self.grid[row, col] >= 80:
                    return True
        return False

    def _world_to_grid(self, x, y):
        info = self.grid_info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if 0 <= col < info.width and 0 <= row < info.height:
            return col, row
        return None

    def _rate_limit(self, cmd, now):
        dt = max(1e-3, now - self.last_output_time)
        limited = deepcopy(cmd)

        prev_v = self.output_cmd.linear.x
        dv = clamp(limited.linear.x - prev_v, -self.max_accel * dt, self.max_accel * dt)
        limited.linear.x = prev_v + dv

        prev_steer = self.output_cmd.angular.z
        dsteer = clamp(
            limited.angular.z - prev_steer,
            -self.max_steer_rate * dt,
            self.max_steer_rate * dt,
        )
        limited.angular.z = clamp(prev_steer + dsteer, -self.max_steer, self.max_steer)
        return limited

    def _publish_status(self, now):
        speed = 0.0
        if self.state is not None:
            speed = float(np.hypot(self.state.twist.x, self.state.twist.y))

        status = {
            'mode': self.mode,
            'manual_recent': self._manual_recent(now, self.semi_override_timeout_s),
            'stopped': self.stop_latched,
            'collision': self.collision_latched,
            'collision_reason': self.collision_reason,
            'speed': speed,
            'target_speed': float(self.auto_cmd.linear.x),
            'cmd_speed': float(self.output_cmd.linear.x),
            'cmd_steer': float(self.output_cmd.angular.z),
            'lateral_error': self.lateral_error,
        }

        status_msg = String()
        status_msg.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(status_msg)

        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

        collision_msg = Bool()
        collision_msg.data = self.collision_latched
        self.collision_pub.publish(collision_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        manager = ControlManager()
        rclpy.spin(manager)
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
