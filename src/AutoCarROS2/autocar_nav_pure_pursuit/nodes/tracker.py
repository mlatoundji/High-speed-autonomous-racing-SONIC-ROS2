#!/usr/bin/env python3
"""Pure Pursuit path tracker.

Adopts the proven control loop from ``need_for_speed`` (measured-speed lookahead,
full target velocity tracking) while keeping forward-only path indexing and
arc-length lookahead interpolation from this package.
"""

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64

from autocar_msgs.msg import Path2D, State2D
from autocar_nav_pure_pursuit.pure_pursuit import (
    closest_path_index,
    dynamic_lookahead,
    find_lookahead_point,
    front_axle_pose,
    initial_path_index,
    lateral_error_front_axle,
    limit_steering_rate,
    pure_pursuit_steering,
    rear_axle_pose,
    smooth_steering,
)
from autocar_nav_pure_pursuit.yaw_to_quaternion import yaw_to_quaternion

STARTUP_RAMP_S = 2.0


class PurePursuitTracker(Node):

    def __init__(self):

        super().__init__('path_tracker')

        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)
        self.lateral_error_pub = self.create_publisher(Float64, '/autocar/lateral_error', 10)

        self.localisation_sub = self.create_subscription(
            State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(
            Path2D, '/autocar/path', self.path_cb, 10)
        self.target_vel_sub = self.create_subscription(
            Float64, '/autocar/target_velocity', self.target_vel_cb, 10)

        try:
            desc = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None, desc),
                    ('centreofgravity_to_frontaxle', None, desc),
                    ('centreofgravity_to_rearaxle', None, desc),
                    ('wheelbase', None, desc),
                    ('lookahead_gain', None, desc),
                    ('lookahead_min', None, desc),
                    ('lookahead_max', None, desc),
                    ('closest_search_ahead', None, desc),
                    ('steering_limits', None, desc),
                    ('steering_rate_limit', None, desc),
                    ('steer_smoothing', None, desc),
                    ('velocity_gain', None, desc),
                    ('startup_ramp_s', None, desc),
                ],
            )

            self.frequency = float(self.get_parameter('update_frequency').value)
            self.cg2front = float(self.get_parameter('centreofgravity_to_frontaxle').value)
            self.cg2rear = float(self.get_parameter('centreofgravity_to_rearaxle').value)
            self.wheelbase = float(self.get_parameter('wheelbase').value)
            self.ld_gain = float(self.get_parameter('lookahead_gain').value)
            self.ld_min = float(self.get_parameter('lookahead_min').value)
            self.ld_max = float(self.get_parameter('lookahead_max').value)
            self.search_ahead = int(self.get_parameter('closest_search_ahead').value)
            self.max_steer = float(self.get_parameter('steering_limits').value)
            self.steer_rate_limit = float(self.get_parameter('steering_rate_limit').value)
            self.steer_smoothing = float(self.get_parameter('steer_smoothing').value)
            self.velocity_gain = float(self.get_parameter('velocity_gain').value)
            ramp = self.get_parameter('startup_ramp_s').value
            self.startup_ramp_s = float(ramp) if ramp is not None else STARTUP_RAMP_S

        except ValueError:
            raise Exception('Missing ROS parameters. Check the configuration file.')

        self.x = None
        self.y = None
        self.yaw = None
        self.vel = 0.0
        self.target_vel = 0.0

        self.path_x = np.array([])
        self.path_y = np.array([])
        self.path_yaw = np.array([])

        self.closest_idx = 0
        self.prev_steer = None
        self._control_ticks = 0

        self.lock = threading.RLock()
        self.dt = 1.0 / self.frequency

        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f'Pure Pursuit: Ld = clip({self.ld_gain:.2f}*v + {self.ld_min:.1f}, '
            f'{self.ld_min:.1f}, {self.ld_max:.1f}) m, steer cap {self.max_steer:.2f} rad.'
        )

    def timer_cb(self):
        self.pure_pursuit_control()

    def vehicle_state_cb(self, msg):
        with self.lock:
            self.x = msg.pose.x
            self.y = msg.pose.y
            self.yaw = msg.pose.theta
            self.vel = float(np.hypot(msg.twist.x, msg.twist.y))

    def path_cb(self, msg):
        with self.lock:
            new_x = np.array([p.x for p in msg.poses], dtype=float)
            new_y = np.array([p.y for p in msg.poses], dtype=float)
            new_yaw = np.array([p.theta for p in msg.poses], dtype=float)

            if (
                new_x.size == self.path_x.size
                and new_x.size > 0
                and new_x[0] == self.path_x[0]
                and new_y[0] == self.path_y[0]
                and new_x[-1] == self.path_x[-1]
                and new_y[-1] == self.path_y[-1]
            ):
                return

            old_x, old_y = self.path_x, self.path_y
            old_idx = self.closest_idx

            self.path_x = new_x
            self.path_y = new_y
            self.path_yaw = new_yaw

            if self.x is None:
                self.closest_idx = 0
            else:
                rx, ry = rear_axle_pose(self.x, self.y, self.yaw, self.cg2rear)
                self.closest_idx = initial_path_index(
                    rx, ry, self.path_x, self.path_y, self.search_ahead)

            if old_x.size and old_idx < old_x.size:
                jump = float(np.hypot(
                    self.path_x[self.closest_idx] - old_x[old_idx],
                    self.path_y[self.closest_idx] - old_y[old_idx],
                ))
                if jump >= 4.0:
                    self.prev_steer = None
            else:
                self.prev_steer = None

    def target_vel_cb(self, msg):
        if msg.data > 0.0:
            self.target_vel = float(msg.data)

    def _startup_blend(self):
        if self.startup_ramp_s <= 0.0:
            return 1.0
        return min(1.0, self._control_ticks / (self.startup_ramp_s * self.frequency))

    def pure_pursuit_control(self):
        self._control_ticks += 1
        boot = self._startup_blend()

        with self.lock:
            if self.x is None or self.path_x.size == 0:
                return

            x = self.x
            y = self.y
            yaw = self.yaw
            vel = self.vel
            target_vel = self.target_vel
            path_x = self.path_x
            path_y = self.path_y
            path_yaw = self.path_yaw
            closest_idx = self.closest_idx
            prev_steer = self.prev_steer

        rx, ry = rear_axle_pose(x, y, yaw, self.cg2rear)
        fx, fy = front_axle_pose(x, y, yaw, self.cg2front)

        cx = path_x.tolist()
        cy = path_y.tolist()

        closest_idx = closest_path_index(
            rx, ry, cx, cy,
            start_idx=closest_idx,
            search_ahead=self.search_ahead,
        )

        # Measured speed sets lookahead (natural damping at the current pace).
        ld = dynamic_lookahead(vel, self.ld_gain, self.ld_min, self.ld_max)

        la_idx, tx, ty = find_lookahead_point(
            rx, ry, cx, cy, closest_idx, ld)
        if la_idx is None:
            return

        steer = pure_pursuit_steering(rx, ry, yaw, tx, ty, self.wheelbase)
        steer = smooth_steering(steer, prev_steer, self.steer_smoothing)
        steer = limit_steering_rate(
            steer, prev_steer, self.dt, self.steer_rate_limit)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        # Trust the local planner speed profile; only ramp up briefly at launch.
        cmd_vel = float(target_vel * self.velocity_gain * boot)

        lat_err = lateral_error_front_axle(fx, fy, yaw, cx, cy, closest_idx)

        ref_yaw = float(path_yaw[la_idx]) if la_idx < len(path_yaw) else yaw
        self._publish_lateral_ref(tx, ty, ref_yaw)

        err_msg = Float64()
        err_msg.data = lat_err
        self.lateral_error_pub.publish(err_msg)

        self._publish_command(cmd_vel, steer)

        with self.lock:
            self.closest_idx = closest_idx
            self.prev_steer = steer

    def _publish_lateral_ref(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        self.lateral_ref_pub.publish(pose)

    def _publish_command(self, velocity, steering_angle):
        drive = Twist()
        drive.linear.x = float(velocity)
        drive.angular.z = float(steering_angle)
        self.tracker_pub.publish(drive)


def main(args=None):
    rclpy.init(args=args)
    try:
        tracker = PurePursuitTracker()
        rclpy.spin(tracker)
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
