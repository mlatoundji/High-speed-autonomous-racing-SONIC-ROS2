#!/usr/bin/env python3

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64

from autocar_msgs.msg import Path2D, State2D
from autocar_nav_mpc.mpc import GAZEBO_MAX_STEER, LinearMPCController
from autocar_nav_mpc.path_tracking import (
    closest_path_index,
    curvature_horizon_from_path,
    frenet_errors,
    front_axle_pose,
    limit_steering_rate,
    path_index_on_update,
    path_tangent_heading,
    smooth_steering,
    speed_scale_from_errors,
)
from autocar_nav_mpc.yaw_to_quaternion import yaw_to_quaternion

DEFAULT_CRUISE_SPEED = 8.0
STARTUP_RAMP_S = 2.0


class MPCPathTracker(Node):

    def __init__(self):

        super().__init__('path_tracker')

        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)

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
                    ('wheelbase', None, desc),
                    ('mpc_horizon', None, desc),
                    ('closest_search_ahead', None, desc),
                    ('q_ey', None, desc),
                    ('q_epsi', None, desc),
                    ('r_delta', None, desc),
                    ('r_ddelta', None, desc),
                    ('steering_limits', None, desc),
                    ('steering_rate_limit', None, desc),
                    ('velocity_gain', None, desc),
                    ('cruise_velocity', None, desc),
                    ('startup_ramp_s', None, desc),
                    ('steer_smoothing', None, desc),
                    ('lateral_soft', None, desc),
                    ('heading_soft', None, desc),
                    ('closest_min_advance', None, desc),
                ],
            )

            self.frequency = float(self.get_parameter('update_frequency').value)
            self.cg2frontaxle = float(self.get_parameter('centreofgravity_to_frontaxle').value)
            self.wheelbase = float(self.get_parameter('wheelbase').value)
            self.horizon = int(self.get_parameter('mpc_horizon').value)
            self.search_ahead = int(self.get_parameter('closest_search_ahead').value)
            self.q_ey = float(self.get_parameter('q_ey').value)
            self.q_epsi = float(self.get_parameter('q_epsi').value)
            self.r_delta = float(self.get_parameter('r_delta').value)
            self.r_ddelta = float(self.get_parameter('r_ddelta').value)
            steer_param = float(self.get_parameter('steering_limits').value)
            self.max_steer = min(steer_param, GAZEBO_MAX_STEER)
            self.steer_rate_limit = float(self.get_parameter('steering_rate_limit').value)
            self.velocity_gain = float(self.get_parameter('velocity_gain').value)
            self.default_speed = self._read_cruise_speed()
            ramp = self.get_parameter('startup_ramp_s').value
            self.startup_ramp_s = float(ramp) if ramp is not None else STARTUP_RAMP_S
            self.steer_smoothing = float(self.get_parameter('steer_smoothing').value)
            self.lateral_soft = float(self.get_parameter('lateral_soft').value)
            self.heading_soft = float(self.get_parameter('heading_soft').value)
            self.closest_min_advance = float(self.get_parameter('closest_min_advance').value)

        except ValueError:
            raise Exception('Missing ROS parameters. Check the configuration file.')

        self.x = None
        self.y = None
        self.yaw = None
        self.vel = 0.0
        self.target_vel = self.default_speed

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.closest_idx = 0
        self.prev_steer = 0.0
        self._warn_counter = 0
        self._control_ticks = 0
        self._path_version = 0

        self.dt = 1.0 / self.frequency
        self.mpc = LinearMPCController(
            horizon=self.horizon,
            dt=self.dt,
            wheelbase=self.wheelbase,
            q_ey=self.q_ey,
            q_epsi=self.q_epsi,
            r_delta=self.r_delta,
            r_ddelta=self.r_ddelta,
            max_steer=self.max_steer,
            max_steer_rate=self.steer_rate_limit,
        )

        self.lock = threading.RLock()
        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info(
            f'MPC tracker ready ({self.frequency:.0f} Hz, '
            f'cruise={self.default_speed:.1f} m/s, ramp={self.startup_ramp_s:.0f}s)'
        )

    def _read_cruise_speed(self):
        if not self.has_parameter('cruise_velocity'):
            return DEFAULT_CRUISE_SPEED
        value = self.get_parameter('cruise_velocity').value
        if value is None:
            return DEFAULT_CRUISE_SPEED
        try:
            speed = float(value)
        except (TypeError, ValueError):
            return DEFAULT_CRUISE_SPEED
        return speed if speed > 0.0 else DEFAULT_CRUISE_SPEED

    def timer_cb(self):
        self.mpc_control()

    def vehicle_state_cb(self, msg):
        with self.lock:
            self.x = msg.pose.x
            self.y = msg.pose.y
            self.yaw = msg.pose.theta
            self.vel = np.sqrt(msg.twist.x ** 2.0 + msg.twist.y ** 2.0)

    def path_cb(self, msg):
        poses = msg.poses
        with self.lock:
            new_len = len(poses)
            if new_len == len(self.cx) and new_len > 0:
                if (
                    self.cx[0] == poses[0].x and self.cy[0] == poses[0].y
                    and self.cx[-1] == poses[-1].x and self.cy[-1] == poses[-1].y
                ):
                    return

            old_cx, old_cy = self.cx, self.cy
            old_idx = self.closest_idx

            self.cx = [p.x for p in poses]
            self.cy = [p.y for p in poses]
            self.cyaw = [p.theta for p in poses]
            self._path_version += 1

            if self.x is None:
                self.closest_idx = 0
            else:
                fx, fy = front_axle_pose(self.x, self.y, self.yaw, self.cg2frontaxle)
                self.closest_idx = path_index_on_update(
                    fx, fy, self.cx, self.cy, old_cx, old_cy, old_idx, self.search_ahead)

            if old_cx and old_idx < len(old_cx):
                jump = np.hypot(
                    self.cx[self.closest_idx] - old_cx[old_idx],
                    self.cy[self.closest_idx] - old_cy[old_idx],
                )
                if jump >= 4.0:
                    self.prev_steer = 0.0
                    self.mpc.reset()
            else:
                self.mpc.reset()

            if self.cx:
                self.get_logger().info(
                    f'Path received ({len(self.cx)} points, start_idx={self.closest_idx})'
                )

    def target_vel_cb(self, msg):
        with self.lock:
            if msg.data > 0.0:
                self.target_vel = msg.data

    def _throttled_warn(self, message):
        self._warn_counter += 1
        if self._warn_counter <= 5 or self._warn_counter % 200 == 0:
            self.get_logger().warn(message)

    def _startup_blend(self):
        if self.startup_ramp_s <= 0.0:
            return 1.0
        return min(1.0, self._control_ticks / (self.startup_ramp_s * self.frequency))

    def mpc_control(self):
        self._control_ticks += 1
        boot = self._startup_blend()

        with self.lock:
            if self.x is None:
                self._throttled_warn('Waiting for /autocar/state2D (check /autocar/odom)')
                return

            x = self.x
            y = self.y
            yaw = self.yaw
            vel = self.vel
            target_vel = self.target_vel
            cx = list(self.cx)
            cy = list(self.cy)
            cyaw = list(self.cyaw)
            closest_idx = self.closest_idx
            prev_steer = self.prev_steer

        if not cx:
            speed = boot * 0.5 * self.default_speed * self.velocity_gain
            self._publish_command(speed, 0.0)
            self._throttled_warn('Waiting for /autocar/path (check goals & local_planner)')
            return

        fx, fy = front_axle_pose(x, y, yaw, self.cg2frontaxle)

        closest_idx = closest_path_index(
            fx, fy, cx, cy,
            start_idx=closest_idx,
            search_ahead=self.search_ahead,
            min_advance=self.closest_min_advance,
        )

        e_y, e_psi = frenet_errors(
            fx, fy, yaw, cx, cy, cyaw, closest_idx)

        kappa_seq = curvature_horizon_from_path(
            cx, cy, closest_idx, self.horizon)
        speed = max(vel, 0.5)

        steer = self.mpc.solve(e_y, e_psi, speed, kappa_seq)
        steer = smooth_steering(steer, prev_steer, self.steer_smoothing)
        steer = limit_steering_rate(
            steer, prev_steer, self.dt, self.steer_rate_limit)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        err_scale = speed_scale_from_errors(
            e_y, e_psi, self.lateral_soft, self.heading_soft)
        cmd_vel = float(target_vel * self.velocity_gain * boot * err_scale)

        ref_yaw = path_tangent_heading(cyaw[closest_idx])
        self._publish_lateral_ref(cx[closest_idx], cy[closest_idx], ref_yaw)
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
        tracker = MPCPathTracker()
        rclpy.spin(tracker)
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
