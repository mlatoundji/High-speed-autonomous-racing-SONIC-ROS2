#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from autocar_msgs.msg import Path2D, State2D
from autocar_nav_pure_pursuit import generate_cubic_path, yaw_to_quaternion
from autocar_nav_pure_pursuit.pure_pursuit import (
    anchor_path_index,
    apply_speed_ramp,
    closest_path_index,
    curvature_speed_limit,
    front_axle_pose,
    peak_curvature,
)


class LocalPathPlanner(Node):

    def __init__(self):
        super().__init__('local_planner')

        self.path_pub = self.create_publisher(Path2D, '/autocar/path', 10)
        self.path_viz_pub = self.create_publisher(Path, '/autocar/viz_path', 10)
        self.target_vel_pub = self.create_publisher(Float64, '/autocar/target_velocity', 10)

        self.goals_sub = self.create_subscription(
            Path2D, '/autocar/goals', self.goals_cb, 10)
        self.state_sub = self.create_subscription(
            State2D, '/autocar/state2D', self.state_cb, 10)
        self.mode_sub = self.create_subscription(
            Int32, '/autocar/nav_mode', self.mode_cb, 10)

        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameters('', [
            ('update_frequency', 10.0),
            ('frame_id', 'base_link'),
            ('car_width', 2.0),
            ('centreofgravity_to_frontaxle', 1.483),
            ('cruise_velocity', 6.0),
            ('avoid_velocity', 6.0),
            ('exploration_velocity', 4.0),
            ('max_lateral_accel', 4.5),
            ('min_curvature', 0.012),
            ('curvature_lookahead', 140),
            ('curvature_smooth_window', 5),
            ('accel_rate', 4.0),
            ('decel_rate', 6.0),
        ])

        self.frequency = float(self.get_parameter('update_frequency').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.cg2front = float(self.get_parameter('centreofgravity_to_frontaxle').value)
        self.cruise_vel = float(self.get_parameter('cruise_velocity').value)
        self.explore_vel = float(self.get_parameter('exploration_velocity').value)
        self.max_lat_accel = float(self.get_parameter('max_lateral_accel').value)
        self.min_kappa = float(self.get_parameter('min_curvature').value)
        self.curv_lookahead = int(self.get_parameter('curvature_lookahead').value)
        self.curv_smooth = int(self.get_parameter('curvature_smooth_window').value)
        self.accel_rate = float(self.get_parameter('accel_rate').value)
        self.decel_rate = float(self.get_parameter('decel_rate').value)

        self.ds = 1.0 / self.frequency
        self.nav_mode = 0
        self.target_vel = self.explore_vel
        self.ramped_vel = self.explore_vel
        self.ax: list[float] = []
        self.ay: list[float] = []
        self.path_cx: list[float] = []
        self.path_cy: list[float] = []
        self.path_ck: list[float] = []
        self.closest_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(self.ds, self.timer_cb)

    def mode_cb(self, msg: Int32):
        self.nav_mode = int(msg.data)
        self.target_vel = self.cruise_vel if self.nav_mode >= 1 else self.explore_vel

    def timer_cb(self):
        self._update_target_velocity()
        out = Float64()
        out.data = self.ramped_vel
        self.target_vel_pub.publish(out)

    def _update_target_velocity(self):
        if not self.path_cx:
            return

        fx, fy = front_axle_pose(self.x, self.y, self.yaw, self.cg2front)
        self.closest_idx = closest_path_index(
            fx, fy, self.path_cx, self.path_cy,
            start_idx=self.closest_idx, search_ahead=60)

        end_idx = min(len(self.path_ck), self.closest_idx + self.curv_lookahead)
        k_peak = peak_curvature(
            self.path_ck, self.closest_idx, end_idx, self.curv_smooth)
        v_curve = curvature_speed_limit(k_peak, self.max_lat_accel, self.min_kappa)
        v_target = min(self.target_vel, v_curve)

        self.ramped_vel = apply_speed_ramp(
            self.ramped_vel, v_target, self.ds,
            self.accel_rate, self.decel_rate)

    def state_cb(self, msg: State2D):
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def goals_cb(self, msg: Path2D):
        self.ax = [p.x for p in msg.poses]
        self.ay = [p.y for p in msg.poses]
        self._publish_path()

    def _publish_path(self):
        if len(self.ax) < 2:
            return

        cx, cy, cyaw, ck = generate_cubic_path(self.ax, self.ay, self.ds)
        n = min(len(cx), len(cy), len(cyaw), len(ck))
        self.path_cx = cx[:n]
        self.path_cy = cy[:n]
        self.path_ck = ck[:n]

        fx, fy = front_axle_pose(self.x, self.y, self.yaw, self.cg2front)
        self.closest_idx = anchor_path_index(
            fx, fy, self.path_cx, self.path_cy, self.closest_idx, 120)

        target = Path2D()
        viz = Path()
        viz.header.frame_id = 'odom'
        viz.header.stamp = self.get_clock().now().to_msg()

        for i in range(n):
            pose = Pose2D(x=cx[i], y=cy[i], theta=cyaw[i])
            target.poses.append(pose)

            vp = PoseStamped()
            vp.header.frame_id = 'odom'
            vp.header.stamp = self.get_clock().now().to_msg()
            vp.pose.position.x = cx[i]
            vp.pose.position.y = cy[i]
            vp.pose.orientation = yaw_to_quaternion(np.pi * 0.5 - cyaw[i])
            viz.poses.append(vp)

        self.path_pub.publish(target)
        self.path_viz_pub.publish(viz)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LocalPathPlanner()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
