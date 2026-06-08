#!/usr/bin/env python3
"""Hybrid global planner: lap-1 LiDAR centerline, lap-2+ SLAM pose + racing line."""

import os

import numpy as np
import pandas as pd
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener

from autocar_msgs.msg import Path2D, State2D
from autocar_nav_pure_pursuit.pure_pursuit import (
    closest_waypoint_index_closed,
    forward_vector,
)
from autocar_nav_pure_pursuit_lidar.centerline_extractor import extract_local_centerline
from autocar_nav_pure_pursuit_lidar.slam_pose import slam_pose_in_map


class GlobalPlannerLidar(Node):

    def __init__(self):
        super().__init__('global_planner_lidar')

        self.goals_pub = self.create_publisher(Path2D, '/autocar/goals', 10)
        self.goals_viz_pub = self.create_publisher(PoseArray, '/autocar/viz_goals', 10)
        self.mode_pub = self.create_publisher(Int32, '/autocar/nav_mode', 10)

        self.state_sub = self.create_subscription(
            State2D, '/autocar/state2D', self.state_cb, 10)
        self.lap_sub = self.create_subscription(
            Int32, '/autocar/lap_count', self.lap_cb, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)

        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameters('', [
            ('waypoints_ahead', 5),
            ('waypoints_behind', 2),
            ('passed_threshold', 0.25),
            ('centreofgravity_to_frontaxle', 1.483),
            ('waypoint_search_ahead', 30),
            ('waypoints_file', 'waypoints_racing.csv'),
            ('exploration_goal_count', 10),
            ('exploration_goal_step', 4.0),
            ('cg_to_lidar', 2.4),
        ])

        self.wp_ahead = int(self.get_parameter('waypoints_ahead').value)
        self.wp_behind = int(self.get_parameter('waypoints_behind').value)
        self.passed_threshold = float(self.get_parameter('passed_threshold').value)
        self.cg2front = float(self.get_parameter('centreofgravity_to_frontaxle').value)
        self.search_ahead = int(self.get_parameter('waypoint_search_ahead').value)
        self.goal_count = int(self.get_parameter('exploration_goal_count').value)
        self.goal_step = float(self.get_parameter('exploration_goal_step').value)
        self.cg2lidar = float(self.get_parameter('cg_to_lidar').value)

        self._load_racing_line()

        self.lap_count = 0
        self.x = None
        self.y = None
        self.theta = None
        self.closest_id = 0
        self._publish_key = None

        self.live_grid = None
        self.live_grid_info = None
        self.map_frame_id = 'map'
        self.scan = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_cb)

    def _load_racing_line(self):
        waypoints_file = str(self.get_parameter('waypoints_file').value)
        csv_path = os.path.join(
            get_package_share_directory('autocar_racing_line'),
            'data', waypoints_file)
        self.get_logger().info(f'Racing line (lap 2+): {csv_path}')
        df = pd.read_csv(csv_path)
        self.rx = np.asarray(df['X-axis'].values, dtype=float)
        self.ry = np.asarray(df['Y-axis'].values, dtype=float)
        self.racing_n = min(len(self.rx), len(self.ry))

    def lap_cb(self, msg: Int32):
        if msg.data > self.lap_count:
            self.lap_count = msg.data
            if self.lap_count >= 1:
                self.get_logger().info(
                    'Switching to RACING mode (SLAM pose + racing line)')

    def map_cb(self, msg: OccupancyGrid):
        self.map_frame_id = msg.header.frame_id or 'map'
        self.live_grid_info = msg.info
        self.live_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def state_cb(self, msg: State2D):
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta

    def _map_frame_pose(self) -> tuple[float, float, float] | None:
        try:
            stamp = rclpy.time.Time()
            timeout = Duration(seconds=0.05)
            map_to_base = self._tf_buffer.lookup_transform(
                'map', 'base_link', stamp, timeout=timeout)
        except Exception:
            return None
        return slam_pose_in_map(map_to_base)

    def timer_cb(self):
        if self.x is None:
            return

        mode_msg = Int32()
        if self.lap_count < 1:
            mode_msg.data = 0  # exploration
            self.mode_pub.publish(mode_msg)
            self._publish_exploration_goals()
        else:
            mode_msg.data = 1  # racing
            self.mode_pub.publish(mode_msg)
            self._publish_racing_goals()

    def _publish_exploration_goals(self):
        ranges = None
        angle_min = angle_inc = range_min = range_max = 0.0
        if self.scan is not None:
            ranges = np.asarray(self.scan.ranges, dtype=float)
            angle_min = self.scan.angle_min
            angle_inc = self.scan.angle_increment
            range_min = self.scan.range_min
            range_max = self.scan.range_max

        map_pose = None
        grid = self.live_grid
        grid_info = self.live_grid_info
        if grid is not None and self.map_frame_id == 'map':
            map_pose = self._map_frame_pose()

        pts = extract_local_centerline(
            self.x, self.y, self.theta, self.cg2lidar,
            self.goal_step, self.goal_count,
            scan_ranges=ranges,
            scan_angle_min=angle_min,
            scan_angle_increment=angle_inc,
            scan_range_min=range_min,
            scan_range_max=range_max,
            grid=grid if map_pose is not None else None,
            grid_info=grid_info if map_pose is not None else None,
            map_x=map_pose[0] if map_pose else None,
            map_y=map_pose[1] if map_pose else None,
            map_yaw=map_pose[2] if map_pose else None,
        )

        fwd_x, fwd_y = forward_vector(self.theta)
        behind = [
            (self.x - self.goal_step * fwd_x, self.y - self.goal_step * fwd_y),
            (self.x, self.y),
        ]
        if map_pose is not None and grid is not None:
            mx, my, _ = map_pose
            dx = self.x - mx
            dy = self.y - my
            pts = [(px + dx, py + dy) for px, py in pts]

        px = [p[0] for p in behind + pts]
        py = [p[1] for p in behind + pts]
        self._emit_goals(px, py, 'explore', force=True)

    def _publish_racing_goals(self):
        fx = self.x + self.cg2front * -np.sin(self.theta)
        fy = self.y + self.cg2front * np.cos(self.theta)

        self.closest_id = closest_waypoint_index_closed(
            fx, fy, self.rx, self.ry,
            start_idx=self.closest_id,
            search_ahead=self.search_ahead,
        )
        cid = self.closest_id

        transform = self._body_offset(
            self.rx[cid], self.ry[cid], fx, fy, self.theta)

        if cid < 2:
            mode = 'start'
            px = self.rx[0:self.wp_ahead + self.wp_behind]
            py = self.ry[0:self.wp_ahead + self.wp_behind]
        elif cid > (self.racing_n - self.wp_ahead - self.wp_behind):
            mode = 'end'
            px = self.rx[-(self.wp_ahead + self.wp_behind):]
            py = self.ry[-(self.wp_ahead + self.wp_behind):]
        elif transform[1] < -self.passed_threshold:
            mode = 'passed'
            lo = cid - (self.wp_behind - 1)
            hi = cid + (self.wp_ahead + 1)
            px = self.rx[lo:hi]
            py = self.ry[lo:hi]
        else:
            mode = 'approach'
            lo = cid - self.wp_behind
            hi = cid + self.wp_ahead
            px = self.rx[lo:hi]
            py = self.ry[lo:hi]

        self._emit_goals(px.tolist(), py.tolist(), mode, cid)

    def _body_offset(self, px, py, ax, ay, theta):
        c, s = np.cos(-theta), np.sin(-theta)
        rel = np.dot(np.array([[c, -s], [s, c]]), np.array([px - ax, py - ay]))
        return rel

    def _emit_goals(self, px, py, mode, cid=None, force=False):
        if len(px) < 2:
            return

        key = (mode, round(float(px[0]), 2), round(float(py[0]), 2), cid)
        if not force and key == self._publish_key:
            return
        self._publish_key = key

        goals = Path2D()
        viz = PoseArray()
        viz.header.frame_id = 'odom'
        viz.header.stamp = self.get_clock().now().to_msg()

        for x, y in zip(px, py):
            g = Pose2D(x=float(x), y=float(y))
            goals.poses.append(g)
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            viz.poses.append(p)

        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz)
        label = f'#{cid} {mode}' if cid is not None else mode
        self.get_logger().info(f'Goals ({label}): {len(px)} pts', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GlobalPlannerLidar()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
