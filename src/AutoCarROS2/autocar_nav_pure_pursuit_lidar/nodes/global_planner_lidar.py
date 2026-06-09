#!/usr/bin/env python3
"""Hybrid global planner: lap-1 LiDAR centerline, lap-2+ smoothed racing line from SLAM map."""

import numpy as np
import rclpy
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
from autocar_nav_pure_pursuit_lidar.centerline_extractor import extract_local_centerline
from autocar_nav_pure_pursuit_lidar.map_centerline import extract_loop_centerline_from_map
from autocar_nav_pure_pursuit_lidar.map_track_geometry import (
    map_corridor_bounds_for_polyline,
    mean_corridor_half_width,
)
from autocar_nav_pure_pursuit_lidar.pure_pursuit import (
    closest_waypoint_index_closed,
    forward_vector,
)
from autocar_nav_pure_pursuit_lidar.racing_line_mincurv import compute_mincurv_racing_line
from autocar_nav_pure_pursuit_lidar.racing_line_smooth import compute_smooth_racing_line
from autocar_nav_pure_pursuit_lidar.slam_pose import map_point_to_odom, slam_pose_in_map


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

        # slam_toolbox publishes /map with transient_local + reliable.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
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
            ('exploration_goal_count', 10),
            ('exploration_goal_step', 3.0),
            ('cg_to_lidar', 2.4),
            ('centerline_step', 3.5),
            ('centerline_close_dist', 4.0),
            ('centerline_min_points', 20),
            ('centerline_post_smooth_passes', 3),
            ('centerline_refine_passes', 3),
            ('racing_use_map_corridor', True),
            ('racing_boundary_margin', 1.0),
            ('racing_mincurv_max_offset', 5.0),
            ('racing_mincurv_iters', 8000),
            ('racing_smooth_alpha', 0.4),
            ('racing_smooth_iters', 10),
            ('racing_smooth_max_dev', 1.5),
            ('racing_smooth_pre_iters', 4),
            ('racing_smooth_pre_alpha', 0.4),
            ('racing_smooth_coarse_points', 80),
            ('racing_smooth_energy_ratio', 0.05),
            ('racing_track_half_width', 8.0),
        ])

        self.wp_ahead = int(self.get_parameter('waypoints_ahead').value)
        self.wp_behind = int(self.get_parameter('waypoints_behind').value)
        self.passed_threshold = float(self.get_parameter('passed_threshold').value)
        self.cg2front = float(self.get_parameter('centreofgravity_to_frontaxle').value)
        self.search_ahead = int(self.get_parameter('waypoint_search_ahead').value)
        self.goal_count = int(self.get_parameter('exploration_goal_count').value)
        self.goal_step = float(self.get_parameter('exploration_goal_step').value)
        self.cg2lidar = float(self.get_parameter('cg_to_lidar').value)
        self.centerline_step = float(self.get_parameter('centerline_step').value)
        self.centerline_close_dist = float(self.get_parameter('centerline_close_dist').value)
        self.centerline_min_points = int(self.get_parameter('centerline_min_points').value)
        self.centerline_post_smooth_passes = int(
            self.get_parameter('centerline_post_smooth_passes').value)
        self.centerline_refine_passes = int(self.get_parameter('centerline_refine_passes').value)
        self.racing_use_map_corridor = bool(self.get_parameter('racing_use_map_corridor').value)
        self.racing_boundary_margin = float(self.get_parameter('racing_boundary_margin').value)
        self.racing_mincurv_max_offset = float(
            self.get_parameter('racing_mincurv_max_offset').value)
        self.racing_mincurv_iters = int(self.get_parameter('racing_mincurv_iters').value)
        self.racing_smooth_alpha = float(self.get_parameter('racing_smooth_alpha').value)
        self.racing_smooth_iters = int(self.get_parameter('racing_smooth_iters').value)
        self.racing_smooth_max_dev = float(self.get_parameter('racing_smooth_max_dev').value)
        self.racing_smooth_pre_iters = int(self.get_parameter('racing_smooth_pre_iters').value)
        self.racing_smooth_pre_alpha = float(self.get_parameter('racing_smooth_pre_alpha').value)
        self.racing_smooth_coarse_points = int(
            self.get_parameter('racing_smooth_coarse_points').value)
        self.racing_smooth_energy_ratio = float(
            self.get_parameter('racing_smooth_energy_ratio').value)
        self.racing_track_half_width = float(self.get_parameter('racing_track_half_width').value)

        self.lap_count = 0
        self.x = None
        self.y = None
        self.theta = None
        self.closest_id = 0
        self._publish_key = None

        self.rx_map = np.array([])
        self.ry_map = np.array([])
        self.racing_n = 0
        self._racing_line_ready = False
        self._map_to_odom_snapshot = None

        self.live_grid = None
        self.live_grid_info = None
        self.map_frame_id = 'map'
        self._map_logged = False
        self.scan = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_cb)

    def lap_cb(self, msg: Int32):
        if msg.data > self.lap_count:
            self.lap_count = msg.data
            if self.lap_count == 1:
                # First lap complete: exploration done, build racing line once.
                self._racing_line_ready = False
                self._map_to_odom_snapshot = None
                self.get_logger().info(
                    'Switching to RACING mode — smoothed line from SLAM map')

    def map_cb(self, msg: OccupancyGrid):
        self.map_frame_id = msg.header.frame_id or 'map'
        self.live_grid_info = msg.info
        self.live_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        if not self._map_logged:
            self._map_logged = True
            self.get_logger().info(
                f'SLAM /map received: {msg.info.width}x{msg.info.height} '
                f'cells, resolution={msg.info.resolution:.2f} m')

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def state_cb(self, msg: State2D):
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta

    def _lookup_map_to_odom(self):
        stamp = rclpy.time.Time()
        timeout = Duration(seconds=0.05)
        return self._tf_buffer.lookup_transform(
            'map', 'odom', stamp, timeout=timeout)

    def _map_frame_pose(self) -> tuple[float, float, float] | None:
        try:
            stamp = rclpy.time.Time()
            timeout = Duration(seconds=0.05)
            map_to_base = self._tf_buffer.lookup_transform(
                'map', 'base_link', stamp, timeout=timeout)
        except Exception:
            return None
        return slam_pose_in_map(map_to_base)

    def _build_racing_line_from_map(self) -> bool:
        if self.live_grid is None or self.live_grid_info is None:
            return False
        if self.map_frame_id != 'map':
            return False

        map_pose = self._map_frame_pose()
        if map_pose is None:
            return False

        mx, my, myaw = map_pose
        cx, cy = extract_loop_centerline_from_map(
            self.live_grid,
            self.live_grid_info,
            mx, my, myaw,
            step=self.centerline_step,
            close_dist=self.centerline_close_dist,
            min_points=self.centerline_min_points,
            post_smooth_passes=self.centerline_post_smooth_passes,
            refine_passes=self.centerline_refine_passes,
        )
        if len(cx) < self.centerline_min_points:
            self.get_logger().warn(
                f'Centerline extraction: only {len(cx)} points (need '
                f'{self.centerline_min_points})',
                throttle_duration_sec=3.0)
            return False

        alpha_left = alpha_right = None
        track_half_width = self.racing_track_half_width
        if self.racing_use_map_corridor:
            alpha_left, alpha_right = map_corridor_bounds_for_polyline(
                self.live_grid,
                self.live_grid_info,
                cx, cy,
                margin=self.racing_boundary_margin,
                max_cap=self.racing_mincurv_max_offset,
            )
            track_half_width = mean_corridor_half_width(
                self.live_grid, self.live_grid_info, cx, cy) * 0.5

        try:
            mincurv_x, mincurv_y, mincurv_info = compute_mincurv_racing_line(
                cx, cy,
                max_offset=self.racing_mincurv_max_offset,
                iters=self.racing_mincurv_iters,
                alpha_max_left=alpha_left,
                alpha_max_right=alpha_right,
            )
        except ValueError as exc:
            self.get_logger().error(f'Min-curvature racing line failed: {exc}')
            return False

        try:
            rx_map, ry_map, smooth_info = compute_smooth_racing_line(
                mincurv_x, mincurv_y, cx, cy,
                alpha=self.racing_smooth_alpha,
                iters=self.racing_smooth_iters,
                max_dev=self.racing_smooth_max_dev,
                half_width=track_half_width,
                pre_iters=self.racing_smooth_pre_iters,
                pre_alpha=self.racing_smooth_pre_alpha,
                coarse_points=self.racing_smooth_coarse_points,
                energy_improve_ratio=self.racing_smooth_energy_ratio,
                require_radius_improvement=True,
            )
        except ValueError as exc:
            self.get_logger().error(f'Racing line smooth failed: {exc}')
            return False

        try:
            self._map_to_odom_snapshot = self._lookup_map_to_odom()
        except Exception:
            self.get_logger().warn('No map->odom TF at build time; using live TF for goals')
            self._map_to_odom_snapshot = None

        self.rx_map = np.asarray(rx_map, dtype=float)
        self.ry_map = np.asarray(ry_map, dtype=float)
        self.racing_n = len(self.rx_map)
        self._racing_line_ready = True
        self.closest_id = 0

        corridor_note = ''
        if mincurv_info.get('map_corridor'):
            corridor_note = (
                f', map corridor L/R {mincurv_info["alpha_bound_left_mean"]:.1f}/'
                f'{mincurv_info["alpha_bound_right_mean"]:.1f} m'
            )
        mincurv_note = (
            f'mincurv R_min {mincurv_info["min_radius_center"]:.2f} -> '
            f'{mincurv_info["min_radius_racing"]:.2f} m{corridor_note}'
        )
        if 'fallback' in mincurv_info:
            mincurv_note += f' (fallback: {mincurv_info["fallback"]})'

        smooth_note = (
            f'smooth R_min {smooth_info["min_radius_before"]:.2f} -> '
            f'{smooth_info["min_radius_after"]:.2f} m'
        )
        if 'fallback' in smooth_info:
            smooth_note += f' (fallback: {smooth_info["fallback"]})'

        self.get_logger().info(
            f'Racing line ready: {smooth_info["n_points"]} pts, {mincurv_note}, {smooth_note}, '
            f'max offset vs centerline {smooth_info["max_centerline_offset"]:.2f} m')
        return True

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

        scan_available = ranges is not None and len(ranges) > 0

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

        # Align the anchor with the first goal direction so the cubic spline's
        # tangent at the car's position immediately follows the corridor rather
        # than extending the car's current heading.  On a straight road the
        # first goal is forward and this reduces to the original behaviour.
        if pts:
            dx = pts[0][0] - self.x
            dy = pts[0][1] - self.y
            d = float(np.hypot(dx, dy)) or 1.0
            anchor_x, anchor_y = dx / d, dy / d
        else:
            anchor_x, anchor_y = fwd_x, fwd_y

        behind = [
            (self.x - self.goal_step * anchor_x, self.y - self.goal_step * anchor_y),
            (self.x, self.y),
        ]
        # Only transform map-frame pts to odom. Scan-based pts are already in odom.
        if not scan_available and map_pose is not None and grid is not None:
            try:
                map_to_odom = self._lookup_map_to_odom()
                pts = [map_point_to_odom(px, py, map_to_odom) for px, py in pts]
            except Exception:
                mx, my, _ = map_pose
                dx = self.x - mx
                dy = self.y - my
                pts = [(px + dx, py + dy) for px, py in pts]

        px = [p[0] for p in behind + pts]
        py = [p[1] for p in behind + pts]
        self._emit_goals(px, py, 'explore', force=True)

    def _racing_line_odom(self) -> tuple[np.ndarray, np.ndarray] | None:
        map_to_odom = self._map_to_odom_snapshot
        if map_to_odom is None:
            try:
                map_to_odom = self._lookup_map_to_odom()
            except Exception:
                return None
        ox, oy = [], []
        for px, py in zip(self.rx_map, self.ry_map):
            x, y = map_point_to_odom(float(px), float(py), map_to_odom)
            ox.append(x)
            oy.append(y)
        return np.asarray(ox, dtype=float), np.asarray(oy, dtype=float)

    def _publish_racing_goals(self):
        if not self._racing_line_ready and not self._build_racing_line_from_map():
            return

        odom_line = self._racing_line_odom()
        if odom_line is None:
            return
        rx, ry = odom_line

        fx = self.x + self.cg2front * -np.sin(self.theta)
        fy = self.y + self.cg2front * np.cos(self.theta)

        self.closest_id = closest_waypoint_index_closed(
            fx, fy, rx, ry,
            start_idx=self.closest_id,
            search_ahead=self.search_ahead,
        )
        cid = self.closest_id

        transform = self._body_offset(
            rx[cid], ry[cid], fx, fy, self.theta)

        half = self.wp_ahead + self.wp_behind
        if transform[1] < -self.passed_threshold:
            lo = cid - (self.wp_behind - 1)
            hi = cid + (self.wp_ahead + 1)
        else:
            lo = cid - self.wp_behind
            hi = cid + self.wp_ahead

        # Wrap indices for a closed loop so the end-of-array seam is smooth.
        indices = [i % self.racing_n for i in range(lo, hi)]
        if not indices:
            indices = list(range(min(half, self.racing_n)))
        mode = 'wrap' if (lo < 0 or hi > self.racing_n) else ('start' if cid < 2 else 'race')
        px = rx[indices].tolist()
        py = ry[indices].tolist()

        self._emit_goals(px, py, mode, cid)

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
