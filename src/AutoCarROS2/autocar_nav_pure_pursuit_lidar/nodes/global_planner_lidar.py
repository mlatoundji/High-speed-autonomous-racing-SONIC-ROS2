#!/usr/bin/env python3
"""Hybrid global planner: lap-1 LiDAR centerline, lap-2+ smoothed racing line from SLAM map."""

import csv
import threading
from pathlib import Path

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
    remove_fold_backs,
    snap_racing_line_to_free_space,
)
from autocar_nav_pure_pursuit_lidar.pure_pursuit import (
    closest_waypoint_index_closed,
    closest_waypoint_index_closed_disambiguated,
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
            ('centerline_min_loop_len', 200.0),
            ('centerline_min_bb_span', 80.0),
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
            ('racing_resample_step', 2.7),
            ('racing_build_delay_s', 5.0),
            ('run_id', ''),
            ('run_dir', ''),
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
        self.centerline_min_loop_len = float(self.get_parameter('centerline_min_loop_len').value)
        self.centerline_min_bb_span = float(self.get_parameter('centerline_min_bb_span').value)
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
        self.racing_resample_step = float(self.get_parameter('racing_resample_step').value)
        self.racing_build_delay_s = float(self.get_parameter('racing_build_delay_s').value)
        self.run_id = str(self.get_parameter('run_id').value).strip()
        self.run_dir = str(self.get_parameter('run_dir').value).strip()

        self.lap_count = 0
        self.x = None
        self.y = None
        self.theta = None
        self.closest_id = 0
        self._publish_key = None
        self._last_goal_reject_reason = ''

        self.rx_map = np.array([])
        self.ry_map = np.array([])
        self.racing_n = 0
        self._racing_line_ready = False
        self._build_thread: threading.Thread | None = None
        self._racing_goals_force_once = False
        self._last_racing_px: list[float] | None = None
        self._last_racing_py: list[float] | None = None
        self._racing_window_strategy: str = 'trimmed'
        self._racing_goals_active = False
        self._racing_line_saved = False
        self._lap1_complete_time = None

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
                self._racing_goals_force_once = True
                self._racing_goals_active = False
                self._racing_window_strategy = 'forward-only'
                self._racing_line_saved = False
                self._lap1_complete_time = self.get_clock().now()
                self.get_logger().info(
                    'Lap 1 complete — keep exploration steering until racing line ready')

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

    def _build_racing_line_from_map(self) -> None:
        # Snapshot mutable ROS state up front so this method is safe to run in a
        # background thread while timer_cb / map_cb continue on the main thread.
        grid = self.live_grid
        grid_info = self.live_grid_info
        if grid is None or grid_info is None:
            return
        if self.map_frame_id != 'map':
            return

        map_pose = self._map_frame_pose()
        if map_pose is None:
            return

        mx, my, myaw = map_pose
        cx, cy = extract_loop_centerline_from_map(
            grid, grid_info,
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
            return

        closure = float(np.hypot(cx[0] - cx[-1], cy[0] - cy[-1])) if len(cx) >= 2 else float('inf')
        closure_limit = max(2.0 * self.centerline_step, 1.5 * self.centerline_close_dist)
        if closure > closure_limit:
            self.get_logger().warn(
                f'Centerline extraction: open loop (closure {closure:.1f} m > {closure_limit:.1f} m); '
                'skip racing line build this cycle',
                throttle_duration_sec=2.0)
            return

        # Geometric sanity gates — reject partial/local loops that are too small.
        pts = np.column_stack([cx, cy])
        loop_segs = np.hypot(np.diff(np.append(pts[:, 0], pts[0, 0])),
                             np.diff(np.append(pts[:, 1], pts[0, 1])))
        loop_len = float(np.sum(loop_segs))
        bb_span_x = float(cx.max() - cx.min())
        bb_span_y = float(cy.max() - cy.min())
        bb_span = max(bb_span_x, bb_span_y)
        if loop_len < self.centerline_min_loop_len:
            self.get_logger().warn(
                f'Centerline extraction: loop too short ({loop_len:.1f} m < '
                f'{self.centerline_min_loop_len:.1f} m); skip racing line build',
                throttle_duration_sec=2.0)
            return
        if bb_span < self.centerline_min_bb_span:
            self.get_logger().warn(
                f'Centerline extraction: bbox too small ({bb_span:.1f} m < '
                f'{self.centerline_min_bb_span:.1f} m); skip racing line build',
                throttle_duration_sec=2.0)
            return

        alpha_left = alpha_right = None
        track_half_width = self.racing_track_half_width
        if self.racing_use_map_corridor:
            alpha_left, alpha_right = map_corridor_bounds_for_polyline(
                grid, grid_info,
                cx, cy,
                margin=self.racing_boundary_margin,
                max_cap=self.racing_mincurv_max_offset,
            )
            track_half_width = mean_corridor_half_width(
                grid, grid_info, cx, cy) * 0.5

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
            return

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
            return

        # Guard against map gaps: snap any point pushed into or too close to
        # an occupied cell back toward the nearest centerline point.
        rx_map, ry_map = snap_racing_line_to_free_space(
            rx_map, ry_map, cx, cy,
            grid, grid_info,
            wall_margin=self.racing_boundary_margin,
        )

        # Remove any fold-back artefacts (single-step direction reversals >107°)
        # that may have been introduced by loop-seam overshoot or snap collapses.
        # Legitimate hairpins produce ≤40° per-step turns after smoothing and are
        # not removed.
        n_before = len(rx_map)
        rx_map, ry_map = remove_fold_backs(rx_map, ry_map)
        n_removed = n_before - len(rx_map)
        if n_removed:
            self.get_logger().warn(
                f'Racing line: removed {n_removed} fold-back points '
                f'({n_before} → {len(rx_map)} pts)')

        if self.racing_resample_step > 0.0 and len(rx_map) >= 4:
            rx_map, ry_map = self._resample_closed_uniform(
                np.asarray(rx_map, dtype=float),
                np.asarray(ry_map, dtype=float),
                self.racing_resample_step,
            )

        rx_arr = np.asarray(rx_map, dtype=float)
        ry_arr = np.asarray(ry_map, dtype=float)
        min_racing_pts = max(8, self.wp_ahead + self.wp_behind + 2)
        if len(rx_arr) < min_racing_pts:
            self.get_logger().warn(
                f'Racing line: only {len(rx_arr)} points after cleanup '
                f'(need {min_racing_pts})',
                throttle_duration_sec=3.0)
            return

        init_closest = self._closest_racing_index_live(rx_arr, ry_arr)

        # Assign all racing line state with _racing_line_ready set last so that
        # the main thread never sees a partially-initialised racing line.
        self.rx_map = rx_arr
        self.ry_map = ry_arr
        self.racing_n = len(rx_arr)
        self.closest_id = init_closest
        self._racing_goals_force_once = True
        self._racing_line_ready = True
        if not self._racing_line_saved:
            self._save_racing_line_results(rx_arr, ry_arr)
            self._racing_line_saved = True

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
            f'max offset vs centerline {smooth_info["max_centerline_offset"]:.2f} m, '
            f'init closest_id={init_closest}')

    def _save_racing_line_results(self, rx_map: np.ndarray, ry_map: np.ndarray) -> None:
        """Persist map/odom racing line to the benchmark run directory."""
        if not self.run_dir:
            self.get_logger().warn(
                'Racing line not saved: run_dir parameter is empty.',
                throttle_duration_sec=5.0)
            return
        run_path = Path(self.run_dir).expanduser()
        try:
            run_path.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self.get_logger().warn(f'Racing line save skipped: cannot create run_dir ({exc})')
            return

        map_csv = run_path / 'racing_line_map.csv'
        try:
            with map_csv.open('w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(['index', 'x_map', 'y_map'])
                for i, (x, y) in enumerate(zip(rx_map, ry_map)):
                    w.writerow([i, float(x), float(y)])
        except Exception as exc:
            self.get_logger().warn(f'Failed writing {map_csv.name}: {exc}')
            return

        odom_line = self._racing_line_odom_from_map(rx_map, ry_map)
        if odom_line is None:
            self.get_logger().warn(
                f'Racing line saved to {map_csv.name}; odom export skipped (TF unavailable).')
            return
        ox, oy = odom_line
        odom_csv = run_path / 'racing_line_odom.csv'
        try:
            with odom_csv.open('w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(['index', 'x_odom', 'y_odom'])
                for i, (x, y) in enumerate(zip(ox, oy)):
                    w.writerow([i, float(x), float(y)])
        except Exception as exc:
            self.get_logger().warn(f'Failed writing {odom_csv.name}: {exc}')
            return

        self.get_logger().info(
            f'Racing line saved: {map_csv.name} and {odom_csv.name} '
            f'({len(rx_map)} points, run_id={self.run_id or "unknown"})')

    def _resample_closed_uniform(
            self,
            xs: np.ndarray,
            ys: np.ndarray,
            step: float) -> tuple[np.ndarray, np.ndarray]:
        """Uniformly resample a closed loop to avoid long sparse segments."""
        if len(xs) < 3 or step <= 0.0:
            return xs, ys
        closure = float(np.hypot(xs[0] - xs[-1], ys[0] - ys[-1]))
        if closure > max(3.0 * step, 6.0):
            # Not a closed loop; keep original points and let caller decide.
            self.get_logger().warn(
                f'Resample skipped: path is open (closure {closure:.1f} m)',
                throttle_duration_sec=2.0)
            return xs, ys
        pts = np.column_stack([xs, ys])
        pts = np.vstack([pts, pts[0]])
        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        total = float(np.sum(seg))
        if total < 1e-6:
            return xs, ys
        n = max(8, int(total / max(step, 0.1)))
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        s = np.linspace(0.0, total, n, endpoint=False)
        rx = np.interp(s, cum, pts[:, 0])
        ry = np.interp(s, cum, pts[:, 1])
        return np.asarray(rx, dtype=float), np.asarray(ry, dtype=float)

    def timer_cb(self):
        if self.x is None:
            return

        mode_msg = Int32()
        if self.lap_count < 1:
            mode_msg.data = 0  # exploration
            self.mode_pub.publish(mode_msg)
            self._publish_exploration_goals()
            return

        can_build = True
        if self.racing_build_delay_s > 0.0 and self._lap1_complete_time is not None:
            elapsed = (self.get_clock().now() - self._lap1_complete_time).nanoseconds * 1e-9
            can_build = elapsed >= self.racing_build_delay_s
            if not can_build:
                self.get_logger().info(
                    f'Racing build delayed: {elapsed:.1f}/{self.racing_build_delay_s:.1f} s',
                    throttle_duration_sec=1.0)
        if (not self._racing_line_ready) and can_build and (
                self._build_thread is None or not self._build_thread.is_alive()):
            t = threading.Thread(
                target=self._build_racing_line_from_map, daemon=True)
            self._build_thread = t
            t.start()

        # Transition: keep LiDAR exploration goals + nav_mode=0 for continuous
        # steering until the racing line is built and first racing goals publish.
        if not self._racing_goals_active:
            if self._racing_line_ready and self._publish_racing_goals():
                self._racing_goals_active = True
                mode_msg.data = 1
                self.mode_pub.publish(mode_msg)
                self.get_logger().info(
                    'Lap2 steering handoff: exploration -> racing line (nav_mode=1)')
                return
            mode_msg.data = 0
            self.mode_pub.publish(mode_msg)
            self._publish_exploration_goals()
            return

        # Steady racing: never fall back to exploration goals.
        mode_msg.data = 1
        self.mode_pub.publish(mode_msg)
        if self._publish_racing_goals():
            return

        if self._last_racing_px and self._last_racing_py:
            self._emit_goals(
                self._last_racing_px,
                self._last_racing_py,
                'race-hold',
                cid=self.closest_id,
                force=False)
            self.get_logger().warn(
                'Racing goals unavailable — reusing last valid racing goals',
                throttle_duration_sec=2.0)
        else:
            self.get_logger().warn(
                'Racing goals unavailable and no cached racing goals yet',
                throttle_duration_sec=2.0)

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
        """Transform cached map-frame racing line using live map→odom TF.

        A frozen TF snapshot drifts from SLAM-corrected state2D after loop
        closure at the end of lap 1, which misplaces goals/path in RViz.
        """
        return self._racing_line_odom_from_map(self.rx_map, self.ry_map)

    def _front_axle_xy(self) -> tuple[float, float] | None:
        if self.x is None or self.theta is None:
            return None
        fx = self.x + self.cg2front * -np.sin(self.theta)
        fy = self.y + self.cg2front * np.cos(self.theta)
        return fx, fy

    def _closest_racing_index_live(
            self,
            rx_map: np.ndarray,
            ry_map: np.ndarray,
            hint_idx: int = 0) -> int:
        """Nearest forward index on the racing line (heading-disambiguated)."""
        axle = self._front_axle_xy()
        if axle is None or self.theta is None:
            return 0
        odom_line = self._racing_line_odom_from_map(rx_map, ry_map)
        if odom_line is None:
            return 0
        rx, ry = odom_line
        fx, fy = axle
        return closest_waypoint_index_closed_disambiguated(
            fx, fy, self.theta, rx, ry, hint_idx=hint_idx)

    def _racing_line_odom_from_map(
            self,
            rx_map: np.ndarray,
            ry_map: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            map_to_odom = self._lookup_map_to_odom()
        except Exception:
            return None
        ox, oy = [], []
        for px, py in zip(rx_map, ry_map):
            x, y = map_point_to_odom(float(px), float(py), map_to_odom)
            ox.append(x)
            oy.append(y)
        return np.asarray(ox, dtype=float), np.asarray(oy, dtype=float)

    def _prepend_path_anchors(
            self,
            px: list[float],
            py: list[float]) -> tuple[list[float], list[float]]:
        """Anchor spline start at the car, matching exploration goal layout."""
        if not px or self.x is None or self.y is None or self.theta is None:
            return px, py

        # Use body heading, not the chord toward the first goal — if that goal is
        # on the wrong loop branch the chord can steer the spline across the track.
        fwd_x, fwd_y = forward_vector(self.theta)
        behind = [
            (self.x - self.goal_step * fwd_x, self.y - self.goal_step * fwd_y),
            (self.x, self.y),
        ]
        return [p[0] for p in behind] + px, [p[1] for p in behind] + py

    def _goal_polyline_ok(
            self,
            px: list[float],
            py: list[float],
            max_seg: float | None = None) -> bool:
        """Reject polylines with long jumps that produce infield spline loops."""
        self._last_goal_reject_reason = ''
        if len(px) < 2:
            self._last_goal_reject_reason = 'not enough points'
            return False
        limit = max_seg if max_seg is not None else max(
            4.0 * self.goal_step, 4.0 * self.centerline_step)
        for i in range(len(px) - 1):
            seg = float(np.hypot(px[i + 1] - px[i], py[i + 1] - py[i]))
            if seg > limit:
                self._last_goal_reject_reason = (
                    f'long segment {i}->{i + 1}: {seg:.1f} m > {limit:.1f} m')
                return False
        axle = self._front_axle_xy()
        if axle is not None:
            fx, fy = axle
            # With anchors prepended by `_prepend_path_anchors`, indices are:
            # 0 = behind anchor, 1 = car anchor, 2 = first racing waypoint.
            car_idx = 1 if len(px) >= 2 else 0
            first_dist = float(np.hypot(px[car_idx] - fx, py[car_idx] - fy))
            if first_dist > max(2.0, 0.75 * self.goal_step):
                self._last_goal_reject_reason = (
                    f'car anchor too far: {first_dist:.1f} m')
                return False

            # Prevent immediate "hook" after anchor insertion: segment from car
            # anchor to the first racing waypoint must stay forward and local.
            if len(px) >= 3 and self.theta is not None:
                fwd_x, fwd_y = forward_vector(self.theta)
                sx = float(px[2] - px[car_idx])
                sy = float(py[2] - py[car_idx])
                first_seg = float(np.hypot(sx, sy))
                local_limit = max(2.5 * self.goal_step, 2.0 * self.centerline_step)
                if first_seg > local_limit:
                    self._last_goal_reject_reason = (
                        f'car->first waypoint too long: {first_seg:.1f} m > {local_limit:.1f} m')
                    return False
                # Allow small local backward component on tight corners.
                forward_proj = sx * fwd_x + sy * fwd_y
                if forward_proj < -2.0:
                    self._last_goal_reject_reason = (
                        f'car->first waypoint backward: proj {forward_proj:.1f} m')
                    return False
        return True

    def _trim_leading_points_behind_car(
            self,
            px: list[float],
            py: list[float]) -> tuple[list[float], list[float]]:
        """Drop leading racing-window points that lie behind the car.

        The racing window intentionally includes a few points behind `closest_id`
        for continuity. After prepending anchors, starting the polyline with these
        behind-points can force the spline to hook backwards near the car.
        """
        if not px or self.x is None or self.y is None or self.theta is None:
            return px, py

        fwd_x, fwd_y = forward_vector(self.theta)
        keep_from = 0
        for i, (x, y) in enumerate(zip(px, py)):
            dx = float(x - self.x)
            dy = float(y - self.y)
            if dx * fwd_x + dy * fwd_y >= 0.0:
                keep_from = i
                break
            keep_from = i + 1

        # Keep at least one point if all were behind.
        if keep_from >= len(px):
            keep_from = len(px) - 1
        return px[keep_from:], py[keep_from:]

    def _publish_racing_goals(self) -> bool:
        if not self._racing_line_ready:
            return False

        odom_line = self._racing_line_odom()
        if odom_line is None:
            return False
        rx, ry = odom_line

        axle = self._front_axle_xy()
        if axle is None:
            return False
        fx, fy = axle

        # 1) Fast local tracking from previous index (steady-state).
        cid_local = closest_waypoint_index_closed(
            fx, fy, rx, ry,
            start_idx=self.closest_id,
            search_ahead=self.search_ahead,
        )
        dist_local = float(np.hypot(rx[cid_local] - fx, ry[cid_local] - fy))

        # 2) Robust global re-anchor + heading disambiguation (lap switch / seam).
        d2 = (rx - fx) ** 2 + (ry - fy) ** 2
        seed = int(np.argmin(d2))
        cid_global = closest_waypoint_index_closed_disambiguated(
            fx, fy, self.theta, rx, ry, hint_idx=seed, behind_margin=8.0)
        dist_global = float(np.hypot(rx[cid_global] - fx, ry[cid_global] - fy))

        # Prefer local continuity when valid; otherwise trust global anchor.
        if dist_local <= 10.0:
            cid = cid_local
            dist_to_cid = dist_local
        else:
            cid = cid_global
            dist_to_cid = dist_global

        if dist_to_cid > 20.0:
            self.get_logger().warn(
                f'Racing re-anchor failed: nearest idx {cid} still {dist_to_cid:.1f} m away',
                throttle_duration_sec=2.0)
            return False
        self.closest_id = cid

        transform = self._body_offset(
            rx[cid], ry[cid], fx, fy, self.theta)

        half = self.wp_ahead + self.wp_behind
        if transform[1] < -self.passed_threshold:
            lo = cid - (self.wp_behind - 1)
            hi = cid + (self.wp_ahead + 1)
        else:
            lo = cid - self.wp_behind
            hi = cid + self.wp_ahead

        # Forward arc along the loop (index order == track order).
        indices = [i % self.racing_n for i in range(lo, hi)]
        if not indices:
            indices = list(range(min(half, self.racing_n)))
        mode = 'wrap' if (lo < 0 or hi > self.racing_n) else ('start' if cid < 2 else 'race')
        px_base = rx[indices].tolist()
        py_base = ry[indices].tolist()
        half = self.wp_ahead + self.wp_behind

        # Try multiple window constructions before giving up.
        candidates: dict[str, tuple[list[float], list[float]]] = {}
        tpx, tpy = self._trim_leading_points_behind_car(px_base, py_base)
        candidates['trimmed'] = self._prepend_path_anchors(tpx, tpy)
        candidates['untrimmed'] = self._prepend_path_anchors(px_base, py_base)
        fw_indices = [(cid + i) % self.racing_n for i in range(max(2, half))]
        fw_px = rx[fw_indices].tolist()
        fw_py = ry[fw_indices].tolist()
        candidates['forward-only'] = self._prepend_path_anchors(fw_px, fw_py)

        picked: tuple[str, list[float], list[float]] | None = None
        reject_reasons: list[str] = []
        ordered_labels = [self._racing_window_strategy]
        ordered_labels += [k for k in ('trimmed', 'untrimmed', 'forward-only')
                           if k != self._racing_window_strategy and k in candidates]
        for label in ordered_labels:
            cx, cy = candidates[label]
            if self._goal_polyline_ok(cx, cy):
                picked = (label, cx, cy)
                break
            reject_reasons.append(f'{label}: {self._last_goal_reject_reason or "unknown"}')

        if picked is None:
            # Last resort: keep the previous valid racing goals to avoid mode thrash.
            if self._last_racing_px and self._last_racing_py:
                self.get_logger().warn(
                    f'Racing goals: reusing last valid window near idx {cid} '
                    f'(dist {dist_to_cid:.1f} m, rejects: {" | ".join(reject_reasons)})',
                    throttle_duration_sec=2.0)
                px, py = self._last_racing_px, self._last_racing_py
            else:
                self.get_logger().warn(
                    f'Racing goals rejected: bad spacing near idx {cid} '
                    f'(dist to car {dist_to_cid:.1f} m, '
                    f'reasons: {" | ".join(reject_reasons) or "unknown"})',
                    throttle_duration_sec=2.0)
                return False
        else:
            label, px, py = picked
            if label != self._racing_window_strategy:
                self.get_logger().warn(
                    f'Racing goals: strategy {self._racing_window_strategy} -> {label} near idx {cid}',
                    throttle_duration_sec=2.0)
            self._racing_window_strategy = label
            self._last_racing_px = px
            self._last_racing_py = py

        force = self._racing_goals_force_once
        self._racing_goals_force_once = False
        self._emit_goals(px, py, mode, cid, force=force)
        return True

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
