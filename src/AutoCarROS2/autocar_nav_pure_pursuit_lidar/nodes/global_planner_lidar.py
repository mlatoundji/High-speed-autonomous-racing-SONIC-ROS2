#!/usr/bin/env python3
"""Hybrid global planner: lap-1 LiDAR centerline, lap-2+ smoothed racing line from SLAM map."""

import threading

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
        self._build_thread: threading.Thread | None = None
        self._racing_goals_force_once = False

        # Recorded driven trajectory (map frame) during the exploration lap. Used as
        # a robust centerline seed for the racing line (the corridor march derails at
        # corners; the driven path is a clean closed loop by construction).
        self._explore_path: list[tuple[float, float]] = []

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
                self._racing_goals_force_once = False
                self.get_logger().info(
                    'Lap 1 complete — building racing line from SLAM map')

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

    def _build_racing_line_safe(self) -> None:
        """Run the racing-line build, logging the traceback if it raises -- a daemon
        thread's exception would otherwise be silently swallowed."""
        try:
            self._build_racing_line_from_map()
        except Exception:  # noqa: BLE001
            import traceback
            self.get_logger().error(
                'Racing line build failed:\n' + traceback.format_exc())

    def _build_racing_line_from_map(self) -> None:
        # Snapshot mutable ROS state up front so this method is safe to run in a
        # background thread while timer_cb / map_cb continue on the main thread.
        grid = self.live_grid
        grid_info = self.live_grid_info
        if grid is None or grid_info is None:
            return
        if self.map_frame_id != 'map':
            return

        # Centerline seed = the recorded EXPLORATION TRAJECTORY (map frame). The car
        # drove a clean closed lap, so its driven path is a robust centerline — far
        # more reliable than marching the corridor (which derails at corners and
        # never closes the loop).
        path = list(self._explore_path)
        if len(path) < self.centerline_min_points:
            self.get_logger().warn(
                f'Exploration path too short: {len(path)} points (need '
                f'{self.centerline_min_points})',
                throttle_duration_sec=3.0)
            return
        cx = np.asarray([p[0] for p in path], dtype=float)
        cy = np.asarray([p[1] for p in path], dtype=float)

        # Resample the driven loop to UNIFORM spacing as a CLOSED loop. Recording
        # starts a bit after spawn (when the SLAM TF is up) and ends at the finish
        # line, so the first/last points don't meet -> a big seam jump that makes the
        # car get stuck oscillating at the loop seam (idx 0/146). Interpolating the
        # closed loop (the seam falls on the main straight) removes that jump.
        px_loop = np.append(cx, cx[0])
        py_loop = np.append(cy, cy[0])
        seg = np.hypot(np.diff(px_loop), np.diff(py_loop))
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(cum[-1])
        if total > 1e-6:
            n_pts = max(int(total / self.centerline_step), self.centerline_min_points)
            s = np.linspace(0.0, total, n_pts, endpoint=False)
            cx = np.interp(s, cum, px_loop)
            cy = np.interp(s, cum, py_loop)

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

    def _record_explore_pose(self):
        """Append the car's map-frame position to the driven trajectory, spaced by
        ~centerline_step so it directly serves as a centerline seed."""
        mp = self._map_frame_pose()
        if mp is None:
            return
        if (not self._explore_path
                or np.hypot(mp[0] - self._explore_path[-1][0],
                            mp[1] - self._explore_path[-1][1]) >= self.centerline_step):
            self._explore_path.append((float(mp[0]), float(mp[1])))

    def timer_cb(self):
        if self.x is None:
            return

        # Record the driven path during exploration (lap 1) -> centerline seed.
        if self.lap_count < 1:
            self._record_explore_pose()

        mode_msg = Int32()
        if self.lap_count < 1 or not self._racing_line_ready:
            mode_msg.data = 0  # exploration (also while racing line is building)
            self.mode_pub.publish(mode_msg)
            if self.lap_count >= 1:
                if self._build_thread is None or not self._build_thread.is_alive():
                    t = threading.Thread(
                        target=self._build_racing_line_safe, daemon=True)
                    self._build_thread = t
                    t.start()
            self._publish_exploration_goals()
        else:
            mode_msg.data = 1  # racing
            self.mode_pub.publish(mode_msg)
            if not self._publish_racing_goals():
                self.get_logger().warn(
                    'Racing goals unavailable — holding exploration goals',
                    throttle_duration_sec=2.0)
                self._publish_exploration_goals()

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
        if len(px) < 2:
            return False
        limit = max_seg if max_seg is not None else max(
            4.0 * self.goal_step, 4.0 * self.centerline_step)
        for i in range(len(px) - 1):
            if float(np.hypot(px[i + 1] - px[i], py[i + 1] - py[i])) > limit:
                return False
        axle = self._front_axle_xy()
        if axle is not None:
            fx, fy = axle
            if float(np.hypot(px[0] - fx, py[0] - fy)) > limit:
                return False
        return True

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

        cid = closest_waypoint_index_closed(
            fx, fy, rx, ry,
            start_idx=self.closest_id,
            search_ahead=self.search_ahead,
        )
        if float(np.hypot(rx[cid] - fx, ry[cid] - fy)) > 6.0:
            cid = closest_waypoint_index_closed_disambiguated(
                fx, fy, self.theta, rx, ry, hint_idx=cid)
        # Still far -> re-acquire globally. closest_id goes stale while the car
        # explores before the first racing goal succeeds, so the windowed search
        # misses the car's true position on the loop. A full argmin bootstraps it.
        gd = float(np.hypot(rx[cid] - fx, ry[cid] - fy))
        if gd > 6.0:
            # Direction-aware global re-acquire. A pure geometric argmin grabs the
            # trajectory's START point (the spawn) which is right BEHIND the car at the
            # seam -> goals loop backward. Pick the closest point that is AHEAD of the
            # car instead (projection of point-car onto heading > 0).
            # Forward is (-sin, cos) here: self.theta is the body yaw (heading-90deg),
            # same convention as _front_axle_xy. Using (cos,sin) would be 90deg off.
            hx, hy = -np.sin(self.theta), np.cos(self.theta)
            ahead = (rx - fx) * hx + (ry - fy) * hy
            d2 = (rx - fx) ** 2 + (ry - fy) ** 2
            fwd = np.nonzero(ahead > -1.0)[0]
            if fwd.size:
                cid = int(fwd[np.argmin(d2[fwd])])
            else:
                cid = int(np.argmin(d2))
            self.get_logger().info(
                f'Racing closest re-acquired (forward): idx {cid}, dist '
                f'{float(np.hypot(rx[cid] - fx, ry[cid] - fy)):.1f} m, '
                f'ahead={float(ahead[cid]):.1f} m (windowed was {gd:.1f} m)',
                throttle_duration_sec=2.0)
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

        # Do NOT let the "behind" window wrap BACKWARD across the loop seam. At the
        # start (cid small) lo goes negative and i % racing_n wraps it to the END of the
        # loop (the finish line, physically behind the car across the seam) -> the goal
        # path loops backward (the green line the user saw). Clamp so behind points never
        # cross the seam. The FORWARD wrap (hi > racing_n near the end) is kept untouched.
        if lo < 0:
            lo = 0
        # Forward arc along the loop (index order == track order).
        indices = [i % self.racing_n for i in range(lo, hi)]
        if not indices:
            indices = list(range(min(half, self.racing_n)))
        mode = 'wrap' if (lo < 0 or hi > self.racing_n) else ('start' if cid < 2 else 'race')
        px = rx[indices].tolist()
        py = ry[indices].tolist()
        # Drop racing goals BEHIND the front axle. The behind-window (and the seam at the
        # start) can place the first goals behind the car -- e.g. the spawn/idx0 point --
        # so the emitted path zigzags BACKWARD (the green line looping back, car reverses).
        # Keep from the first non-behind point; the prepended anchors give the spline its
        # start. forward = (-sin, cos) per the body-yaw convention.
        if self.theta is not None and px:
            fwx, fwy = -np.sin(self.theta), np.cos(self.theta)
            start = next(
                (i for i in range(len(px))
                 if (px[i] - fx) * fwx + (py[i] - fy) * fwy >= -1.0),
                len(px))
            px, py = px[start:], py[start:]
        px, py = self._prepend_path_anchors(px, py)

        if not self._goal_polyline_ok(px, py):
            self.get_logger().warn(
                f'Racing goals rejected: bad spacing near idx {cid} '
                f'(dist to car {float(np.hypot(rx[cid] - fx, ry[cid] - fy)):.1f} m)',
                throttle_duration_sec=2.0)
            return False

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
