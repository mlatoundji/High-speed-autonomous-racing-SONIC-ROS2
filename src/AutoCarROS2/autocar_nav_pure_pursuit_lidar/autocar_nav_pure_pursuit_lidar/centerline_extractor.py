"""Extract a short local centerline from LiDAR and/or SLAM occupancy map.

Lap-1 exploration uses corridor midpoints between detected track boundaries
(inner/outer bollards or walls) ahead of the vehicle.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from autocar_nav_pure_pursuit_lidar.pure_pursuit import forward_vector, right_vector

# SLAM / nav_msgs: treat likely-occupied cells as walls (unknown is -1).
OCCUPIED = 50
RAY_STEP = 0.25
LIDAR_MOUNT_YAW = math.pi / 2  # hokuyo_link yaw relative to base_link
TRACK_HALF_WIDTH = 8.0
MAX_LATERAL_OFFSET = 3.0
MIN_WALL_DISTANCE = 2.5


def _left_vector(yaw: float) -> tuple[float, float]:
    rx, ry = right_vector(yaw)
    return -rx, -ry


def _lidar_pose(x: float, y: float, yaw: float, cg_to_lidar: float) -> tuple[float, float, float]:
    fwd_x, fwd_y = forward_vector(yaw)
    lx = x + cg_to_lidar * fwd_x
    ly = y + cg_to_lidar * fwd_y
    return lx, ly, yaw + LIDAR_MOUNT_YAW


def _lateral_center_offset(left_clr: float, right_clr: float) -> float:
    """Shift toward the side with more clearance (positive = shift left)."""
    offset = (left_clr - right_clr) * 0.5
    if left_clr < MIN_WALL_DISTANCE:
        offset -= (MIN_WALL_DISTANCE - left_clr) * 0.6
    if right_clr < MIN_WALL_DISTANCE:
        offset += (MIN_WALL_DISTANCE - right_clr) * 0.6
    return float(np.clip(offset, -MAX_LATERAL_OFFSET, MAX_LATERAL_OFFSET))


def _gap_center_angle(angles: np.ndarray, clear: np.ndarray) -> float | None:
    """Return the center angle of the largest contiguous arc of clear rays."""
    if not np.any(clear):
        return None
    n = len(clear)
    best_start = 0
    best_len = 0
    cur_start = 0
    cur_len = 0
    for i in range(n):
        if clear[i]:
            if cur_len == 0:
                cur_start = i
            cur_len += 1
            if cur_len > best_len:
                best_len = cur_len
                best_start = cur_start
        else:
            cur_len = 0
    center_idx = best_start + best_len // 2
    return float(angles[center_idx])


def _world_to_grid(x: float, y: float, info) -> tuple[int, int] | None:
    col = int((x - info.origin.position.x) / info.resolution)
    row = int((y - info.origin.position.y) / info.resolution)
    if 0 <= col < info.width and 0 <= row < info.height:
        return col, row
    return None


def _grid_value(grid: np.ndarray, info, x: float, y: float) -> int | None:
    cell = _world_to_grid(x, y, info)
    if cell is None:
        return None
    col, row = cell
    return int(grid[row, col])


def _lateral_ray_to_boundary(
        grid: np.ndarray,
        info,
        ox: float,
        oy: float,
        dir_x: float,
        dir_y: float,
        max_dist: float = 14.0) -> float | None:
    """March along unit direction until occupied cell or max_dist."""
    norm = math.hypot(dir_x, dir_y) or 1.0
    ux, uy = dir_x / norm, dir_y / norm
    dist = RAY_STEP
    while dist <= max_dist:
        val = _grid_value(grid, info, ox + ux * dist, oy + uy * dist)
        if val is not None and val > OCCUPIED:
            return dist
        dist += RAY_STEP
    return None


def corridor_geometry_from_map(
        grid: np.ndarray,
        info,
        x: float,
        y: float,
        tangent_psi: float) -> tuple[float, float, float, float]:
    """Geometric corridor sample: midpoint and left/right clearances (m) from the grid.

    ``tangent_psi`` is the path tangent angle (not body yaw). Rays are cast
    perpendicular to this heading.
    """
    body_yaw = tangent_psi - math.pi / 2
    left_x, left_y = _left_vector(body_yaw)

    left_d = _lateral_ray_to_boundary(grid, info, x, y, left_x, left_y)
    right_d = _lateral_ray_to_boundary(grid, info, x, y, -left_x, -left_y)

    if left_d is not None and right_d is not None:
        offset = (left_d - right_d) * 0.5
        return (
            x + offset * left_x,
            y + offset * left_y,
            float(left_d),
            float(right_d),
        )

    if left_d is not None:
        return x, y, float(left_d), TRACK_HALF_WIDTH
    if right_d is not None:
        return x, y, TRACK_HALF_WIDTH, float(right_d)
    return x, y, TRACK_HALF_WIDTH, TRACK_HALF_WIDTH


def centerline_from_map(
        grid: np.ndarray,
        info,
        x: float,
        y: float,
        yaw: float,
        forward_distances: Iterable[float]) -> list[tuple[float, float]]:
    """Sample corridor midpoints using lateral ray casts on the occupancy grid."""
    fwd_x, fwd_y = forward_vector(yaw)
    tangent_psi = math.atan2(fwd_y, fwd_x)

    points: list[tuple[float, float]] = []
    for d in forward_distances:
        cx = x + d * fwd_x
        cy = y + d * fwd_y
        mx, my, left_d, right_d = corridor_geometry_from_map(
            grid, info, cx, cy, tangent_psi)
        if left_d < TRACK_HALF_WIDTH or right_d < TRACK_HALF_WIDTH:
            offset = _lateral_center_offset(left_d, right_d)
            mx = cx + offset * _left_vector(yaw)[0]
            my = cy + offset * _left_vector(yaw)[1]
        points.append((mx, my))
    return points


def centerline_from_scan(
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        x: float,
        y: float,
        yaw: float,
        cg_to_lidar: float,
        forward_distances: Iterable[float]) -> list[tuple[float, float]]:
    """Build local centerline by following the largest open gap in the scan.

    For each goal distance d, the largest contiguous arc of scan rays that are
    unobstructed at d determines the goal direction.  This handles hairpins and
    sharp corners where a forward wall would be invisible to a lateral-offset
    approach.
    """
    lidar_x, lidar_y, lidar_yaw = _lidar_pose(x, y, yaw, cg_to_lidar)
    n = len(ranges)

    angles = np.fromiter(
        (angle_min + i * angle_increment for i in range(n)), dtype=float, count=n)

    # Treat inf/NaN and out-of-range rays as range_max (no obstacle detected).
    r_arr = np.empty(n, dtype=float)
    for i in range(n):
        r = float(ranges[i])
        if math.isfinite(r) and range_min <= r < range_max:
            r_arr[i] = r
        else:
            r_arr[i] = range_max

    distances = list(forward_distances)
    nd = len(distances)

    # Pass 1: compute gap angle for every distance.
    fallback = 0.0
    gap_angles: list[float] = []
    for d in distances:
        a = _gap_center_angle(angles, r_arr >= d)
        if a is None:
            a = fallback
        else:
            fallback = a
        gap_angles.append(a)

    # Anticipation: near goals inherit the corridor direction seen at a
    # mid-range look-ahead (60th-percentile distance) so the car begins
    # steering before the outer wall enters the close-range scan arc.
    # Without this, gentle curves produce angle≈0 for small d (all-clear),
    # delaying steering until the wall is nearly adjacent.
    mid_idx = max(0, min(int(nd * 0.6), nd - 1))
    baseline = gap_angles[mid_idx]

    pts: list[tuple[float, float]] = []
    for i, (d, ga) in enumerate(zip(distances, gap_angles)):
        t = i / max(nd - 1, 1)          # 0 = near, 1 = far
        angle = (1.0 - t) * baseline + t * ga

        px = d * math.cos(angle)
        py = d * math.sin(angle)
        wx = lidar_x + px * math.cos(lidar_yaw) - py * math.sin(lidar_yaw)
        wy = lidar_y + px * math.sin(lidar_yaw) + py * math.cos(lidar_yaw)
        pts.append((wx, wy))

    return pts


def extract_local_centerline(
        x: float,
        y: float,
        yaw: float,
        cg_to_lidar: float,
        goal_step: float,
        goal_count: int,
        scan_ranges: np.ndarray | None = None,
        scan_angle_min: float = 0.0,
        scan_angle_increment: float = 0.0,
        scan_range_min: float = 0.12,
        scan_range_max: float = 30.0,
        grid: np.ndarray | None = None,
        grid_info=None,
        map_x: float | None = None,
        map_y: float | None = None,
        map_yaw: float | None = None) -> list[tuple[float, float]]:
    """Fuse scan-based and map-based corridor centerline (scan preferred on lap 1)."""
    distances = [goal_step * (i + 1) for i in range(goal_count)]

    if scan_ranges is not None and len(scan_ranges) > 0:
        pts = centerline_from_scan(
            scan_ranges, scan_angle_min, scan_angle_increment,
            scan_range_min, scan_range_max,
            x, y, yaw, cg_to_lidar, distances)
        if len(pts) >= 2:
            return pts

    if grid is not None and grid_info is not None:
        gx = map_x if map_x is not None else x
        gy = map_y if map_y is not None else y
        gyaw = map_yaw if map_yaw is not None else yaw
        pts = centerline_from_map(grid, grid_info, gx, gy, gyaw, distances)
        if len(pts) >= 2:
            return pts

    fwd_x, fwd_y = forward_vector(yaw)
    return [(x + d * fwd_x, y + d * fwd_y) for d in distances]
