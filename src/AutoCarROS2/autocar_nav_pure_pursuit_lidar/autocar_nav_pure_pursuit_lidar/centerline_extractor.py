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


def _hit_xy_in_odom(
        angle: float,
        dist: float,
        lidar_x: float,
        lidar_y: float,
        lidar_yaw: float) -> tuple[float, float]:
    """Scan endpoint in odom (angles relative to lidar link frame)."""
    px = dist * math.cos(angle)
    py = dist * math.sin(angle)
    wx = lidar_x + px * math.cos(lidar_yaw) - py * math.sin(lidar_yaw)
    wy = lidar_y + px * math.sin(lidar_yaw) + py * math.cos(lidar_yaw)
    return wx, wy


def boundary_clearances_from_scan(
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        x: float,
        y: float,
        yaw: float,
        cg_to_lidar: float) -> tuple[float, float]:
    """Return (left_clearance, right_clearance) in metres using body-frame lateral.

    The Hokuyo is mounted with yaw=pi/2 on base_link, so scan-angle sign does NOT
    map to vehicle left/right.  Classify hits by lateral offset in the odom frame.
    """
    lidar_x, lidar_y, lidar_yaw = _lidar_pose(x, y, yaw, cg_to_lidar)
    left_x, left_y = _left_vector(yaw)

    left_dists: list[float] = []
    right_dists: list[float] = []

    for i in range(len(ranges)):
        r = float(ranges[i])
        if not math.isfinite(r) or r < range_min or r >= range_max:
            continue
        angle = angle_min + i * angle_increment
        hx, hy = _hit_xy_in_odom(angle, r, lidar_x, lidar_y, lidar_yaw)
        lat = (hx - x) * left_x + (hy - y) * left_y
        if lat > 0.4:
            left_dists.append(r)
        elif lat < -0.4:
            right_dists.append(r)

    def _robust_min(vals: list[float], default: float) -> float:
        if not vals:
            return default
        return float(np.percentile(vals, 15))

    return _robust_min(left_dists, TRACK_HALF_WIDTH), _robust_min(right_dists, TRACK_HALF_WIDTH)


def _lateral_center_offset(left_clr: float, right_clr: float) -> float:
    """Shift toward the side with more clearance (positive = shift left)."""
    offset = (left_clr - right_clr) * 0.5
    if left_clr < MIN_WALL_DISTANCE:
        offset -= (MIN_WALL_DISTANCE - left_clr) * 0.6
    if right_clr < MIN_WALL_DISTANCE:
        offset += (MIN_WALL_DISTANCE - right_clr) * 0.6
    return float(np.clip(offset, -MAX_LATERAL_OFFSET, MAX_LATERAL_OFFSET))


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
    """Build local centerline using body-frame left/right clearance."""
    left_clr, right_clr = boundary_clearances_from_scan(
        ranges, angle_min, angle_increment, range_min, range_max,
        x, y, yaw, cg_to_lidar)

    lateral_offset = _lateral_center_offset(left_clr, right_clr)

    fwd_x, fwd_y = forward_vector(yaw)
    left_x, left_y = _left_vector(yaw)
    base_x = x + cg_to_lidar * fwd_x
    base_y = y + cg_to_lidar * fwd_y

    return [
        (
            base_x + d * fwd_x + lateral_offset * left_x,
            base_y + d * fwd_y + lateral_offset * left_y,
        )
        for d in forward_distances
    ]


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
