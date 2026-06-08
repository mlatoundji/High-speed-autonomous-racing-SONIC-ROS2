"""Scan-to-map pose correction against a saved BOF occupancy grid."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

OCCUPIED = 50


@dataclass
class PoseCorrection:
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    score: float = 0.0


def _world_to_grid(x: float, y: float, info) -> tuple[int, int] | None:
    col = int((x - info.origin.position.x) / info.resolution)
    row = int((y - info.origin.position.y) / info.resolution)
    if 0 <= col < info.width and 0 <= row < info.height:
        return col, row
    return None


def _endpoint_score(
        grid: np.ndarray,
        info,
        lidar_x: float,
        lidar_y: float,
        yaw: float,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        dx: float,
        dy: float,
        dyaw: float) -> float:
    """Higher is better: fraction of scan endpoints landing on occupied cells."""
    cyaw = yaw + dyaw
    lx = lidar_x + dx
    ly = lidar_y + dy
    hits = 0
    total = 0

    for i in range(0, len(ranges), 4):
        r = float(ranges[i])
        if not math.isfinite(r) or r < range_min or r >= range_max:
            continue
        angle = angle_min + i * angle_increment
        px = r * math.cos(angle)
        py = r * math.sin(angle)
        wx = lx + px * math.cos(cyaw) - py * math.sin(cyaw)
        wy = ly + px * math.sin(cyaw) + py * math.cos(cyaw)
        cell = _world_to_grid(wx, wy, info)
        if cell is None:
            continue
        col, row = cell
        total += 1
        if grid[row, col] >= OCCUPIED:
            hits += 1

    return hits / max(total, 1)


def scan_match_pose(
        grid: np.ndarray,
        info,
        x: float,
        y: float,
        yaw: float,
        cg_to_lidar: float,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        search_xy: float = 1.0,
        search_yaw: float = 0.08,
        xy_step: float = 0.25,
        yaw_step: float = 0.02) -> PoseCorrection:
    """Brute-force 3-DOF scan matching on a static track map."""
    from autocar_nav_pure_pursuit.pure_pursuit import forward_vector

    fwd_x, fwd_y = forward_vector(yaw)
    lidar_x = x + cg_to_lidar * fwd_x
    lidar_y = y + cg_to_lidar * fwd_y

    best = PoseCorrection()
    best_score = -1.0

    n_xy = max(1, int(search_xy / xy_step))
    n_yaw = max(1, int(search_yaw / yaw_step))

    for ix in range(-n_xy, n_xy + 1):
        dx = ix * xy_step
        for iy in range(-n_xy, n_xy + 1):
            dy = iy * xy_step
            for iyaw in range(-n_yaw, n_yaw + 1):
                dyaw = iyaw * yaw_step
                score = _endpoint_score(
                    grid, info, lidar_x, lidar_y, yaw, ranges,
                    angle_min, angle_increment, range_min, range_max,
                    dx, dy, dyaw)
                if score > best_score:
                    best_score = score
                    best = PoseCorrection(dx=dx, dy=dy, dyaw=dyaw, score=score)

    return best
