"""Extract a closed-loop track centerline from a SLAM occupancy grid."""

from __future__ import annotations

import math

import numpy as np

from autocar_nav_pure_pursuit_lidar.centerline_extractor import corridor_geometry_from_map
from autocar_nav_pure_pursuit_lidar.map_track_geometry import refine_closed_centerline_from_map
from autocar_nav_pure_pursuit_lidar.pure_pursuit import forward_vector


def _body_yaw_from_heading(psi: float) -> float:
    return psi - math.pi * 0.5


def _moving_average_closed(xs: np.ndarray, ys: np.ndarray, passes: int) -> tuple[np.ndarray, np.ndarray]:
    """Light closed-loop smoothing to damp grid ray-cast noise."""
    if passes <= 0:
        return xs, ys
    out_x, out_y = xs.copy(), ys.copy()
    for _ in range(passes):
        out_x = 0.25 * np.roll(out_x, 1) + 0.5 * out_x + 0.25 * np.roll(out_x, -1)
        out_y = 0.25 * np.roll(out_y, 1) + 0.5 * out_y + 0.25 * np.roll(out_y, -1)
    return out_x, out_y


def _corridor_midpoint_with_tangent(
        grid,
        info,
        x: float,
        y: float,
        tangent_psi: float) -> tuple[float, float]:
    mx, my, _, _ = corridor_geometry_from_map(grid, info, x, y, tangent_psi)
    return mx, my


def extract_loop_centerline_from_map(
        grid: np.ndarray,
        info,
        start_x: float,
        start_y: float,
        start_yaw: float,
        step: float = 2.0,
        close_dist: float = 4.0,
        min_points: int = 20,
        max_points: int = 400,
        post_smooth_passes: int = 0,
        refine_passes: int = 3) -> tuple[np.ndarray, np.ndarray]:
    """March along the map corridor, then refine vertices against grid boundaries."""
    fwd_x, fwd_y = forward_vector(start_yaw)
    start_psi = math.atan2(fwd_y, fwd_x)
    x, y = start_x, start_y
    xs: list[float] = []
    ys: list[float] = []

    for _ in range(max_points):
        if len(xs) >= 2:
            tangent_psi = math.atan2(y - ys[-1], x - xs[-1])
        elif xs:
            tangent_psi = start_psi
        else:
            tangent_psi = start_psi

        mx, my = _corridor_midpoint_with_tangent(grid, info, x, y, tangent_psi)
        if xs and np.hypot(mx - xs[0], my - ys[0]) < close_dist and len(xs) >= min_points:
            break

        xs.append(mx)
        ys.append(my)

        if len(xs) >= 2:
            move_psi = math.atan2(my - ys[-2], mx - xs[-2])
        else:
            move_psi = tangent_psi

        move_x, move_y = forward_vector(_body_yaw_from_heading(move_psi))
        look_x = mx + step * move_x
        look_y = my + step * move_y
        nmx, nmy = _corridor_midpoint_with_tangent(grid, info, look_x, look_y, move_psi)
        ddx = nmx - mx
        ddy = nmy - my
        if abs(ddx) < 1e-6 and abs(ddy) < 1e-6:
            break

        psi = math.atan2(ddy, ddx)
        x, y = nmx, nmy

    arr_x = np.asarray(xs, dtype=float)
    arr_y = np.asarray(ys, dtype=float)
    if len(arr_x) >= 3 and refine_passes > 0:
        arr_x, arr_y = refine_closed_centerline_from_map(
            grid, info, arr_x, arr_y, passes=refine_passes)
    if post_smooth_passes > 0 and len(arr_x) >= 3:
        arr_x, arr_y = _moving_average_closed(arr_x, arr_y, post_smooth_passes)
    return arr_x, arr_y
