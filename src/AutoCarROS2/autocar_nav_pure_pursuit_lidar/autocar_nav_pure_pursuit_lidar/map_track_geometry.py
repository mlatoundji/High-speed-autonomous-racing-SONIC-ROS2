"""Map-native track geometry: corridor midpoints and per-point boundary limits."""

from __future__ import annotations

import math

import numpy as np

from autocar_nav_pure_pursuit_lidar.centerline_extractor import (
    TRACK_HALF_WIDTH,
    corridor_geometry_from_map,
)


def tangent_psi_closed(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """Path tangent angle (rad) for each vertex on a closed polyline."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    n = len(xs)
    if n < 2:
        return np.zeros(n, dtype=float)
    dx = np.roll(xs, -1) - np.roll(xs, 1)
    dy = np.roll(ys, -1) - np.roll(ys, 1)
    return np.arctan2(dy, dx)


def refine_closed_centerline_from_map(
        grid: np.ndarray,
        info,
        xs: np.ndarray,
        ys: np.ndarray,
        passes: int = 3) -> tuple[np.ndarray, np.ndarray]:
    """Re-snap each vertex to the geometric corridor midpoint from the occupancy grid."""
    if passes <= 0 or len(xs) < 3:
        return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)

    out_x = np.asarray(xs, dtype=float).copy()
    out_y = np.asarray(ys, dtype=float).copy()

    for _ in range(passes):
        tangents = tangent_psi_closed(out_x, out_y)
        new_x = np.empty_like(out_x)
        new_y = np.empty_like(out_y)
        for i in range(len(out_x)):
            mx, my, _, _ = corridor_geometry_from_map(
                grid, info, float(out_x[i]), float(out_y[i]), float(tangents[i]))
            new_x[i] = mx
            new_y[i] = my
        out_x, out_y = new_x, new_y

    return out_x, out_y


def map_corridor_bounds_for_polyline(
        grid: np.ndarray,
        info,
        xs: np.ndarray,
        ys: np.ndarray,
        margin: float = 1.0,
        max_cap: float = 5.0) -> tuple[np.ndarray, np.ndarray]:
    """Per-point lateral limits for min-curv (positive normal = left, negative = right)."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    n = len(xs)
    tangents = tangent_psi_closed(xs, ys)
    alpha_left = np.full(n, max_cap, dtype=float)
    alpha_right = np.full(n, max_cap, dtype=float)

    for i in range(n):
        _, _, left_d, right_d = corridor_geometry_from_map(
            grid, info, float(xs[i]), float(ys[i]), float(tangents[i]))
        alpha_left[i] = min(max_cap, max(0.0, left_d - margin))
        alpha_right[i] = min(max_cap, max(0.0, right_d - margin))

    return alpha_left, alpha_right


def dedupe_closed_polyline(
        xs: np.ndarray,
        ys: np.ndarray,
        alpha_left: np.ndarray | None = None,
        alpha_right: np.ndarray | None = None,
        min_d: float = 0.5) -> tuple[np.ndarray, ...]:
    """Remove nearly duplicate vertices; keep companion arrays aligned."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    n = len(xs)
    if n == 0:
        return (xs, ys) if alpha_left is None else (xs, ys, alpha_left, alpha_right)

    keep = [0]
    for i in range(1, n):
        if np.hypot(xs[i] - xs[keep[-1]], ys[i] - ys[keep[-1]]) >= min_d:
            keep.append(i)

    if len(keep) > 1 and np.hypot(xs[keep[0]] - xs[keep[-1]], ys[keep[0]] - ys[keep[-1]]) < min_d:
        keep = keep[:-1]

    idx = np.asarray(keep, dtype=int)
    if alpha_left is None:
        return xs[idx], ys[idx]
    return xs[idx], ys[idx], alpha_left[idx], alpha_right[idx]


def mean_corridor_half_width(
        grid: np.ndarray,
        info,
        xs: np.ndarray,
        ys: np.ndarray) -> float:
    """Average left/right clearance; fallback to TRACK_HALF_WIDTH."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    if len(xs) < 1:
        return TRACK_HALF_WIDTH

    tangents = tangent_psi_closed(xs, ys)
    widths: list[float] = []
    for i in range(len(xs)):
        _, _, left_d, right_d = corridor_geometry_from_map(
            grid, info, float(xs[i]), float(ys[i]), float(tangents[i]))
        widths.append(left_d + right_d)
    return float(np.mean(widths)) if widths else TRACK_HALF_WIDTH
