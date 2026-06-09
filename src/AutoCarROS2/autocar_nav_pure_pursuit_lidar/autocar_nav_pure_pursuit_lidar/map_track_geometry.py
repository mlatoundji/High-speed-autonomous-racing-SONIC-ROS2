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


def _near_occupied(grid: np.ndarray, info, x: float, y: float, margin_cells: int) -> bool:
    """Return True if any occupied cell (>50) is within margin_cells of (x, y)."""
    col_f = (x - info.origin.position.x) / info.resolution
    row_f = (y - info.origin.position.y) / info.resolution
    col = int(col_f)
    row = int(row_f)
    h, w = grid.shape
    if col < 0 or col >= w or row < 0 or row >= h:
        return True  # outside grid boundary → treat as wall
    for dc in range(-margin_cells, margin_cells + 1):
        for dr in range(-margin_cells, margin_cells + 1):
            c, r = col + dc, row + dr
            if 0 <= c < w and 0 <= r < h:
                if int(grid[r, c]) > 50:
                    return True
    return False


def snap_racing_line_to_free_space(
        xs: np.ndarray,
        ys: np.ndarray,
        cx: np.ndarray,
        cy: np.ndarray,
        grid: np.ndarray,
        info,
        wall_margin: float = 1.0,
        max_iter: int = 25) -> tuple[np.ndarray, np.ndarray]:
    """Pull any racing line point too close to an occupied cell back toward the centerline.

    This guards against mincurv pushing the line through a wall that the SLAM map
    missed (unknown cells where walls actually exist). After each move step the point
    is re-checked; iteration stops as soon as the clearance criterion is satisfied.
    """
    xs = xs.copy()
    ys = ys.copy()
    margin_cells = max(1, int(math.ceil(wall_margin / info.resolution)))

    for i in range(len(xs)):
        for _ in range(max_iter):
            if not _near_occupied(grid, info, float(xs[i]), float(ys[i]), margin_cells):
                break
            # Move 25 % toward the nearest centerline point each iteration.
            dists = np.hypot(cx - xs[i], cy - ys[i])
            j = int(np.argmin(dists))
            xs[i] = xs[i] * 0.75 + cx[j] * 0.25
            ys[i] = ys[i] * 0.75 + cy[j] * 0.25

    # Remove any collapsed/coincident points the snap may have introduced so
    # that consecutive nearly-identical vertices cannot create fold-back artefacts.
    xs, ys = dedupe_closed_polyline(xs, ys, min_d=0.2)
    return xs, ys


def remove_fold_backs(
        xs: np.ndarray,
        ys: np.ndarray,
        max_cos: float = -0.3) -> tuple[np.ndarray, np.ndarray]:
    """Iteratively remove points where the path reverses direction by more than ~107°.

    After Laplacian smoothing, legitimate F1 hairpins produce at most ~40° per-segment
    angle change in the racing line (radius ≥ 5 m, step ≈ 3.5 m).  Fold-back
    artefacts — from loop-seam overshoot or snap-induced collapses — appear as
    single-step reversals of 120°+; max_cos = -0.3 (≈ 107°) safely separates them.
    """
    xs = np.asarray(xs, dtype=float).copy()
    ys = np.asarray(ys, dtype=float).copy()

    changed = True
    while changed and len(xs) >= 4:
        changed = False
        n = len(xs)
        bad: list[int] = []
        for i in range(n):
            p = (i - 1) % n
            q = (i + 1) % n
            d1x = xs[i] - xs[p]; d1y = ys[i] - ys[p]
            d2x = xs[q] - xs[i]; d2y = ys[q] - ys[i]
            l1 = math.hypot(d1x, d1y)
            l2 = math.hypot(d2x, d2y)
            if l1 < 0.1 or l2 < 0.1:
                bad.append(i)  # degenerate (nearly-coincident) point
            elif (d1x * d2x + d1y * d2y) / (l1 * l2) < max_cos:
                bad.append(i)  # direction reversal > 107°
        if bad:
            mask = np.ones(n, dtype=bool)
            mask[bad] = False
            xs = xs[mask]
            ys = ys[mask]
            changed = True

    return xs, ys
