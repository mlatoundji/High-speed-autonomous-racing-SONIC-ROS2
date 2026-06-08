"""Minimum-curvature racing line: centerline + alpha * normal within a corridor.

Ported from ``autocar_racing_line/scripts/generate_racing_line_mincurv.py``.
"""

from __future__ import annotations

import numpy as np


def _prepare_loop(xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    if len(xs) >= 3 and np.hypot(xs[0] - xs[-1], ys[0] - ys[-1]) < 1e-6:
        xs, ys = xs[:-1], ys[:-1]
    return xs, ys


def _dedupe(cx: np.ndarray, cy: np.ndarray, min_d: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
    kx, ky = [float(cx[0])], [float(cy[0])]
    for x, y in zip(cx[1:], cy[1:]):
        if np.hypot(x - kx[-1], y - ky[-1]) >= min_d:
            kx.append(float(x))
            ky.append(float(y))
    if len(kx) > 1 and np.hypot(kx[0] - kx[-1], ky[0] - ky[-1]) < min_d:
        kx, ky = kx[:-1], ky[:-1]
    return np.asarray(kx, dtype=float), np.asarray(ky, dtype=float)


def _normals(xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    dx = np.roll(xs, -1) - np.roll(xs, 1)
    dy = np.roll(ys, -1) - np.roll(ys, 1)
    n = np.hypot(dx, dy)
    n[n < 1e-9] = 1.0
    return -dy / n, dx / n


def _peak_curvature(xs: np.ndarray, ys: np.ndarray) -> float:
    xp, xn = np.roll(xs, 1), np.roll(xs, -1)
    yp, yn = np.roll(ys, 1), np.roll(ys, -1)
    d1x, d1y = xs - xp, ys - yp
    d2x, d2y = xn - xs, yn - ys
    cross = d1x * d2y - d1y * d2x
    den = np.hypot(d1x, d1y) * np.hypot(d2x, d2y) * np.hypot(xn - xp, yn - yp)
    den[den < 1e-9] = 1.0
    return float(np.max(np.abs(2.0 * cross / den)))


def compute_mincurv_racing_line(
        cx: np.ndarray,
        cy: np.ndarray,
        max_offset: float = 5.0,
        iters: int = 8000,
        dedupe_min_dist: float = 0.5,
        alpha_max_left: np.ndarray | None = None,
        alpha_max_right: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, dict]:
    """Optimise lateral offset alpha to minimise curvature within a corridor.

    When ``alpha_max_left/right`` are given (from map boundary rays), each point
    can use a different asymmetric corridor instead of a fixed ±max_offset.
    """
    cx, cy = _prepare_loop(cx, cy)
    n0 = len(cx)
    if n0 < 3:
        raise ValueError(f'centerline needs >= 3 points, got {n0}')

    if alpha_max_left is not None and alpha_max_right is not None:
        from autocar_nav_pure_pursuit_lidar.map_track_geometry import dedupe_closed_polyline
        cx, cy, alpha_max_left, alpha_max_right = dedupe_closed_polyline(
            cx, cy, alpha_max_left, alpha_max_right, min_d=dedupe_min_dist)
    else:
        cx, cy = _dedupe(cx, cy, min_d=dedupe_min_dist)
        alpha_max_left = None
        alpha_max_right = None

    n = len(cx)
    if n < 3:
        raise ValueError(f'centerline needs >= 3 unique points after dedupe, got {n}')

    nx, ny = _normals(cx, cy)

    eye = np.eye(n)
    d_mat = -2 * eye + np.roll(eye, 1, axis=1) + np.roll(eye, -1, axis=1)
    ax = d_mat @ np.diag(nx)
    ay = d_mat @ np.diag(ny)
    bx = d_mat @ cx
    by = d_mat @ cy
    hess = 2 * (ax.T @ ax + ay.T @ ay)
    lin = 2 * (ax.T @ bx + ay.T @ by)

    vec = np.ones(n)
    for _ in range(60):
        vec = hess @ vec
        norm = np.linalg.norm(vec)
        if norm < 1e-12:
            break
        vec /= norm
    lam = float(vec @ (hess @ vec))
    step = 0.9 / max(lam, 1e-9)

    alpha = np.zeros(n)

    def objective(a: np.ndarray) -> float:
        return float(np.sum((ax @ a + bx) ** 2) + np.sum((ay @ a + by) ** 2))

    if alpha_max_left is not None and alpha_max_right is not None:
        lo = -np.asarray(alpha_max_right, dtype=float)
        hi = np.asarray(alpha_max_left, dtype=float)
        map_corridor = True
    else:
        lo = np.full(n, -max_offset, dtype=float)
        hi = np.full(n, max_offset, dtype=float)
        map_corridor = False

    j0 = objective(alpha)
    for _ in range(iters):
        grad = hess @ alpha + lin
        alpha = np.clip(alpha - step * grad, lo, hi)
    j1 = objective(alpha)

    rx = cx + alpha * nx
    ry = cy + alpha * ny
    k_center = _peak_curvature(cx, cy)
    k_racing = _peak_curvature(rx, ry)
    r_center = 1.0 / max(k_center, 1e-9)
    r_racing = 1.0 / max(k_racing, 1e-9)
    improved = k_racing <= k_center

    info = {
        'n_points': n,
        'n_deduped': n0 - n,
        'curvature_energy_before': j0,
        'curvature_energy_after': j1,
        'peak_curvature_center': k_center,
        'peak_curvature_racing': k_racing,
        'min_radius_center': r_center,
        'min_radius_racing': r_racing,
        'alpha_min': float(alpha.min()),
        'alpha_max': float(alpha.max()),
        'alpha_mean_abs': float(np.mean(np.abs(alpha))),
        'max_offset': max_offset,
        'map_corridor': map_corridor,
        'alpha_bound_left_mean': float(np.mean(hi)),
        'alpha_bound_right_mean': float(np.mean(-lo)),
        'improved': improved,
    }

    if not improved:
        info['fallback'] = 'centerline'
        return cx.copy(), cy.copy(), info

    return rx, ry, info
