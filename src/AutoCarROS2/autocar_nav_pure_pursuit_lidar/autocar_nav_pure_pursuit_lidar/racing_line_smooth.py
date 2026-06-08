"""Polish a closed-loop racing line: Laplacian filter + uniform resample + safety checks.

Used as step 2 after ``racing_line_mincurv`` (min-curvature optimise, then smooth).
Ported from ``autocar_racing_line/scripts/smooth_racing_line.py``.
"""

from __future__ import annotations

import numpy as np


def _curvature(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    xp, xn = np.roll(xs, 1), np.roll(xs, -1)
    yp, yn = np.roll(ys, 1), np.roll(ys, -1)
    d1x, d1y = xs - xp, ys - yp
    d2x, d2y = xn - xs, yn - ys
    cross = d1x * d2y - d1y * d2x
    den = np.hypot(d1x, d1y) * np.hypot(d2x, d2y) * np.hypot(xn - xp, yn - yp)
    den[den < 1e-9] = 1.0
    return 2.0 * cross / den


def _min_turn_radius(xs: np.ndarray, ys: np.ndarray) -> float:
    k = np.abs(_curvature(xs, ys))
    return float(1.0 / max(k.max(), 1e-9))


def _curvature_energy(xs: np.ndarray, ys: np.ndarray) -> float:
    k = np.abs(_curvature(xs, ys))
    return float(np.sum(k ** 2))


def _resample_uniform(xs: np.ndarray, ys: np.ndarray, n: int) -> tuple[np.ndarray, np.ndarray]:
    if n < 3:
        return xs.copy(), ys.copy()
    pts = np.column_stack([xs, ys])
    pts = np.vstack([pts, pts[0]])
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    if total < 1e-6:
        return xs.copy(), ys.copy()
    s = np.linspace(0, total, n, endpoint=False)
    rx = np.interp(s, cum, pts[:, 0])
    ry = np.interp(s, cum, pts[:, 1])
    return rx, ry


def _laplacian_pass(xs: np.ndarray, ys: np.ndarray, alpha: float) -> tuple[np.ndarray, np.ndarray]:
    xp, xn = np.roll(xs, 1), np.roll(xs, -1)
    yp, yn = np.roll(ys, 1), np.roll(ys, -1)
    xs = xs + alpha * (0.5 * (xp + xn) - xs)
    ys = ys + alpha * (0.5 * (yp + yn) - ys)
    return xs, ys


def _clamp_deviation(
        xs: np.ndarray,
        ys: np.ndarray,
        ox: np.ndarray,
        oy: np.ndarray,
        max_dev: float) -> tuple[np.ndarray, np.ndarray]:
    dx, dy = xs - ox, ys - oy
    d = np.hypot(dx, dy)
    over = d > max_dev
    xs = xs.copy()
    ys = ys.copy()
    xs[over] = ox[over] + dx[over] / d[over] * max_dev
    ys[over] = oy[over] + dy[over] / d[over] * max_dev
    return xs, ys


def _max_centerline_offset(
        xs: np.ndarray,
        ys: np.ndarray,
        cx: np.ndarray,
        cy: np.ndarray) -> float:
    maxoff = 0.0
    for x, y in zip(xs, ys):
        maxoff = max(maxoff, float(np.min(np.hypot(cx - x, cy - y))))
    return maxoff


def _prepare_loop(xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    if len(xs) >= 3 and np.hypot(xs[0] - xs[-1], ys[0] - ys[-1]) < 1e-6:
        xs, ys = xs[:-1], ys[:-1]
    return xs, ys


def compute_smooth_racing_line(
        ox: np.ndarray,
        oy: np.ndarray,
        cx: np.ndarray,
        cy: np.ndarray,
        alpha: float = 0.5,
        iters: int = 15,
        max_dev: float = 2.5,
        half_width: float = 8.0,
        min_radius_improvement: float = 0.05,
        pre_iters: int = 8,
        pre_alpha: float = 0.4,
        coarse_points: int = 0,
        energy_improve_ratio: float = 0.05,
        require_radius_improvement: bool = False) -> tuple[np.ndarray, np.ndarray, dict]:
    """Smooth ``ox/oy`` and validate against centerline corridor ``cx/cy``.

    When ``require_radius_improvement`` is True (post min-curv polish), only accept
    if min turn radius improves. Otherwise also accept curvature-energy reduction.
    """
    ox, oy = _prepare_loop(ox, oy)
    cx, cy = _prepare_loop(cx, cy)
    n = len(ox)
    if n < 3:
        raise ValueError(f'path needs >= 3 points, got {n}')

    r0 = _min_turn_radius(ox, oy)
    e0 = _curvature_energy(ox, oy)

    work_x, work_y = ox.copy(), oy.copy()

    if pre_iters > 0:
        for _ in range(pre_iters):
            work_x, work_y = _laplacian_pass(work_x, work_y, pre_alpha)

    if coarse_points >= 3 and coarse_points < n:
        work_x, work_y = _resample_uniform(work_x, work_y, coarse_points)
        for _ in range(max(1, iters // 3)):
            work_x, work_y = _laplacian_pass(work_x, work_y, alpha)
        work_x, work_y = _resample_uniform(work_x, work_y, n)

    xs, ys = work_x.copy(), work_y.copy()
    for _ in range(iters):
        xs, ys = _laplacian_pass(xs, ys, alpha)
        xs, ys = _clamp_deviation(xs, ys, ox, oy, max_dev)

    xs, ys = _resample_uniform(xs, ys, n)
    r1 = _min_turn_radius(xs, ys)
    e1 = _curvature_energy(xs, ys)
    maxoff = _max_centerline_offset(xs, ys, cx, cy)
    within_track = maxoff < half_width
    radius_improved = r1 > r0 + min_radius_improvement
    energy_improved = e1 < e0 * (1.0 - energy_improve_ratio)
    if require_radius_improvement:
        improved = radius_improved
    else:
        improved = radius_improved or energy_improved

    info = {
        'n_points': n,
        'min_radius_before': r0,
        'min_radius_after': r1,
        'curvature_energy_before': e0,
        'curvature_energy_after': e1,
        'max_centerline_offset': maxoff,
        'within_track': within_track,
        'improved': improved,
        'radius_improved': radius_improved,
        'energy_improved': energy_improved,
    }

    if not improved or not within_track:
        info['fallback'] = 'original'
        return ox.copy(), oy.copy(), info

    return xs, ys, info
