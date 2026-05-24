"""Pure Pursuit path tracking helpers.

Coordinate convention (matches localisation / Stanley-era code):
  - State2D.pose.theta is aligned with the vehicle +y axis in odom.
  - Driving heading in x-y is theta + pi/2 (see vehicle_heading).
  - Control reference point: front axle, offset from CG by cg_to_front.
"""

import numpy as np

from autocar_nav_pure_pursuit.normalise_angle import normalise_angle


def front_axle_pose(x, y, yaw, cg_to_front):
    """Front-axle position in the odom frame (yaw aligned to +y)."""
    fx = x + cg_to_front * -np.sin(yaw)
    fy = y + cg_to_front * np.cos(yaw)
    return fx, fy


def vehicle_heading(yaw):
    """Heading angle in the odom x-y frame."""
    return yaw + np.pi * 0.5


def closest_path_index(fx, fy, cx, cy, start_idx=0, search_ahead=80):
    """Forward-only closest-point search for path tracking stability."""
    n = len(cx)
    if n == 0:
        return 0

    start_idx = int(np.clip(start_idx, 0, n - 1))
    end_idx = min(n, start_idx + search_ahead + 1)

    best_idx = start_idx
    best_d2 = (cx[start_idx] - fx) ** 2 + (cy[start_idx] - fy) ** 2

    for idx in range(start_idx + 1, end_idx):
        d2 = (cx[idx] - fx) ** 2 + (cy[idx] - fy) ** 2
        if d2 <= best_d2:
            best_d2 = d2
            best_idx = idx
        else:
            break

    return best_idx


def lookahead_point(fx, fy, cx, cy, start_idx, lookahead_dist):
    """Return (x, y, index) on the path at least lookahead_dist from (fx, fy)."""
    n = len(cx)
    if n == 0:
        return fx, fy, 0

    start_idx = int(np.clip(start_idx, 0, n - 1))
    la_idx = start_idx
    dist = np.hypot(cx[la_idx] - fx, cy[la_idx] - fy)

    while la_idx < n - 1 and dist < lookahead_dist:
        la_idx += 1
        dist = np.hypot(cx[la_idx] - fx, cy[la_idx] - fy)

    if la_idx == 0 or dist >= lookahead_dist:
        return cx[la_idx], cy[la_idx], la_idx

    x0, y0 = cx[la_idx - 1], cy[la_idx - 1]
    x1, y1 = cx[la_idx], cy[la_idx]
    seg_dx = x1 - x0
    seg_dy = y1 - y0
    seg_len = np.hypot(seg_dx, seg_dy)
    if seg_len < 1e-6:
        return x1, y1, la_idx

    d0 = np.hypot(x0 - fx, y0 - fy)
    d1 = np.hypot(x1 - fx, y1 - fy)
    if d1 <= d0:
        return x1, y1, la_idx

    t = np.clip((lookahead_dist - d0) / (d1 - d0 + 1e-9), 0.0, 1.0)
    tx = x0 + t * seg_dx
    ty = y0 + t * seg_dy
    return tx, ty, la_idx


def dynamic_lookahead(speed, gain, minimum, maximum):
    """Ld = clip(gain * v + Ld_min, Ld_min, Ld_max)."""
    return float(np.clip(gain * max(speed, 0.0) + minimum, minimum, maximum))


def pure_pursuit_steering(fx, fy, yaw, target_x, target_y, lookahead_dist, wheelbase):
    """Bicycle-model steer angle (rad): atan2(2·L·sin(alpha), Ld)."""
    psi = vehicle_heading(yaw)
    alpha = normalise_angle(np.arctan2(target_y - fy, target_x - fx) - psi)
    ld = max(lookahead_dist, 1e-3)
    return float(np.arctan2(2.0 * wheelbase * np.sin(alpha), ld))


def estimate_curvature(cx, cy, idx, window=3):
    """Discrete curvature estimate (1/m) from three path samples."""
    n = len(cx)
    if n < 3:
        return 0.0

    i0 = max(0, idx - window)
    i2 = min(n - 1, idx + window)
    i1 = int(np.clip(idx, 1, n - 2))

    x0, y0 = cx[i0], cy[i0]
    x1, y1 = cx[i1], cy[i1]
    x2, y2 = cx[i2], cy[i2]

    a = np.hypot(x1 - x0, y1 - y0)
    b = np.hypot(x2 - x1, y2 - y1)
    c = np.hypot(x2 - x0, y2 - y0)
    if a < 1e-6 or b < 1e-6 or c < 1e-6:
        return 0.0

    cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    area2 = abs(cross)
    return float(4.0 * area2 / (a * b * c + 1e-9))


def peak_curvature(ck, start_idx, end_idx, smooth_window=5):
    """Peak |kappa| with moving-average filter."""
    segment = [abs(k) for k in ck[start_idx:end_idx]]
    if not segment:
        return 0.0
    if smooth_window <= 1 or len(segment) < smooth_window:
        return float(max(segment))

    half = smooth_window // 2
    smoothed = []
    for i in range(len(segment)):
        window = segment[max(0, i - half):min(len(segment), i + half + 1)]
        smoothed.append(sum(window) / len(window))
    return float(max(smoothed))


def curvature_speed_limit(kappa, max_lateral_accel, min_kappa=0.02):
    """v_max = sqrt(a_lat / |kappa|). min_kappa caps top speed on straights."""
    return float(np.sqrt(max_lateral_accel / max(abs(kappa), min_kappa)))


def apply_speed_ramp(current, target, dt, accel_rate, decel_rate):
    """Asymmetric first-order ramp: faster accel than decel for lap-time."""
    if target >= current:
        alpha = min(accel_rate * dt, 1.0)
    else:
        alpha = min(decel_rate * dt, 1.0)
    return float(current + alpha * (target - current))


def limit_steering_rate(steer, prev_steer, dt, rate_limit):
    """Clamp steering change per cycle for stable high-speed tracking."""
    if rate_limit <= 0.0 or prev_steer is None:
        return steer
    max_delta = rate_limit * dt
    delta = np.clip(steer - prev_steer, -max_delta, max_delta)
    return float(prev_steer + delta)
