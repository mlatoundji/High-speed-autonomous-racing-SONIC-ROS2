"""Path-tracking helpers shared by the MPC stack.

Coordinate convention (matches localisation / Pure Pursuit):
  - State2D.pose.theta is aligned with the vehicle +y axis in odom.
  - Driving heading in x-y is theta + pi/2 (see vehicle_heading).
  - Control reference point: front axle, offset from CG by cg_to_front.
"""

import numpy as np

from autocar_nav_mpc.normalise_angle import normalise_angle


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


def frenet_errors(fx, fy, yaw, cx, cy, cyaw, idx):
    """Signed lateral and heading error at the front axle w.r.t. path index."""
    idx = int(np.clip(idx, 0, len(cx) - 1))
    psi = vehicle_heading(yaw)
    psi_ref = vehicle_heading(cyaw[idx])

    dx = fx - cx[idx]
    dy = fy - cy[idx]

    path_dir = np.array([np.cos(psi_ref), np.sin(psi_ref)])
    normal = np.array([-path_dir[1], path_dir[0]])
    e_y = float(np.dot(np.array([dx, dy]), normal))
    e_psi = normalise_angle(psi - psi_ref)
    return e_y, e_psi


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


def build_curvature_profile(cx, cy):
    """Per-point curvature array from path geometry."""
    return [estimate_curvature(cx, cy, i) for i in range(len(cx))]


def curvature_horizon(ck, start_idx, horizon):
    """Reference curvature sequence for the MPC horizon."""
    if not ck:
        return [0.0] * horizon

    n = len(ck)
    start_idx = int(np.clip(start_idx, 0, n - 1))
    seq = []
    for k in range(horizon):
        idx = min(start_idx + k, n - 1)
        seq.append(float(ck[idx]))
    return seq


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
