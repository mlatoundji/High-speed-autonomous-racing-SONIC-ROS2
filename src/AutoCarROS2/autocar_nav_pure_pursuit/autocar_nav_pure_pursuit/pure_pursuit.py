"""Pure Pursuit path tracking helpers.

Coordinate convention (matches localisation / Stanley-era code):
  - State2D.pose.theta is aligned with the vehicle +y axis in odom.
  - Forward direction in x-y is (-sin(theta), +cos(theta)).
  - Control reference for Pure Pursuit: rear axle (classic bicycle model).
  - Lateral error for metrics matches Stanley: front axle projection.
"""

import numpy as np

from autocar_nav_pure_pursuit.normalise_angle import normalise_angle


def forward_vector(yaw):
    """Unit forward vector in odom (+y-aligned body yaw convention)."""
    return -np.sin(yaw), np.cos(yaw)


def right_vector(yaw):
    """Unit right vector in odom."""
    return np.cos(yaw), np.sin(yaw)


def front_axle_pose(x, y, yaw, cg_to_front):
    """Front-axle position in the odom frame."""
    fwd_x, fwd_y = forward_vector(yaw)
    return x + cg_to_front * fwd_x, y + cg_to_front * fwd_y


def rear_axle_pose(x, y, yaw, cg_to_rear):
    """Rear-axle position in the odom frame."""
    fwd_x, fwd_y = forward_vector(yaw)
    return x - cg_to_rear * fwd_x, y - cg_to_rear * fwd_y


def closest_path_index(px, py, cx, cy, start_idx=0, search_ahead=80):
    """Forward-only closest-point search — avoids index jumps on closed paths."""
    n = len(cx)
    if n == 0:
        return 0

    start_idx = int(np.clip(start_idx, 0, n - 1))
    end_idx = min(n, start_idx + search_ahead + 1)

    best_idx = start_idx
    best_d2 = (cx[start_idx] - px) ** 2 + (cy[start_idx] - py) ** 2

    for idx in range(start_idx + 1, end_idx):
        d2 = (cx[idx] - px) ** 2 + (cy[idx] - py) ** 2
        if d2 <= best_d2:
            best_d2 = d2
            best_idx = idx
        else:
            break

    return best_idx


def initial_path_index(px, py, cx, cy, search_ahead=80):
    """Anchor on a freshly published sliding-window path (forward from index 0)."""
    return closest_path_index(px, py, cx, cy, start_idx=0, search_ahead=search_ahead)


def vehicle_heading(yaw):
    """Driving heading in the odom x-y frame."""
    return yaw + np.pi * 0.5


def path_tangent_heading(path_theta):
    """Driving heading from Path2D.theta (cubic spline arctan2(dy, dx))."""
    return float(path_theta)


def frenet_errors(fx, fy, yaw, cx, cy, cyaw, idx):
    """Signed lateral and heading error at the front axle."""
    idx = int(np.clip(idx, 0, len(cx) - 1))
    psi = vehicle_heading(yaw)
    psi_ref = path_tangent_heading(cyaw[idx])
    dx = fx - cx[idx]
    dy = fy - cy[idx]
    path_dir = np.array([np.cos(psi_ref), np.sin(psi_ref)])
    normal = np.array([-path_dir[1], path_dir[0]])
    e_y = float(np.dot(np.array([dx, dy]), normal))
    e_psi = normalise_angle(psi - psi_ref)
    return e_y, e_psi


def speed_scale_from_errors(e_y, e_psi, lateral_soft=3.0, heading_soft=0.5):
    """Reduce speed when far from the path (0..1)."""
    lat = float(np.exp(-abs(e_y) / max(lateral_soft, 0.5)))
    head = float(np.exp(-abs(e_psi) / max(heading_soft, 0.1)))
    return lat * head


def lookahead_curvature_scale(kappa_peak, kappa_soft=0.08):
    """Shorten lookahead in sharp bends (0.45..1)."""
    return float(1.0 / (1.0 + (abs(kappa_peak) / max(kappa_soft, 1e-3)) ** 2))


def dynamic_lookahead(speed, gain, minimum, maximum):
    """Ld = clip(gain * v + minimum, minimum, maximum)."""
    return float(np.clip(gain * max(speed, 0.0) + minimum, minimum, maximum))


def find_lookahead_point(rx, ry, cx, cy, start_idx, lookahead_dist):
    """Arc-length lookahead from a forward anchor, with segment interpolation.

    Returns (index, lx, ly) or (None, None, None) when the path is empty.
    """
    n = len(cx)
    if n == 0:
        return None, None, None

    idx = int(np.clip(start_idx, 0, n - 1))
    acc = float(np.hypot(cx[idx] - rx, cy[idx] - ry))

    if acc >= lookahead_dist:
        return idx, float(cx[idx]), float(cy[idx])

    while idx + 1 < n:
        seg = float(np.hypot(cx[idx + 1] - cx[idx], cy[idx + 1] - cy[idx]))
        if seg < 1e-6:
            idx += 1
            continue
        if acc + seg >= lookahead_dist:
            t = float(np.clip((lookahead_dist - acc) / seg, 0.0, 1.0))
            lx = cx[idx] + t * (cx[idx + 1] - cx[idx])
            ly = cy[idx] + t * (cy[idx + 1] - cy[idx])
            return idx + 1, lx, ly
        acc += seg
        idx += 1

    return n - 1, float(cx[-1]), float(cy[-1])


def pure_pursuit_steering(rx, ry, yaw, target_x, target_y, wheelbase):
    """Bicycle-model steer (rad) from rear axle to lookahead, simulator sign."""
    dx = target_x - rx
    dy = target_y - ry
    fwd_x, fwd_y = forward_vector(yaw)
    rgt_x, rgt_y = right_vector(yaw)
    longitudinal = dx * fwd_x + dy * fwd_y
    lateral = dx * rgt_x + dy * rgt_y
    alpha = float(np.arctan2(lateral, longitudinal))
    chord = max(float(np.hypot(dx, dy)), 1e-3)
    return -float(np.arctan2(2.0 * wheelbase * np.sin(alpha), chord))


def lateral_error_front_axle(fx, fy, yaw, cx, cy, path_idx):
    """Signed cross-track at the front axle (Stanley-compatible)."""
    if not cx:
        return 0.0
    idx = int(np.clip(path_idx, 0, len(cx) - 1))
    dx = fx - cx[idx]
    dy = fy - cy[idx]
    rx, ry = right_vector(yaw)
    return float(dx * rx + dy * ry)


def smooth_steering(steer, prev, alpha):
    """Exponential low-pass on steering (alpha in (0, 1], 1 = no filter)."""
    if prev is None or alpha >= 1.0:
        return steer
    return float(prev + alpha * (steer - prev))


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
