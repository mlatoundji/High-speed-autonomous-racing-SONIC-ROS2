"""Fast Frenet bicycle controller for path tracking.

Bicycle feedforward (curvature) + speed-softened error feedback, tuned to
avoid full steering lock at startup when lateral/heading errors are large.
"""

import numpy as np

GAZEBO_MAX_STEER = 0.85


class LinearMPCController:
    """Bicycle feedforward + softened error feedback steering."""

    def __init__(
        self,
        horizon,
        dt,
        wheelbase,
        q_ey,
        q_epsi,
        r_delta,
        r_ddelta,
        max_steer,
        max_steer_rate,
    ):
        self.N = int(horizon)
        self.dt = float(dt)
        self.L = float(wheelbase)
        self.max_steer = min(float(max_steer), GAZEBO_MAX_STEER)
        self.max_steer_rate = float(max_steer_rate)

        # Gentler than sqrt(q) — avoids flip on large startup e_y.
        self.k_ey = min(np.sqrt(float(q_ey)) * 0.04, 0.55)
        self.k_epsi = min(np.sqrt(float(q_epsi)) * 0.12, 0.45)
        self.softening = 2.5
        self._delta_prev = 0.0

    def reset(self):
        self._delta_prev = 0.0

    def solve(self, e_y, e_psi, speed, kappa_seq):
        """Return front-wheel steer angle (rad)."""
        v = max(float(speed), 0.5)
        kappa = float(kappa_seq[0]) if kappa_seq else 0.0
        delta_ff = float(np.arctan(self.L * kappa))

        denom = self.softening + v
        delta_fb = (
            -self.k_ey * float(e_y) / denom
            -self.k_epsi * float(e_psi) / denom
        )
        delta = float(np.clip(
            delta_ff + delta_fb,
            -self.max_steer,
            self.max_steer,
        ))
        self._delta_prev = delta
        return delta
