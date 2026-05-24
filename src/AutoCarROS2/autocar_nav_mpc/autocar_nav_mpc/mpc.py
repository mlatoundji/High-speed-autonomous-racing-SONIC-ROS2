"""Linear MPC path tracker (Frenet-frame bicycle model).

Discrete error dynamics (small-angle linearisation):
  e_y[k+1]   = e_y[k]   + v * e_psi[k] * dt
  e_psi[k+1] = e_psi[k] + (v / L) * delta[k] * dt - v * kappa_ref[k] * dt

Control u[k] = front-wheel steer angle delta[k].
"""

import numpy as np
from scipy.optimize import minimize

class LinearMPCController:
    """Condensed linear MPC with box and steering-rate constraints."""

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
        self.q_ey = float(q_ey)
        self.q_epsi = float(q_epsi)
        self.r_delta = float(r_delta)
        self.r_ddelta = float(r_ddelta)
        self.max_steer = float(max_steer)
        self.max_steer_rate = float(max_steer_rate)

        self._u_warm = np.zeros(self.N)
        self._delta_prev = 0.0

    def reset(self):
        self._u_warm[:] = 0.0
        self._delta_prev = 0.0

    def _state_matrices(self, speed):
        v = max(float(speed), 0.5)
        a = np.array([[1.0, v * self.dt], [0.0, 1.0]])
        b = np.array([[0.0], [v * self.dt / self.L]])
        return a, b

    def _predict(self, x0, u_seq, speed, kappa_seq):
        a, b = self._state_matrices(speed)
        x = np.asarray(x0, dtype=float).reshape(2)
        states = [x.copy()]
        for k in range(self.N):
            kappa = kappa_seq[k] if k < len(kappa_seq) else 0.0
            v = max(float(speed), 0.5)
            d = np.array([0.0, -v * kappa * self.dt])
            x = a @ x + (b * u_seq[k]).reshape(2) + d
            states.append(x.copy())
        return states

    def _cost(self, u_seq, x0, speed, kappa_seq):
        states = self._predict(x0, u_seq, speed, kappa_seq)
        cost = 0.0
        u_prev = self._delta_prev

        for k in range(self.N):
            xk = states[k + 1]
            cost += self.q_ey * xk[0] ** 2 + self.q_epsi * xk[1] ** 2
            cost += self.r_delta * u_seq[k] ** 2
            du = u_seq[k] - u_prev
            cost += self.r_ddelta * du ** 2
            u_prev = u_seq[k]

        return cost

    def _constraints(self):
        bounds = [(-self.max_steer, self.max_steer)] * self.N
        constraints = []

        if self.max_steer_rate > 0.0:
            max_du = self.max_steer_rate * self.dt

            def rate_at_k(k, u_seq):
                prev = self._delta_prev if k == 0 else u_seq[k - 1]
                return max_du - abs(u_seq[k] - prev)

            for k in range(self.N):
                constraints.append({
                    'type': 'ineq',
                    'fun': lambda u, kk=k: rate_at_k(kk, u),
                })

        return bounds, constraints

    def solve(self, e_y, e_psi, speed, kappa_seq):
        """Return the first steering command (rad)."""
        x0 = np.array([float(e_y), float(e_psi)])
        u0 = self._u_warm.copy()

        bounds, constraints = self._constraints()
        result = minimize(
            self._cost,
            u0,
            args=(x0, speed, kappa_seq),
            method='SLSQP',
            bounds=bounds,
            constraints=constraints,
            options={'maxiter': 80, 'ftol': 1e-4},
        )

        if result.success:
            u_opt = np.asarray(result.x, dtype=float)
        else:
            u_opt = u0

        self._u_warm = np.roll(u_opt, -1)
        self._u_warm[-1] = u_opt[-1]

        delta = float(np.clip(u_opt[0], -self.max_steer, self.max_steer))
        self._delta_prev = delta
        return delta

