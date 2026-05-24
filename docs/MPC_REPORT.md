# MPC Report — `autocar_nav_mpc` vs Stanley baseline

Frozen lap log: [`mpc_lap_times.csv`](mpc_lap_times.csv) (**lap 1 only**).  
Reference baseline: [`BASELINE.md`](BASELINE.md) / [`baseline_lap_times.csv`](baseline_lap_times.csv) (Stanley lap 1).

## Setup

| Item | MPC stack | Stanley baseline |
|------|-----------|------------------|
| World | [`race_circuit.world`](../src/AutoCarROS2/autocar_gazebo/worlds/race_circuit.world) | Same |
| Vehicle | Stock `autocar` (1580 kg, Ackermann) | Same |
| Tracker | Linear Frenet MPC in [`tracker.py`](../src/AutoCarROS2/autocar_nav_mpc/nodes/tracker.py) | Stanley in [`autocar_nav`](../src/AutoCarROS2/autocar_nav/nodes/tracker.py) |
| Cruise target | `cruise_velocity: 8.0` m/s ([`navigation_params.yaml`](../src/AutoCarROS2/autocar_nav_mpc/config/navigation_params.yaml)) | `CRUISE_VEL = 6.0` m/s |
| Local planner | Cubic spline + curvature speed limit + map avoidance | Same pattern, lower cruise |
| Launch | `ros2 launch launches race_mpc_launch.py` | `ros2 launch launches race_launch.py` |
| MPC horizon | 15 steps @ 50 Hz (0.02 s) | — |

MPC path tracker weights at time of recording: `q_ey=120`, `q_epsi=25`, `r_delta=0.08`, `r_ddelta=1.2`, `wheelbase=2.966` m.

## Recorded laps — lap 1 only (2 sessions)

| Session | Lap time | Avg speed | Peak speed | Distance |
|---------|----------|-----------|------------|----------|
| 2026-05-22T22-29-43 | 139.70 s | 4.72 m/s (17.0 km/h) | 6.02 m/s | 659.1 m |
| 2026-05-24T11-37-34 | **115.70 s** | **5.64 m/s (20.3 km/h)** | 5.87 m/s | 652.7 m |

**Reference MPC lap for comparison:** session `2026-05-24T11-37-34`, lap 1 — **115.70 s** (1 min 56 s).  
This is the faster of the two first laps and reflects the tuned configuration. Session 2026-05-22 lap 1 is included for context only.

## Comparison vs Stanley baseline (lap 1 vs lap 1)

Stanley reference (lap 1, 2026-05-20): **190.90 s**, avg **3.42 m/s**, peak **5.85 m/s**, distance **652.66 m**.

| Metric | Stanley lap 1 | MPC lap 1 (2026-05-24) | Δ (MPC − baseline) | Relative |
|--------|---------------|----------------------|--------------------|----------|
| Lap time | 190.90 s | **115.70 s** | **−75.20 s** | **−39.4%** |
| Average speed | 3.42 m/s | 5.64 m/s | +2.22 m/s | +64.9% |
| Peak speed | 5.85 m/s | 5.87 m/s | +0.03 m/s | +0.4% |
| Distance | 652.66 m | 652.72 m | +0.06 m | +0.0% |

Earlier session (2026-05-22, lap 1 only): **139.70 s** — still **51.2 s** faster than the Stanley baseline (−26.8%).

```
Lap time (s) — lap 1 only
200 |████████████████████████████████████████  Stanley 190.9 s
140 |████████████████████████████              MPC 2026-05-22 lap 1 139.7 s
120 |████████████████████                      PP roadmap target 120 s
116 |███████████████████                       MPC 2026-05-24 lap 1 115.7 s
 90 |██████████████                            Racing target 90 s
```

## Roadmap targets ([`BASELINE.md`](BASELINE.md))

Evaluated against MPC lap 1 (2026-05-24) vs Stanley lap 1.

| Target | Goal | MPC lap 1 | Status |
|--------|------|-----------|--------|
| Beat Stanley baseline | &lt; 190.90 s | 115.70 s | **Met** (both MPC lap 1s) |
| Pure Pursuit + speed profile | &lt; **120 s** | 115.70 s | **Met** (−4.3 s margin) |
| Racing line + tuned controller | &lt; **90 s** | 115.70 s | **Not met** (+25.7 s gap) |

## Analysis

**Lateral / path tracking.** Lap 1 distance (652.7 m) matches the Stanley baseline (652.66 m) and centerline length (~650 m). Gains are not from cutting corners.

**Longitudinal performance.** Average speed rises from 3.42 m/s (Stanley lap 1) to 5.64 m/s (MPC lap 1) while peak speed stays near 5.9 m/s. The MPC stack’s higher cruise target (8 m/s), curvature-based speed ramp, and predictive steering improve **mean speed** more than **peak speed**.

**Peak speed ceiling.** MPC lap 1 peak (5.87 m/s) is essentially the same as Stanley’s (5.85 m/s). Further gains toward 90 s will likely need higher straight-line speed or a shorter racing line.

**Session progression (lap 1 only).** Tuning between sessions shortened lap 1 by **24.0 s** (139.7 → 115.7 s), indicating controller / speed-profile improvements rather than warm-up effects on later laps.

## Summary

| | |
|--|--|
| **Verdict** | MPC lap 1 delivers a **~39% lap-time reduction** vs Stanley lap 1 on the same track (**115.7 s** vs **190.9 s**). |
| **vs baseline** | −75.2 s, +65% average speed, same path length. |
| **vs roadmap** | 120 s target **met** on lap 1; 90 s target **not yet** (+25.7 s). |
| **Next levers** | Racing line, higher cruise utilization on straights, vehicle longitudinal tuning. |

## Raw data & reproduction

```bash
colcon build --packages-select autocar_nav autocar_nav_mpc launches --symlink-install
source install/setup.bash
ros2 launch launches race_mpc_launch.py
python3 scripts/compare_lap_times.py --csv docs/mpc_lap_times.csv --detail
```

To refresh this report, update [`mpc_lap_times.csv`](mpc_lap_times.csv) with new **lap 1** rows only and revise the tables above.
