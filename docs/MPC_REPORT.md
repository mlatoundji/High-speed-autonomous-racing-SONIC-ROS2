# MPC Report — `autocar_nav_mpc` vs Stanley baseline

Frozen lap log: [`mpc_lap_times.csv`](mpc_lap_times.csv) (**lap 1 only** for baseline comparison).  
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

## Recorded laps — session 2026-05-24T11-37-34

| Lap | Lap time | Avg speed | Peak speed | Distance |
|-----|----------|-----------|------------|----------|
| 1 | **115.70 s** | 5.64 m/s (20.3 km/h) | 5.87 m/s | 652.7 m |
| 2 | 113.20 s | 5.76 m/s (20.7 km/h) | 5.89 m/s | 651.9 m |
| 3 | 114.60 s | 5.69 m/s (20.5 km/h) | 5.91 m/s | 652.2 m |

**Reference for baseline comparison:** lap 1 — **115.70 s** (1 min 56 s).  
Laps 2–3 are listed for repeatability only; they are not used in the tables below.

Across all three laps: mean **114.50 s**, spread **2.50 s** (113.2–115.7 s). Lap 2 is fastest; lap 1 is slowest by 2.5 s.

## Comparison vs Stanley baseline (lap 1 vs lap 1)

Stanley reference (lap 1, 2026-05-20): **190.90 s**, avg **3.42 m/s**, peak **5.85 m/s**, distance **652.66 m**.

| Metric | Stanley lap 1 | MPC lap 1 | Δ (MPC − baseline) | Relative |
|--------|---------------|-----------|--------------------|----------|
| Lap time | 190.90 s | **115.70 s** | **−75.20 s** | **−39.4%** |
| Average speed | 3.42 m/s | 5.64 m/s | +2.22 m/s | +64.9% |
| Peak speed | 5.85 m/s | 5.87 m/s | +0.03 m/s | +0.4% |
| Distance | 652.66 m | 652.72 m | +0.06 m | +0.0% |

```
Lap time (s) — lap 1 vs baseline
200 |████████████████████████████████████████  Stanley 190.9 s
120 |████████████████████                      PP roadmap target 120 s
116 |███████████████████                       MPC lap 1 115.7 s
 90 |██████████████                            Racing target 90 s
```

## Roadmap targets ([`BASELINE.md`](BASELINE.md))

Evaluated against MPC lap 1 vs Stanley lap 1.

| Target | Goal | MPC lap 1 | Status |
|--------|------|-----------|--------|
| Beat Stanley baseline | &lt; 190.90 s | 115.70 s | **Met** |
| Pure Pursuit + speed profile | &lt; **120 s** | 115.70 s | **Met** (−4.3 s margin) |
| Racing line + tuned controller | &lt; **90 s** | 115.70 s | **Not met** (+25.7 s gap) |

If lap 2 were used instead (not the report convention): 113.20 s would still beat the 120 s target by 6.8 s, but remain 23.2 s above the 90 s goal.

## Analysis

**Lateral / path tracking.** Lap 1 distance (652.7 m) matches the Stanley baseline (652.66 m) and centerline length (~650 m). All three laps stay within ~0.8 m of the same path length. Gains are not from cutting corners.

**Longitudinal performance.** Average speed on lap 1 rises from 3.42 m/s (Stanley) to 5.64 m/s (MPC) while peak speed stays near 5.9 m/s. The MPC stack’s higher cruise target (8 m/s), curvature-based speed ramp, and predictive steering improve **mean speed** more than **peak speed**.

**Peak speed ceiling.** MPC lap 1 peak (5.87 m/s) is essentially the same as Stanley’s (5.85 m/s). Lap 3 reaches 5.91 m/s — still only ~0.06 m/s above baseline. Further gains toward 90 s will likely need higher straight-line speed or a shorter racing line.

**Repeatability (same session).** Three consecutive laps cluster within 2.5 s (113.2–115.7 s), suggesting stable control after the first crossing. Lap 1 is slightly slower than laps 2–3, consistent with a short warm-up on the opening lap.

## Summary

| | |
|--|--|
| **Verdict** | MPC lap 1 delivers a **~39% lap-time reduction** vs Stanley lap 1 (**115.7 s** vs **190.9 s**). |
| **vs baseline** | −75.2 s, +65% average speed, same path length. |
| **vs roadmap** | 120 s target **met** on lap 1; 90 s target **not yet** (+25.7 s). |
| **Session note** | 3 laps logged; mean 114.5 s, best 113.2 s (lap 2). |
| **Next levers** | Racing line, higher cruise utilization on straights, vehicle longitudinal tuning. |

## Raw data & reproduction

```bash
colcon build --packages-select autocar_nav autocar_nav_mpc launches --symlink-install
source install/setup.bash
ros2 launch launches race_mpc_launch.py
python3 scripts/compare_lap_times.py --csv docs/mpc_lap_times.csv --detail
```

To refresh this report, update [`mpc_lap_times.csv`](mpc_lap_times.csv) and revise the tables above (baseline comparison uses **lap 1** only).
