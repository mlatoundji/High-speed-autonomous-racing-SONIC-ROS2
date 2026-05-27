# `autocar_nav_pure_pursuit`

Pure Pursuit race stack for AutoCarROS2: cubic-spline local planning, curvature-based speed profiling, and a dedicated path tracker. This package is the production successor to the **`need_for_speed`** branch controller in `autocar_nav/nodes/pure_pursuit.py` (single-file node, `controller:=` launch switch).

Measured performance (2026-05-25, Gazebo `race_circuit.world`): **~108 s** centerline (warmed laps), **~92.5 s** racing line best lap — see [`docs/REPORT_PURE_PURSUIT.md`](../../../docs/REPORT_PURE_PURSUIT.md).

---

## Layout vs `need_for_speed`

| Aspect | `need_for_speed` | This package |
|--------|------------------|--------------|
| Location | `autocar_nav/nodes/pure_pursuit.py` | `autocar_nav_pure_pursuit/` (own ROS package) |
| Launch | `race_launch.py` + `controller:=pure_pursuit` | `race_pure_pursuit_launch.py` |
| Planner / tracker | Shared `autocar_nav` local planner @ 6 m/s fixed cruise | Dedicated `localplanner.py` + curvature speed cap @ 8 m/s |
| Library code | All in one node class | `pure_pursuit.py` helpers + `nodes/tracker.py` |
| Waypoints | `autocar_nav/data/*.csv` | `autocar_racing_line/data/` (`line:=centerline\|racing`) |

Topic contract is unchanged for planning inputs: `/autocar/state2D`, `/autocar/path`, `/autocar/target_velocity` in. The tracker now publishes autonomous commands on `/autocar/auto_cmd_vel`; `autocar_nav/control_manager.py` arbitrates manual/semi/auto modes and is the only node publishing `/autocar/cmd_vel`. `/autocar/lateral_error` and `/autocar/lateral_ref` remain comparable to Stanley.

---

## Core algorithm kept from `need_for_speed`

These choices came from the first clean PP laps on the branch (~195 s centerline) and are **intentionally preserved**:

1. **Measured-speed lookahead** — `Ld = clip(gain × v + Ld_min, Ld_min, Ld_max)` uses **current** speed `v`, not the planner target. The loop self-damps when the car is slow; target speed does not inflate lookahead on straights.

2. **Rear-axle pursuit, front-axle metrics** — Steering uses the rear axle (bicycle model). `/autocar/lateral_error` is the signed front-axle cross-track projection (same convention as Stanley `tracker.py`) so benchmark CSVs stay aligned.

3. **Steering sign** — `δ = -atan2(2 L sin(α), chord)` matches the simulator / Stanley sign convention (without the negation the car leaves on the outside of turn one; verified 2026-05-23 on the branch).

4. **True chord length** — Denominator uses `hypot(Δx, Δy)` to the lookahead point, not the nominal `Ld` distance.

Reference implementation on the branch: `git show need_for_speed:src/AutoCarROS2/autocar_nav/nodes/pure_pursuit.py`.

---

## Important optimizations (tracker + `pure_pursuit.py`)

### 1. Forward-only path indexing (closed loop stability)

**Branch:** `_find_lookahead_point` used `argmin` distance over the **entire** sliding path each cycle — on a ~650 m closed spline this can snap to the wrong lap segment after a few corners.

**Here:** `closest_path_index()` searches only **forward** from the previous index (`search_ahead` window, monotonic break). `anchor_path_index()` re-runs a global nearest search only if the forward anchor is stale (`> reanchor_dist` m). `path_cb` re-anchors when the local planner publishes a new window without resetting progress blindly.

### 2. Arc-length lookahead with segment interpolation

**Branch:** Walked discrete path vertices until arc length ≥ `Ld`, then used that **vertex** as the lookahead.

**Here:** `find_lookahead_point()` accumulates segment length and **interpolates** along the segment where `Ld` falls. The pursuit point is smoother and less sensitive to spline sampling density (`ds = 1/f_planner`).

### 3. Bounded lookahead and path-update hygiene

- `lookahead_max` caps `Ld` at high speed (default 6 m with gain 0.4 @ 8 m/s → 4.7 m before cap).
- `path_cb` skips republication when first/last points are unchanged (same sliding window).
- `prev_steer` cleared when the anchored path index jumps ≥ 4 m (large planner refresh).

### 4. Longitudinal guards (racing line / spawn)

- `speed_scale_from_errors(e_y, e_psi)` — exponential soft cap on `cmd_vel` when far from the path or misaligned (critical on **racing** line at spawn).
- `startup_ramp_s` — blends command speed in over ~2 s to avoid a step on first control ticks.
- `velocity_gain` — scalar on planner target (default 1.0).
- Optional `steer_smoothing` and `steering_rate_limit` (defaults off / pass-through).

Helpers `lookahead_curvature_scale()` and `estimate_curvature()` exist in `pure_pursuit.py` for future tightening in bends; the live tracker uses measured-speed `Ld` only.

---

## Local planner optimizations (`nodes/localplanner.py`)

Relative to the branch’s shared `autocar_nav` planner (cubic spline + BOF offsets, but **fixed 6 m/s** cruise):

| Feature | `need_for_speed` planner | This package |
|---------|-------------------------|--------------|
| Cruise / avoid speed | 6 m/s; `AVOID_VEL` on lateral offset | **8 m/s**; `cruise_velocity == avoid_velocity` (no spike when path shifts) |
| Corner speed | Fixed target | `v ≤ √(a_lat_max / κ)` from path curvature ahead |
| Speed dynamics | Instant `target_vel` | `apply_speed_ramp()` — asymmetric accel/decel on published target |
| Curvature lookahead | — | `peak_curvature()` over ~140 samples, smoothed window |
| Blocked path | Crawl @ 0.5× avoid | Crawl only if **all** offsets blocked; else steady cruise |

The steady cruise + geometric offset choice follows [`docs/pp_racing_line_2026-05-23/LESSONS.md`](../../../docs/pp_racing_line_2026-05-23/LESSONS.md): PP faithfully tracks planner shifts; varying `target_vel` on every BOF false positive was destabilising.

---

## Global planner (`nodes/globalplanner.py`)

- **Closed-loop waypoint index:** `closest_waypoint_index_closed()` — forward search with wrap on the lap CSV (avoids jumping to the wrong side of the track).
- **Goals dedup:** `_publish_key` skips identical goal republication (less local-planner churn).
- **Racing line data:** waypoints from `autocar_racing_line` via `waypoints_file` launch arg (`centerline` / `racing`).
- **Horizon:** `waypoints_ahead: 5` (branch Stanley stack often used 3).

---

## Parameters (defaults)

See [`config/navigation_params.yaml`](config/navigation_params.yaml). Tracker highlights:

```yaml
lookahead_gain: 0.4    # need_for_speed tuning: Ld ≈ 4.7 m @ 8 m/s
lookahead_min: 1.5
lookahead_max: 6.0
closest_search_ahead: 120
lateral_soft: 4.0      # speed_scale_from_errors
heading_soft: 0.6
startup_ramp_s: 2.0
```

Local planner: `cruise_velocity: 8.0`, `max_lateral_accel: 5.5`, `min_curvature: 0.012`.

---

## Run

From repo root after `colcon build`:

```bash
ros2 launch launches race_pure_pursuit_launch.py line:=centerline
ros2 launch launches race_pure_pursuit_launch.py line:=racing
```

Batch (vs Stanley): `python3 scripts/benchmark.py --config scripts/configs/r1_pp_vs_stanley.yaml`

---

## Data

Example sessions under [`results/`](../../../results/):

- `pure_pursuit_2026-05-25T16-28-02/` — centerline  
- `pure_pursuit_racing_2026-05-25T16-34-12/` — racing line  

---

## Further reading

- [`docs/REPORT_PURE_PURSUIT.md`](../../../docs/REPORT_PURE_PURSUIT.md) — lap tables and cross-stack comparison  
- [`docs/pp_racing_line_2026-05-23/LESSONS.md`](../../../docs/pp_racing_line_2026-05-23/LESSONS.md) — sign convention, BOF coupling, integration notes  
- Branch snapshot: `git checkout need_for_speed -- src/AutoCarROS2/autocar_nav/nodes/pure_pursuit.py`
