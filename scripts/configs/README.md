# Benchmark configs

YAML run lists for `[../benchmark.py](../benchmark.py)`.  
Output: `[results/benchmark_<timestamp>/](../../results/)` (`config.yaml`, `summary.csv`, `figures/`).

## Experiment matrix (REPORT Section 5)


| Config                                                     | Report      | Stack / line                       | Notes                                                                                                             |
| ---------------------------------------------------------- | ----------- | ---------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| —                                                          | R0 baseline | Stanley, centerline                | Frozen in `[results/baseline_2026-05-20T22-07-12/](../../results/baseline_2026-05-20T22-07-12/)`; not re-run here |
| `[smoke.yaml](smoke.yaml)`                                 | —           | Stanley, centerline, 1 lap         | Harness smoke test                                                                                                |
| `[r1_pp_vs_stanley.yaml](r1_pp_vs_stanley.yaml)`           | 5.2 R1      | Stanley vs PP, centerline, default | 3 laps, warmup 1 per run (2 effective laps in summary)                                                              |
| `[r2_tuning_sweep.yaml](r2_tuning_sweep.yaml)`             | 5.3 R2      | PP only, 3 profile labels          | **Temporary**: profile does not change yaml yet                                                                   |
| `[r3_racing_line.yaml](r3_racing_line.yaml)`               | 5.4 R3      | Stanley, racing                    | Headline: ~172 s vs centerline ~190 s                                                                             |
| `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)` | 5.4 note    | PP, racing                         | Optional: integration coupling repro                                                                              |
| `[r4_latency_sweep.yaml](r4_latency_sweep.yaml)`           | 5.5 R4      | Stanley, racing                    | `latency_ms`: 0, 100, 200, 300, 500, 1000                                                                         |
| `[r5_pp_racing_lookahead.yaml](r5_pp_racing_lookahead.yaml)` | R5 (PP)   | PP, racing                         | 3 laps, warmup 1; lookahead / `closest_search_ahead` grid (4 runs + baseline)                                   |
| `[r5_pp_racing_speed_soft.yaml](r5_pp_racing_speed_soft.yaml)` | R5 (PP) | PP, racing                         | 3 laps, warmup 1; cruise / `max_lateral_accel` / soft weights / accel (5 runs + baseline)                         |
| `[r5_pp_racing_path_curvature.yaml](r5_pp_racing_path_curvature.yaml)` | R5 (PP) | PP, racing              | 3 laps, warmup 1; curvature lookahead, global waypoint horizon, steer smoothing (4 runs + baseline)                 |


Historical snapshots: `[results/benchmark_2026-05-23T16-28-02](../../results/benchmark_2026-05-23T16-28-02)`.

## Inline `navigation` (Pure Pursuit stacks)

When `stack` is `pure_pursuit`, a run may include a top-level `navigation:` mapping. Its shape matches `[autocar_nav_pure_pursuit/config/navigation_params.yaml](../../src/AutoCarROS2/autocar_nav_pure_pursuit/config/navigation_params.yaml)` (ROS 2 nested `ros__parameters` under `localisation`, `local_planner`, `global_planner`, `path_tracker`). `benchmark.py` writes each run’s dict to `results/benchmark_<ts>/nav_overrides/run_NNN.yaml` and passes it as `nav_config:=` to the launch file. **Each list entry must include the full `navigation` tree** if you use inline overrides (there is no merge with the repo default file at benchmark time).

Use `profile` as the CSV label to tell runs apart; it does not switch YAML by itself.

## R5 — PP + racing line tuning

Derived from `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)`. REPORT Section 5.4 notes that Pure Pursuit on the **inward racing line** can degrade by lap 2; R5 splits sweeps by theme so batches stay shorter than one giant matrix. All R5 runs use `line: racing`, `lap_count: 3`, `warmup_laps: 1` (same convention as R1: one warmup lap excluded from `summary.csv`; wall-clock is order-of-magnitude ~30–40 min per launch, machine-dependent).

In the R5 YAML files, keys that **differ from that file’s `*_baseline` profile** are marked with an end-of-line comment ``# TUNED (baseline …)``; block comments ``# Deltas vs …`` / ``# --- frozen …`` summarise what moved vs what did not.

### Shared parameters (where to look in `navigation`)

| ROS path | Parameter | Role |
| -------- | --------- | ---- |
| `local_planner` | `cruise_velocity`, `avoid_velocity` | Target speed caps for the velocity planner along the path. |
| `local_planner` | `max_lateral_accel` | Lateral acceleration limit used when curvature / geometry tightens the feasible speed. |
| `local_planner` | `min_curvature` | Floor on curvature magnitude used in speed shaping (higher → more conservative in bends). |
| `local_planner` | `curvature_lookahead` | How far along the discretised path curvature is peeked for slowdown (shorter → react earlier in index space). |
| `local_planner` | `curvature_smooth_window` | Moving-window smoothing of curvature samples (wider → smoother κ, less jitter in speed). |
| `local_planner` | `accel_rate`, `decel_rate` | Longitudinal slew limits feeding the commanded speed. |
| `global_planner` | `waypoints_ahead`, `waypoint_search_ahead` | Horizon of waypoint lookahead / search — affects how much of the loop the planner considers ahead of the ego frame. |
| `path_tracker` | `lookahead_gain`, `lookahead_min`, `lookahead_max` | Pure Pursuit distance \(L_d \approx k\cdot v\) with clamps; dominant tuning for tracking vs stability. |
| `path_tracker` | `closest_search_ahead` | Forward index window when re-anchoring closest point on the path (smaller → more local, can reduce jumps on tight geometry). |
| `path_tracker` | `steering_limits`, `steer_smoothing` | Saturation and low-pass on steering command (lower smoothing → snappier, more demand on stability). |
| `path_tracker` | `lateral_soft`, `heading_soft` | Soft-error weights in the velocity / path-following objective (higher lateral → penalise cross-track more; higher heading → penalise yaw error more). |

### `[r5_pp_racing_lookahead.yaml](r5_pp_racing_lookahead.yaml)`

| `profile` | Adjusted keys (path_tracker unless noted) | Intent |
| --------- | ------------------------------------------ | ------ |
| `r5_ld_baseline` | — (matches R3 optional defaults) | Reference. |
| `r5_ld_tight` | `lookahead_gain` 0.32, `lookahead_min` 1.2, `lookahead_max` 5.0, `closest_search_ahead` 100 | Shorter \(L_d\) at speed → sharper response on an aggressive racing line. |
| `r5_ld_smooth` | `lookahead_gain` 0.48, `lookahead_min` 2.0, `lookahead_max` 7.0, `closest_search_ahead` 150 | Longer \(L_d\) → less steering chatter when the line cuts corners. |
| `r5_ld_high_min` | `lookahead_gain` 0.35, `lookahead_min` 2.5, `lookahead_max` 6.0 | Raise the lookahead floor so low-speed segments stay less twitchy through apex transitions. |

### `[r5_pp_racing_speed_soft.yaml](r5_pp_racing_speed_soft.yaml)`

| `profile` | Adjusted keys | Intent |
| --------- | -------------- | ------ |
| `r5_sp_baseline` | — | Reference. |
| `r5_sp_slower_high_g` | `local_planner`: `cruise_velocity` / `avoid_velocity` 7.5; `max_lateral_accel` 6.0 | Slightly lower target speed with more lateral budget — favours clean cornering over straight-line hero numbers. |
| `r5_sp_faster_cap_g` | `cruise_velocity` / `avoid_velocity` 8.5; `max_lateral_accel` 5.0 | Push straight-line pace while tightening lateral cap — tests whether the car stays on the racing line when asked to carry more speed. |
| `r5_sp_soft_strict_path` | `path_tracker`: `lateral_soft` 5.5, `heading_soft` 0.45 | Stronger cross-track penalty, weaker heading penalty → “stay on the drawn line” bias for PP + inward geometry. |
| `r5_sp_soft_loose_time` | `path_tracker`: `lateral_soft` 3.0, `heading_soft` 0.85; `local_planner`: `accel_rate` 5.5, `decel_rate` 7.5 | Softer lateral cost, stronger heading alignment, slightly faster longitudinal ramps — bias toward lap time if the controller can stay stable. |

### `[r5_pp_racing_path_curvature.yaml](r5_pp_racing_path_curvature.yaml)`

| `profile` | Adjusted keys | Intent |
| --------- | -------------- | ------ |
| `r5_pc_baseline` | — | Reference. |
| `r5_pc_early_brake` | `local_planner`: `curvature_lookahead` 115, `min_curvature` 0.016 | See curvature effects sooner and treat “gentle” bends as slightly tighter → earlier speed reduction before hairpins. |
| `r5_pc_late_commit` | `curvature_lookahead` 165, `min_curvature` 0.009 | Delayed / diluted curvature response → carry speed longer; higher risk of overspeed if the line is tight. |
| `r5_pc_smooth_kappa_long_wp` | `local_planner`: `curvature_smooth_window` 9; `global_planner`: `waypoints_ahead` 7, `waypoint_search_ahead` 42 | Smoother κ estimates plus longer waypoint horizon — reduces speed jitter and waypoint churn on a closed loop. |
| `r5_pc_steer_damped` | `path_tracker`: `steer_smoothing` 0.82, `steering_limits` 0.92 | Low-pass and cap steering to damp oscillations that show up when the racing line and PP lookahead fight each other. |

## Run order (full reproduction)

```bash
python3 scripts/benchmark.py --config scripts/configs/r1_pp_vs_stanley.yaml
python3 scripts/benchmark.py --config scripts/configs/r3_racing_line.yaml
python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml
# Placeholder batch (optional):
python3 scripts/benchmark.py --config scripts/configs/r2_tuning_sweep.yaml
# Optional coupling experiment:
python3 scripts/benchmark.py --config scripts/configs/r3_pp_racing_optional.yaml
# R5 PP + racing tuning batches (split by theme):
python3 scripts/benchmark.py --config scripts/configs/r5_pp_racing_lookahead.yaml
python3 scripts/benchmark.py --config scripts/configs/r5_pp_racing_speed_soft.yaml
python3 scripts/benchmark.py --config scripts/configs/r5_pp_racing_path_curvature.yaml
```

Dry-run any matrix:

```bash
python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml --dry-run
```

## Schema


| Field            | Default         | Description                                        |
| ---------------- | --------------- | -------------------------------------------------- |
| `stack`          | `stanley`       | `stanley`, `mpc`, `pure_pursuit`                   |
| `profile`        | `default`       | Label in CSV only (no yaml switching unless noted) |
| `line`           | `centerline`    | `centerline` or `racing`                           |
| `latency_ms`     | `0`             | `latency_injector` delay                           |
| `odom_noise_std` | `0.0`           | `odom_noise_injector` noise                        |
| `lap_count`      | `3`             | Laps to wait for per launch                        |
| `warmup_laps`    | CLI default `1` | Laps excluded from `summary.csv` stats             |
| `navigation`   | omitted         | Optional full nav YAML dict for `pure_pursuit`; mutually exclusive with `nav_config` path (see [Inline `navigation`](#inline-navigation-pure-pursuit-stacks)) |


## Expected magnitudes (sanity check)

- **R1**: Stanley ~190 s; PP ~195–196 s (centerline)
- **R3**: Stanley racing ~172 s
- **R4**: U-curve; best ~164 s @ 300 ms; 500 ms slower; 1000 ms timeout/off-track

Exact values may differ from 2026-05-23 runs (Gazebo state, BOF, etc.).