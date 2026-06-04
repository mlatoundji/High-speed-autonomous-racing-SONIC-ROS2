# Benchmark configs

YAML run lists for `[../benchmark.py](../benchmark.py)`.  
Output: `[results/benchmark_<timestamp>/](../../results/)` (`config.yaml`, `summary.csv`, `figures/`).

## Experiment matrix (REPORT Section 5)


| Config                                                     | Report      | Stack / line                       | Runs | Notes                                                                                                             |
| ---------------------------------------------------------- | ----------- | ---------------------------------- | ---- | ----------------------------------------------------------------------------------------------------------------- |
| —                                                          | R0 baseline | Stanley, centerline                | —    | Frozen in `[results/baseline_2026-05-20T22-07-12/](../../results/baseline_2026-05-20T22-07-12/)`; not re-run here |
| `[smoke.yaml](smoke.yaml)`                                 | —           | Stanley, centerline                | 1    | 1 lap, no warmup — harness smoke test                                                                             |
| `[r1_pp_vs_stanley.yaml](r1_pp_vs_stanley.yaml)`           | 5.2 R1      | Stanley vs PP, centerline          | 2    | `lap_count` 3, `warmup_laps` 1 → 2 laps in `summary.csv`                                                        |
| `[r2_tuning_sweep.yaml](r2_tuning_sweep.yaml)`             | 5.3 R2      | PP only, centerline                | 3    | **Temporary**: `profile` label only; same default nav params for all three runs                                   |
| `[r3_racing_line.yaml](r3_racing_line.yaml)`               | 5.4 R3      | Stanley, racing                    | 1    | Headline: ~172 s vs centerline ~190 s                                                                             |
| `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)` | 5.4 note  | PP, racing + inline `navigation`   | 1    | `lap_count` 1, `warmup_laps` 0; coupling / integration repro                                                      |
| `[r4_latency_sweep.yaml](r4_latency_sweep.yaml)`           | 5.5 R4      | Stanley, racing                    | 6    | `latency_ms`: 0, 100, 200, 300, 500, 1000                                                                         |
| `[r5_pp_racing_lookahead.yaml](r5_pp_racing_lookahead.yaml)` | R5 (PP)   | PP, racing + inline `navigation`   | 4    | Lookahead / `closest_search_ahead` grid (3 tuned + baseline)                                                      |
| `[r5_pp_racing_speed_soft.yaml](r5_pp_racing_speed_soft.yaml)` | R5 (PP) | PP, racing + inline `navigation`   | 5    | Cruise / `max_lateral_accel` / soft weights / accel (4 tuned + baseline)                                          |
| `[r5_pp_racing_path_curvature.yaml](r5_pp_racing_path_curvature.yaml)` | R5 (PP) | PP, racing + inline `navigation` | 5    | Curvature pipeline, global waypoint horizon, steer filter (4 tuned + baseline)                                    |


Historical snapshots: `[results/benchmark_2026-05-23T16-28-02](../../results/benchmark_2026-05-23T16-28-02)`.

## Per-file run lists

### `[smoke.yaml](smoke.yaml)`

| # | `stack` | `profile` | `line`       | `latency_ms` | `odom_noise_std` | `lap_count` | `warmup_laps` | `navigation` |
| - | ------- | --------- | ------------ | ------------ | ---------------- | ----------- | ------------- | ------------ |
| 1 | stanley | default   | centerline   | 0            | 0.0              | 1           | 0             | —            |

### `[r1_pp_vs_stanley.yaml](r1_pp_vs_stanley.yaml)`

| # | `stack`       | `profile` | `line`       | `latency_ms` | `odom_noise_std` | `lap_count` | `warmup_laps` |
| - | ------------- | --------- | ------------ | ------------ | ---------------- | ----------- | ------------- |
| 1 | stanley       | default   | centerline   | 0            | 0.0              | 3           | 1             |
| 2 | pure_pursuit  | default   | centerline   | 0            | 0.0              | 3           | 1             |

No inline `navigation` — PP uses repo `[navigation_params.yaml](../../src/AutoCarROS2/autocar_nav_pure_pursuit/config/navigation_params.yaml)`.

### `[r2_tuning_sweep.yaml](r2_tuning_sweep.yaml)`

| # | `stack`      | `profile`     | `line`       | `latency_ms` | `odom_noise_std` | `lap_count` | `warmup_laps` |
| - | ------------ | ------------- | ------------ | ------------ | ---------------- | ----------- | ------------- |
| 1 | pure_pursuit | conservative  | centerline   | 0            | 0.0              | 3           | 1             |
| 2 | pure_pursuit | balanced      | centerline   | 0            | 0.0              | 3           | 1             |
| 3 | pure_pursuit | aggressive    | centerline   | 0            | 0.0              | 3           | 1             |

### `[r3_racing_line.yaml](r3_racing_line.yaml)`

| # | `stack` | `profile` | `line`  | `latency_ms` | `odom_noise_std` | `lap_count` | `warmup_laps` |
| - | ------- | --------- | ------- | ------------ | ---------------- | ----------- | ------------- |
| 1 | stanley | default   | racing  | 0            | 0.0              | 3           | 1             |

### `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)`

| # | `stack`      | `profile` | `line`  | `latency_ms` | `odom_noise_std` | `lap_count` | `warmup_laps` | Inline `navigation` |
| - | ------------ | --------- | ------- | ------------ | ---------------- | ----------- | ------------- | --------------------- |
| 1 | pure_pursuit | default   | racing  | 0            | 0.0              | 1           | 0             | Full tree — values match [R5 shared baseline](#r5-shared-baseline-navigation); no `# TUNED` deltas (reference for R5) |

### `[r4_latency_sweep.yaml](r4_latency_sweep.yaml)`

| # | `stack` | `profile` | `line`  | `latency_ms` | `odom_noise_std` | `lap_count` | `warmup_laps` |
| - | ------- | --------- | ------- | ------------ | ---------------- | ----------- | ------------- |
| 1 | stanley | default   | racing  | 0            | 0.0              | 3           | 1             |
| 2 | stanley | default   | racing  | 100          | 0.0              | 3           | 1             |
| 3 | stanley | default   | racing  | 200          | 0.0              | 3           | 1             |
| 4 | stanley | default   | racing  | 300          | 0.0              | 3           | 1             |
| 5 | stanley | default   | racing  | 500          | 0.0              | 3           | 1             |
| 6 | stanley | default   | racing  | 1000         | 0.0              | 3           | 1             |

### R5 batch run lists (common launch fields)

All entries in `[r5_pp_racing_lookahead.yaml](r5_pp_racing_lookahead.yaml)`, `[r5_pp_racing_speed_soft.yaml](r5_pp_racing_speed_soft.yaml)`, and `[r5_pp_racing_path_curvature.yaml](r5_pp_racing_path_curvature.yaml)` share: `stack: pure_pursuit`, `line: racing`, `latency_ms: 0`, `odom_noise_std: 0.0`, `lap_count: 2`, `warmup_laps: 1`, inline `navigation`. Parameter sweeps are in [R5 tables](#r5--pp--racing-line-tuning).

#### `[r5_pp_racing_lookahead.yaml](r5_pp_racing_lookahead.yaml)`

| # | `profile`        |
| - | ---------------- |
| 1 | `r5_ld_baseline` |
| 2 | `r5_ld_tight`    |
| 3 | `r5_ld_smooth`   |
| 4 | `r5_ld_high_min` |

#### `[r5_pp_racing_speed_soft.yaml](r5_pp_racing_speed_soft.yaml)`

| # | `profile`               |
| - | ----------------------- |
| 1 | `r5_sp_baseline`        |
| 2 | `r5_sp_slower_high_g`   |
| 3 | `r5_sp_faster_cap_g`    |
| 4 | `r5_sp_soft_strict_path`|
| 5 | `r5_sp_soft_loose_time` |

#### `[r5_pp_racing_path_curvature.yaml](r5_pp_racing_path_curvature.yaml)`

| # | `profile`                  |
| - | -------------------------- |
| 1 | `r5_pc_baseline`           |
| 2 | `r5_pc_early_brake`        |
| 3 | `r5_pc_late_commit`        |
| 4 | `r5_pc_smooth_kappa_long_wp` |
| 5 | `r5_pc_steer_damped`       |

## Inline `navigation` (Pure Pursuit stacks)

When `stack` is `pure_pursuit`, a run may include a top-level `navigation:` mapping. Its shape matches `[autocar_nav_pure_pursuit/config/navigation_params.yaml](../../src/AutoCarROS2/autocar_nav_pure_pursuit/config/navigation_params.yaml)` (ROS 2 nested `ros__parameters` under `localisation`, `local_planner`, `global_planner`, `path_tracker`). `benchmark.py` writes each run’s dict to `results/benchmark_<ts>/nav_overrides/run_NNN.yaml` and passes it as `nav_config:=` to the launch file. **Each list entry must include the full `navigation` tree** if you use inline overrides (there is no merge with the repo default file at benchmark time).

Use `profile` as the CSV label to tell runs apart; it does not switch YAML by itself.

## R5 — PP + racing line tuning

Derived from `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)`. REPORT Section 5.4 notes that Pure Pursuit on the **inward racing line** can degrade by lap 2; R5 splits sweeps by theme so batches stay shorter than one giant matrix.

**Batch defaults (all R5 YAML entries):** `stack: pure_pursuit`, `line: racing`, `latency_ms: 0`, `odom_noise_std: 0.0`, `lap_count: 2`, `warmup_laps: 1` → **1 lap** in `summary.csv` per launch (wall-clock order-of-magnitude ~30–40 min per launch, machine-dependent).

In the R5 YAML files, keys that **differ from that file’s `*_baseline` profile** are marked with an end-of-line comment ``# TUNED (baseline …)``; block comments ``# Deltas vs …`` / ``# --- frozen …`` summarise what moved vs what did not.

### Parameter glossary (tunable keys in R5 tables)

| ROS node | Parameter | Role |
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

Other keys in each R5 `navigation` block (`update_frequency`, `car_width`, `wheelbase`, `waypoints_behind`, `passed_threshold`, `velocity_gain`, `startup_ramp_s`, etc.) are **identical across every profile in a file** and match `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)` / repo defaults.

In the per-file tables below, **Adjusted keys** lists the **union of every parameter marked `# TUNED` in that YAML** (vs that file’s `*_baseline` profile). **Every row—including baseline—lists all keys in that union** with this profile’s values; keys at baseline values are shown without a “(baseline …)” note. Keys outside the union are frozen at [shared baseline](#r5-shared-baseline-navigation) and omitted from the column.

### R5 shared baseline (`navigation`)

Every `r5_*_baseline` profile (and `[r3_pp_racing_optional.yaml](r3_pp_racing_optional.yaml)`) uses these values for the swept keys:

| Node | Parameter | Baseline value |
| ---- | --------- | -------------- |
| `local_planner` | `cruise_velocity` | 8.0 |
| `local_planner` | `avoid_velocity` | 8.0 |
| `local_planner` | `max_lateral_accel` | 5.5 |
| `local_planner` | `min_curvature` | 0.012 |
| `local_planner` | `curvature_lookahead` | 140 |
| `local_planner` | `curvature_smooth_window` | 5 |
| `local_planner` | `accel_rate` | 5.0 |
| `local_planner` | `decel_rate` | 7.0 |
| `global_planner` | `waypoints_ahead` | 5 |
| `global_planner` | `waypoint_search_ahead` | 30 |
| `path_tracker` | `lookahead_gain` | 0.4 |
| `path_tracker` | `lookahead_min` | 1.5 |
| `path_tracker` | `lookahead_max` | 6.0 |
| `path_tracker` | `closest_search_ahead` | 120 |
| `path_tracker` | `steering_limits` | 0.95 |
| `path_tracker` | `steer_smoothing` | 1.0 |
| `path_tracker` | `lateral_soft` | 4.0 |
| `path_tracker` | `heading_soft` | 0.6 |

At v ≈ 8 m/s, baseline PP lookahead is \(L_d \approx 0.4 \times 8 = 3.2\) m, clamped to `[lookahead_min, lookahead_max]` → **3.2 m** (see `need_for_speed` comment in repo `navigation_params.yaml`).

### `[r5_pp_racing_lookahead.yaml](r5_pp_racing_lookahead.yaml)`

**Motivation:** Pure Pursuit mostly steers by “how far ahead on the path to aim.” On the **inward racing line** (REPORT 5.4), the default lookahead tuned for centerline can wobble or drift off by lap 2 while speed planning stays fixed. This file changes **only** lookahead and closest-point search — not cruise speed or bend braking — so you can see whether lap-2 failures are mainly a **steering-distance** problem.

**Sweep keys (union, this file):** `path_tracker`: `lookahead_gain`, `lookahead_min`, `lookahead_max`, `closest_search_ahead`.

Frozen for all profiles: entire `localisation`, `local_planner`, and `global_planner` trees; all other `path_tracker` keys at [shared baseline](#r5-shared-baseline-navigation).

| `profile` | Adjusted keys (`path_tracker`) | Intent |
| --------- | ------------------------------ | ------ |
| `r5_ld_baseline` | `lookahead_gain` **0.4**; `lookahead_min` **1.5**; `lookahead_max` **6.0**; `closest_search_ahead` **120** | Starting point — same as R3 optional / shared baseline; compare other rows to this. |
| `r5_ld_tight` | `lookahead_gain` **0.32** (baseline 0.4); `lookahead_min` **1.2** (1.5); `lookahead_max` **5.0** (6.0); `closest_search_ahead` **100** (120) | Steer toward a nearer point on the path — turns react faster; may help on a tight inward line but can feel nervous. |
| `r5_ld_smooth` | `lookahead_gain` **0.48** (0.4); `lookahead_min` **2.0** (1.5); `lookahead_max` **7.0** (6.0); `closest_search_ahead` **150** (120) | Steer toward a farther point — wheel moves less often when the racing line cuts corners sharply. |
| `r5_ld_high_min` | `lookahead_gain` **0.35** (0.4); `lookahead_min` **2.5** (1.5); `lookahead_max` **6.0**; `closest_search_ahead` **120** | Even at low speed, don’t use a very short steer target — less left-right twitch in slow corners and apexes. |

### `[r5_pp_racing_speed_soft.yaml](r5_pp_racing_speed_soft.yaml)`

**Motivation:** Even with stable steering, PP on the racing line can still lose laps because the **velocity planner** asks for the wrong speed or cares about the wrong errors (on-path vs heading). This file freezes lookahead and curvature/waypoint logic at baseline and sweeps **target speed**, **cornering grip limit**, **soft penalties**, and **how fast speed can ramp** — to separate “can it steer?” from “does it choose a safe, fast pace on this line?”

**Sweep keys (union, this file):** `local_planner`: `cruise_velocity`, `avoid_velocity`, `max_lateral_accel`, `accel_rate`, `decel_rate`; `path_tracker`: `lateral_soft`, `heading_soft`.

Frozen for all profiles: entire `localisation` tree; `global_planner` at baseline; all other `local_planner` / `path_tracker` glossary keys at [shared baseline](#r5-shared-baseline-navigation).

| `profile` | Adjusted keys | Intent |
| --------- | -------------- | ------ |
| `r5_sp_baseline` | `local_planner`: `cruise_velocity` **8.0**; `avoid_velocity` **8.0**; `max_lateral_accel` **5.5**; `accel_rate` **5.0**; `decel_rate` **7.0**; `path_tracker`: `lateral_soft` **4.0**; `heading_soft` **0.6** | Starting point — compare other rows to this. |
| `r5_sp_slower_high_g` | `local_planner`: `cruise_velocity` **7.5** (8.0); `avoid_velocity` **7.5** (8.0); `max_lateral_accel` **6.0** (5.5); `accel_rate` **5.0**; `decel_rate` **7.0**; `path_tracker`: `lateral_soft` **4.0**; `heading_soft` **0.6** | Cap speed a little lower but allow more cornering grip — trade straight-line pace for staying on the racing line. |
| `r5_sp_faster_cap_g` | `local_planner`: `cruise_velocity` **8.5** (8.0); `avoid_velocity` **8.5** (8.0); `max_lateral_accel` **5.0** (5.5); `accel_rate` **5.0**; `decel_rate` **7.0**; `path_tracker`: `lateral_soft` **4.0**; `heading_soft` **0.6** | Ask for a bit more top speed but tighten the cornering limit — check if the car still follows the line when pushed. |
| `r5_sp_soft_strict_path` | `local_planner`: `cruise_velocity` **8.0**; `avoid_velocity` **8.0**; `max_lateral_accel` **5.5**; `accel_rate` **5.0**; `decel_rate` **7.0**; `path_tracker`: `lateral_soft` **5.5** (4.0); `heading_soft` **0.45** (0.6) | Care more about being on the path, less about exact heading — try to stick to the drawn racing line. |
| `r5_sp_soft_loose_time` | `local_planner`: `cruise_velocity` **8.0**; `avoid_velocity` **8.0**; `max_lateral_accel` **5.5**; `accel_rate` **5.5** (5.0); `decel_rate` **7.5** (7.0); `path_tracker`: `lateral_soft` **3.0** (4.0); `heading_soft` **0.85** (0.6) | Allow a bit more sideways slack, align heading more strongly, speed up and slow down faster — aim for lap time if nothing oscillates. |

### `[r5_pp_racing_path_curvature.yaml](r5_pp_racing_path_curvature.yaml)`

**Motivation:** The racing line is sharper and less symmetric than centerline, so **when the car slows for bends** and **how far ahead it plans waypoints** matter as much as PP lookahead. Noisy bend detection causes speed to bounce; a short waypoint window can hop on a closed track; steering can still shake if the line and PP disagree. This file tunes **bend-based braking**, **waypoint horizon**, and **steering smoothing** while holding PP lookahead and cruise settings at baseline.

**Sweep keys (union, this file):** `local_planner`: `min_curvature`, `curvature_lookahead`, `curvature_smooth_window`; `global_planner`: `waypoints_ahead`, `waypoint_search_ahead`; `path_tracker`: `steering_limits`, `steer_smoothing`.

Frozen for all profiles: entire `localisation` tree; `local_planner` cruise / lateral-g / accel keys at baseline; all other `path_tracker` keys at [shared baseline](#r5-shared-baseline-navigation).

| `profile` | Adjusted keys | Intent |
| --------- | -------------- | ------ |
| `r5_pc_baseline` | `local_planner`: `min_curvature` **0.012**; `curvature_lookahead` **140**; `curvature_smooth_window` **5**; `global_planner`: `waypoints_ahead` **5**; `waypoint_search_ahead` **30**; `path_tracker`: `steering_limits` **0.95**; `steer_smoothing` **1.0** | Starting point — compare other rows to this. |
| `r5_pc_early_brake` | `local_planner`: `min_curvature` **0.016** (0.012); `curvature_lookahead` **115** (140); `curvature_smooth_window` **5**; `global_planner`: `waypoints_ahead` **5**; `waypoint_search_ahead` **30**; `path_tracker`: `steering_limits` **0.95**; `steer_smoothing` **1.0** | Treat bends as sharper and notice them sooner — slow down earlier before tight corners. |
| `r5_pc_late_commit` | `local_planner`: `min_curvature` **0.009** (0.012); `curvature_lookahead` **165** (140); `curvature_smooth_window` **5**; `global_planner`: `waypoints_ahead` **5**; `waypoint_search_ahead` **30**; `path_tracker`: `steering_limits` **0.95**; `steer_smoothing` **1.0** | Treat bends as gentler and react later — keep speed longer; may run hot if the corner is actually tight. |
| `r5_pc_smooth_kappa_long_wp` | `local_planner`: `min_curvature` **0.012**; `curvature_lookahead` **140**; `curvature_smooth_window` **9** (5); `global_planner`: `waypoints_ahead` **7** (5); `waypoint_search_ahead` **42** (30); `path_tracker`: `steering_limits` **0.95**; `steer_smoothing` **1.0** | Smooth bend detection and look at more waypoints ahead — less speed bobbing and fewer waypoint jumps on the full loop. |
| `r5_pc_steer_damped` | `local_planner`: `min_curvature` **0.012**; `curvature_lookahead` **140**; `curvature_smooth_window` **5**; `global_planner`: `waypoints_ahead` **5**; `waypoint_search_ahead` **30**; `path_tracker`: `steering_limits` **0.92** (0.95); `steer_smoothing` **0.82** (1.0) | Softer, capped steering — calm left-right shaking when the racing line and Pure Pursuit pull different ways. |

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
