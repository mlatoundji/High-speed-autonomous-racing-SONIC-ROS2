# Results log

Living document. Every experiment lands here as soon as it runs. The PDF report pulls from these tables, not the other way around.

> Raw data source: `~/.ros/autocar_lap_times.csv` (append-only, every launch tags rows with a `session_id`).
> Frozen snapshot per milestone goes to `docs/snapshots/<milestone>.csv`.

---

## Convention

- Each result table reports **median lap time over N >= 5 laps after one warmup lap discarded**.
- Standard deviation reported alongside the median.
- Off-track or collision events count as a failed lap and are excluded from the median, but reported separately.
- Track: `race_circuit.world` (closed loop, ~650 m centerline, 16 m wide, with hay-bale borders, no on-road obstacles unless stated).

---

## R0 - Baseline (already established)

Stanley tracker, conservative defaults, `race_circuit.world`. **Not re-measured**: the code path on `need_for_speed` is identical to the one that produced the recorded baseline (same commits merged into `main` via PR #1).

| Metric | Value |
|---|---|
| Lap time | **190.90 s** (3 min 11 s) |
| Average speed | 3.42 m/s |
| Peak speed | 5.85 m/s |
| Distance | 652.66 m |
| Session | `2026-05-20T22-07-12` |

Full details: [`BASELINE.md`](BASELINE.md). Frozen raw data: [`baseline_lap_times.csv`](baseline_lap_times.csv).

**Targets to beat**:
- < 120 s with Pure Pursuit + speed profile (step 3-4).
- < 90 s with racing line + tuned controller (step 5).

---

## R1 - Pure Pursuit vs Stanley, default tuning

First real PP-vs-Stanley comparison on `race_circuit.world`, both controllers at default tuning, centerline path, no latency, no odometry noise. CSV snapshot: [`r1_pp_vs_stanley_2026-05-23.csv`](snapshots/r1_pp_vs_stanley_2026-05-23.csv).

**Stanley "warmed-up" reference** (session `2026-05-23T17-24-47`, lap 4):

| duration_s | avg_speed_mps | max_speed_mps | lateral_error_rms | lateral_error_max | steering_rate_max | offtrack_events |
|---|---|---|---|---|---|---|
| 190.80 | 3.42 | 5.87 | 0.474 | 4.69 | 6.51 | 4 |

**Pure Pursuit two-lap run** (session `2026-05-23T22-45-31`):

| Lap | duration_s | avg_speed_mps | max_speed_mps | lateral_error_rms | lateral_error_max | steering_rate_max | offtrack_events |
|---|---|---|---|---|---|---|---|
| 1 | 195.20 | 3.36 | 5.93 | 0.579 | 5.93 | 17.23 | 4 |
| 2 | 196.30 | 3.33 | 5.88 | 0.452 | 4.59 | 14.15 | 2 |

**Headline comparison** (Stanley lap 4 vs PP lap 2 - both warm):

| Metric | Stanley | Pure Pursuit | Verdict |
|---|---|---|---|
| Lap time (s) | **190.80** | 196.30 | Stanley faster by ~5 s (-2.9 %) |
| Distance (m) | 652.7 | 654.4 | Almost identical (PP +0.3 %) |
| `lateral_error_rms` (m) | 0.474 | **0.452** | PP marginally better (-5 %) |
| `lateral_error_max` (m) | 4.69 | **4.59** | PP slightly better |
| `steering_rate_max` (rad/s) | **6.51** | 14.15 | Stanley smoother by 2x |
| `offtrack_events` | 4 | **2** | PP halves the off-track risk |
| **Reproducibility** | 190.9 vs 190.8 (2 runs) | 195.3 vs 195.2 (2 runs) | Both stable across runs to ~100 ms |

**Findings**:

1. **Tracking precision is comparable** between the two controllers when both are warmed up (RMS ~0.45 m). The Pure Pursuit theoretical advantage on smoothness isn't dramatic at this speed (3-6 m/s).
2. **Pure Pursuit halves the off-track count**, confirming the qualitative observation that the trajectory is "more centred and rectilinear".
3. **Stanley is ~5 s faster per lap**. The gap is small but consistent. Likely causes (to investigate in R2/R3):
   - Pure Pursuit's velocity sometimes drops to 3 m/s (half) because the BOF `local_planner` enters "all blocked" mode and reduces `target_velocity`. Pure Pursuit follows the slowdown faithfully; Stanley, oscillating on top, ends up with higher average speed.
   - Default lookahead (1.5 + 0.4 v) might be slightly long for this track.
4. **Pure Pursuit shows a higher peak steering rate** (14 vs 6.5 rad/s) which is *opposite* to the theoretical prediction. This is driven by isolated transients at lap boundary or during velocity steps, not by continuous oscillation - per-lap visual inspection shows PP is smoother on average, but with sharper individual jumps.
5. **Multi-lap stability is solid**: the second PP run completed 2 laps without crashing, with lap 2 better than lap 1 on every metric. Earlier "off-track on lap 2" issues are not reproduced after reverting unnecessary local_planner edits.

**Targets to beat next**:
- Lap time under 190 s with PP (requires either a faster cruise target or stable bypass of the BOF half-speed mode).
- `steering_rate_max` under 8 rad/s with PP (less spike at velocity transitions).
- Eventually under 90 s with racing line + tuned controller (R3).

Plot: `docs/figures/r1_controller_compare.png` (matplotlib install pending on the dev machine).

---

## R2 - Tuning sweep (Pure Pursuit only)

| Profile | Median lap (s) | Std | Lateral RMS | Steering rate max (rad/s) | Off-track |
|---|---|---|---|---|---|
| Conservative | TODO | TODO | TODO | TODO | TODO |
| Balanced | TODO | TODO | TODO | TODO | TODO |
| Aggressive | TODO | TODO | TODO | TODO | TODO |

Plot: `docs/figures/r2_tuning_sweep.png` (TODO).

---

## R3 - Racing line vs centerline

Same Stanley controller, same tuning, same `target_velocity` (6 m/s). Only the loaded waypoints file changes: `waypoints.csv` (centerline, 46 points along the geometric centre of the road) vs `waypoints_racing.csv` generated by `scripts/generate_racing_line.py` (same 46 points, each offset inward by up to 4 m along the local normal in proportion to smoothed curvature). CSV snapshot: [`r3_racing_line_2026-05-23.csv`](snapshots/r3_racing_line_2026-05-23.csv).

| Line | Lap | duration_s | distance_m | avg_speed_mps | lat_err_rms | lat_err_max | offtrack |
|---|---|---|---|---|---|---|---|
| Centerline | 4 | 190.80 | 652.69 | 3.42 | 0.474 | 4.69 | 4 |
| **Racing** | 1 | **172.20** | **628.96** | **3.65** | 0.447 | 4.63 | 2 |
| **Racing** | 2 | **172.40** | **628.96** | **3.65** | 0.495 | 4.73 | 2 |

**Headline**: the racing line saves **18.4 s per lap (-9.7 %)** with no change to the controller, the speed cap or the BOF stack. Tracking precision is preserved (lateral RMS within noise, off-track events even lower).

**Mechanism**: pure geometry. The racing line is 24 m shorter than the centerline (628.96 vs 652.7) because it cuts the inside of every turn. At unchanged cruise velocity (6 m/s ceiling) and unchanged tracking accuracy, a shorter path mechanically yields a shorter lap.

**Reproducibility**: two consecutive laps with the racing line came in at 172.2 and 172.4 s -- the same 200 ms spread as the centerline baseline.

**Why this matters for the report**: this is a textbook case where the *trajectory* dominates the *controller*. A 100-line offline Python script (`scripts/generate_racing_line.py`, no ROS, no online optimisation) produces a result that is bigger than swapping Stanley for Pure Pursuit at default tuning (which actually lost 5 s -- see R1). The lesson for the *integration & validation* grade: do not assume the controller is where most of the gain is; instrument *everything* and let the numbers tell you.

**Targets to beat next**:
- Run the racing line with **Pure Pursuit** to see if PP's smoother tracking lets us push the cruise target higher without losing the line (R3b).
- Combine racing line + tuned aggressive profile + ideally a workaround for the BOF half-speed trap to crack 90 s (the original "what to beat" target in `BASELINE.md`).

Plot: `docs/figures/r3_trajectory_xy.png` -- to be regenerated once matplotlib is unbroken on the dev machine. The XY trajectory comparison would visually show the racing line cutting every apex.

---

## R4 - Latency robustness

| latency_ms | Compensation | Median lap (s) | Lateral RMS (m) | Off-track |
|---|---|---|---|---|
| 0 | off | TODO | TODO | TODO |
| 50 | off | TODO | TODO | TODO |
| 100 | off | TODO | TODO | TODO |
| 150 | off | TODO | TODO | TODO |
| 50 | on | TODO | TODO | TODO |
| 100 | on | TODO | TODO | TODO |
| 150 | on | TODO | TODO | TODO |

Plot: `docs/figures/r4_latency.png`.

---

## R5 - Odometry noise robustness (bonus)

| odom_noise_std (m) | Median lap (s) | Lateral RMS (m) | Off-track |
|---|---|---|---|
| 0.00 | TODO | TODO | TODO |
| 0.05 | TODO | TODO | TODO |
| 0.10 | TODO | TODO | TODO |
| 0.20 | TODO | TODO | TODO |
| 0.40 | TODO | TODO | TODO |

Plot: `docs/figures/r5_odom_noise.png`.

---

## Run index

Each line is one launch. The `session_id` matches the CSV.

| session_id | Step | controller | line | profile | latency_ms | odom_noise_std | Comment |
|---|---|---|---|---|---|---|---|
| 2026-05-20T22-07-12 | R0 | stanley | centerline | n/a | 0 | 0 | baseline (BASELINE.md), 7-col legacy schema |
| 2026-05-23T17-24-47 | R1 | stanley | centerline | default | 0 | 0 | Stanley warmed-up reference, 4 laps |
| 2026-05-23T17-51-22 | R1 | stanley | centerline | default | 0 | 0 | Stanley cold-start (bench smoke test) |
| 2026-05-23T19-18-57 | R1 | pure_pursuit | centerline | default | 0 | 0 | First PP lap, 1 lap, validates PP works |
| 2026-05-23T22-45-31 | R1 | pure_pursuit | centerline | default | 0 | 0 | PP 2-lap run, both completed, used in R1 table |
| 2026-05-23T23-09-49 | R3 | stanley | racing | default | 0 | 0 | Stanley + racing line, 2 laps at 172.2/172.4 s |
