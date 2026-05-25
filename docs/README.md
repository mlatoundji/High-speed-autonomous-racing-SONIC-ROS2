# Documentation layout

Reports live under `docs/`; lap-time run logs live under `[results/](../results/)` (see `[lap_times_paths.py](../src/AutoCarROS2/autocar_nav/autocar_nav/lap_times_paths.py)`).

```
docs/
├── REPORT_BASELINE.md          # Stanley baseline report
├── REPORT_MPC.md               # MPC report (manual comparison)
├── REPORT_PURE_PURSUIT.md      # Pure Pursuit report
└── pp_racing_line_2026-05-23/  # need_for_speed experiment archive
    ├── BASELINE.md, REPORT.md, RESULTS.md, LESSONS.md
    ├── baseline_lap_times.csv
    └── snapshots/

results/
├── <stack>_<run_id>/          # per launch (created at launch)
│   ├── params.yaml
│   └── lap_times.csv         # 15 columns (timing + experiment metadata + tracking)
└── benchmark_<timestamp>/     # batch harness (e.g. benchmark_2026-05-25T16-28-02)
    ├── config.yaml           # resolved run list + config source
    ├── summary.csv
    └── figures/              # optional plots
```

`lap_timer` appends completed laps to `results/<stack>_<run_id>/lap_times.csv` (15 columns).  
Launch creates the run directory and writes `params.yaml` from the stack's `navigation_params.yaml`.

**Experiment launch args** (all race launches; defaults preserve baseline behaviour):


| Arg              | Values                  | Role                                                 |
| ---------------- | ----------------------- | ---------------------------------------------------- |
| `line`           | `centerline` | `racing` | Centerline `waypoints.csv` vs `waypoints_racing.csv` |
| `profile`        | e.g. `default`          | Label in CSV only (no yaml switching yet)            |
| `latency_ms`     | `0` … `1000`            | Perception delay via `latency_injector`              |
| `odom_noise_std` | `0.0` …                 | Pose noise via `odom_noise_injector`                 |


Set `AUTOCAR_REPO_ROOT` to the repo root if path auto-detection fails (e.g. Docker).

---

## Reproduce experiments

Run from the **repository root** after a workspace build. Each launch writes under `results/<stack>_<run_id>/` (`params.yaml`, `lap_times.csv`).

### 1. Build once

```bash
colcon build --packages-select autocar_racing_line autocar_nav autocar_nav_mpc autocar_nav_pure_pursuit launches
source install/setup.bash
export AUTOCAR_REPO_ROOT=$PWD   # optional; needed in some Docker layouts
```

For batch runs via `[scripts/benchmark.py](../scripts/benchmark.py)`:

```bash
pip install -r scripts/requirements.txt   # pyyaml, matplotlib
```

### 2. Manual launch — stack × trajectory

Same optional args on every launch: `profile:=default latency_ms:=0 odom_noise_std:=0.0` (defaults shown).


| Stack            | Launch file                   | Centerline                                                          | Racing line                                                     |
| ---------------- | ----------------------------- | ------------------------------------------------------------------- | --------------------------------------------------------------- |
| **Stanley**      | `race_launch.py`              | `ros2 launch launches race_launch.py line:=centerline`              | `ros2 launch launches race_launch.py line:=racing`              |
| **MPC**          | `race_mpc_launch.py`          | `ros2 launch launches race_mpc_launch.py line:=centerline`          | `ros2 launch launches race_mpc_launch.py line:=racing`          |
| **Pure Pursuit** | `race_pure_pursuit_launch.py` | `ros2 launch launches race_pure_pursuit_launch.py line:=centerline` | `ros2 launch launches race_pure_pursuit_launch.py line:=racing` |


Full example (Stanley, racing line, experiment tags):

```bash
ros2 launch launches race_launch.py \
  line:=racing profile:=default latency_ms:=0 odom_noise_std:=0.0
```

Regenerate racing waypoints after editing the centerline:

```bash
ros2 run autocar_racing_line generate_racing_line.py
```

### 3. Batch benchmark — harness + YAML matrices

`[scripts/benchmark.py](../scripts/benchmark.py)` runs a config list, waits for laps, then writes `results/benchmark_<timestamp>/` (`config.yaml`, `summary.csv`, optional `figures/`). Default per-run timing: `**lap_count:=3**`, `**warmup_laps:=1**` (two laps in `summary.csv`).


| What                                 | Command                                                                            |
| ------------------------------------ | ---------------------------------------------------------------------------------- |
| Smoke (1 lap, Stanley centerline)    | `python3 scripts/benchmark.py --smoke`                                             |
| R1 — Stanley vs PP, centerline       | `python3 scripts/benchmark.py --config scripts/configs/r1_pp_vs_stanley.yaml`      |
| R3 — Stanley, racing line            | `python3 scripts/benchmark.py --config scripts/configs/r3_racing_line.yaml`        |
| R4 — Stanley latency sweep, racing   | `python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml`      |
| R2 — PP profile labels (placeholder) | `python3 scripts/benchmark.py --config scripts/configs/r2_tuning_sweep.yaml`       |
| PP + racing (optional coupling)      | `python3 scripts/benchmark.py --config scripts/configs/r3_pp_racing_optional.yaml` |


Dry-run (no Gazebo):

```bash
python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml --dry-run
```

Suggested full batch order (need_for_speed archive): **R1 → R3 → R4**; see `[scripts/configs/README.md](../scripts/configs/README.md)` and `[pp_racing_line_2026-05-23/](pp_racing_line_2026-05-23/)`.

**MPC** is supported by the harness (`stack: mpc`) but not in the R1–R4 YAML files. Example one-off config:

```bash
cat > /tmp/mpc_racing.yaml <<'EOF'
- stack: mpc
  line: racing
  lap_count: 3
  warmup_laps: 1
EOF
python3 scripts/benchmark.py --config /tmp/mpc_racing.yaml
```

Use `line: centerline` for the lap-time comparison in `[REPORT_MPC.md](REPORT_MPC.md)`. You can also save the same YAML under `scripts/configs/`.

### 4. Report ↔ command map


| Report                                                                       | Stack / line                      | How to reproduce                                                                                 |
| ---------------------------------------------------------------------------- | --------------------------------- | ------------------------------------------------------------------------------------------------ |
| `[REPORT_BASELINE.md](REPORT_BASELINE.md)`                                   | Stanley, centerline               | Manual `race_launch.py line:=centerline`; frozen data in `results/baseline_2026-05-20T22-07-12/` |
| `[REPORT_MPC.md](REPORT_MPC.md)`                                             | MPC, centerline                   | Manual `race_mpc_launch.py line:=centerline` or harness `stack: mpc`                             |
| `[REPORT_PURE_PURSUIT.md](REPORT_PURE_PURSUIT.md)`                           | Pure Pursuit, centerline          | Manual `race_pure_pursuit_launch.py line:=centerline` or harness `stack: pure_pursuit`           |
| `[pp_racing_line_2026-05-23/REPORT.md](pp_racing_line_2026-05-23/REPORT.md)` | Stanley / PP, centerline & racing | `scripts/configs/r1`–`r4` (see §3)                                                               |


---

## Further reading

- Stack details: `[src/AutoCarROS2/README.md](../src/AutoCarROS2/README.md)`
- Config schema & sanity checks: `[scripts/configs/README.md](../scripts/configs/README.md)`

