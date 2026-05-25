# Documentation layout

Reports live under `docs/`; lap-time run logs live under [`results/`](../results/) (see [`lap_times_paths.py`](../src/AutoCarROS2/autocar_nav/autocar_nav/lap_times_paths.py)).

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

- `profile:=default` — label recorded in CSV (no yaml switching yet)
- `latency_ms:=0` — perception delay via `latency_injector` (0 = pass-through)
- `odom_noise_std:=0.0` — pose noise via `odom_noise_injector` (0 = pass-through)
- `line:=centerline|racing` — waypoint track

Automated benchmarks: [`scripts/benchmark.py`](../scripts/benchmark.py) with experiment matrices in [`scripts/configs/`](../scripts/configs/) (`r1`–`r4`, see [`scripts/configs/README.md`](../scripts/configs/README.md)).

Set `AUTOCAR_REPO_ROOT` to the repo root if path auto-detection fails (e.g. Docker).
