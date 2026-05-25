# Documentation layout

Reports live under `docs/`; lap-time run logs live under [`results/`](../results/) (see [`lap_times_paths.py`](../src/AutoCarROS2/autocar_nav/autocar_nav/lap_times_paths.py)).

```
docs/
├── REPORT_BASELINE.md          # Stanley baseline report
├── REPORT_MPC.md               # MPC report (manual comparison)
└── REPORT_PURE_PURSUIT.md      # Pure Pursuit report

results/
└── <stack>_<run_id>/          # per run (created at launch)
    ├── params.yml
    └── lap_times.csv
```

`lap_timer` appends completed laps to `results/<stack>_<run_id>/lap_times.csv`.  
Launch creates the run directory and writes `params.yml` from the stack's `navigation_params.yaml`.  
Compare runs manually or update the `REPORT_*.md` files.

Set `AUTOCAR_REPO_ROOT` to the repo root if path auto-detection fails (e.g. Docker).
