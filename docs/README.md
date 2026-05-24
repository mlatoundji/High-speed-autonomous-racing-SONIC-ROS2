# Documentation layout

Aligned with [`lap_times_paths.py`](../src/AutoCarROS2/autocar_nav/autocar_nav/lap_times_paths.py).

```
docs/
├── REPORT_BASELINE.md          # Stanley baseline report
├── REPORT_MPC.md               # MPC report (manual comparison)
└── lap_times/
    ├── README.md
    ├── lap_times_baseline.csv  # frozen reference (not written by lap_timer)
    ├── lap_times_stanley.csv   # live logs, stack=stanley
    ├── lap_times_mpc.csv       # live logs, stack=mpc
    └── lap_times_pure_pursuit.csv
```

`lap_timer` appends completed laps to `lap_times/lap_times_<stack>.csv` for `stanley`, `mpc`, or `pure_pursuit`. Compare CSVs manually or update the `REPORT_*.md` files.

Set `AUTOCAR_REPO_ROOT` to the repo root if path auto-detection fails (e.g. Docker).
