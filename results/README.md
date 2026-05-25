# Lap time run logs

Paths and naming are defined in [`lap_times_paths.py`](../src/AutoCarROS2/autocar_nav/autocar_nav/lap_times_paths.py).

## Layout

Each launch of a race stack creates one run directory:

```text
results/
  <stack>_<run_id>/
    params.yml       # run metadata + navigation ROS parameters
    lap_times.csv    # one row per completed lap
```

`run_id` is a local timestamp, e.g. `2026-05-24T11-37-34`.  
Directory example: `results/mpc_2026-05-24T11-37-34/`.

| Stack            | Launch                                      |
| ---------------- | ------------------------------------------- |
| `stanley`        | `ros2 launch launches race_launch.py`       |
| `mpc`            | `ros2 launch launches race_mpc_launch.py`   |
| `pure_pursuit`   | `ros2 launch launches race_pure_pursuit_launch.py` |

## `lap_times.csv` columns

```text
session_id,lap_number,timestamp_iso,duration_s,avg_speed_mps,max_speed_mps,distance_m
```

`session_id` matches `run_id` for that directory.

## Legacy files

Flat CSVs at the repo root of `results/` (e.g. `lap_times_mpc.csv`) are from an older layout. New runs use `results/<stack>_<run_id>/` only.  
`lap_times_baseline.csv` remains a frozen Stanley reference (not written by `lap_timer`).

Set `AUTOCAR_REPO_ROOT` to the repo root if path auto-detection fails (e.g. Docker).
