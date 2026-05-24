# Lap time CSV logs

Paths and naming are defined in [`lap_times_paths.py`](../../src/AutoCarROS2/autocar_nav/autocar_nav/lap_times_paths.py).

## Files

| File | Source |
|------|--------|
| `lap_times_baseline.csv` | Frozen Stanley reference (copy manually; not appended by `lap_timer`) |
| `lap_times_stanley.csv` | `ros2 launch launches race_launch.py` |
| `lap_times_mpc.csv` | `ros2 launch launches race_mpc_launch.py` |
| `lap_times_pure_pursuit.csv` | `ros2 launch launches race_pure_pursuit_launch.py` |

## CSV columns (7 fields, no `stack` column)

```
session_id,lap_number,timestamp_iso,duration_s,avg_speed_mps,max_speed_mps,distance_m
```

The controller is identified by the filename (`lap_times_<stack>.csv`), not a column in the file.
