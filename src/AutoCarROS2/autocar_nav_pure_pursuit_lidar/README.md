# autocar_nav_pure_pursuit_lidar

LiDAR-driven Pure Pursuit stack with a two-phase race strategy:

| Phase | Trigger | Pipeline |
|-------|---------|----------|
| **Lap 1 — exploration** | `lap_count < 1` | LiDAR + BOF `/map` → local centerline → cubic spline → Pure Pursuit |
| **Lap 2+ — racing** | `lap_count >= 1` | LiDAR scan-to-map localization → precomputed global racing line → Pure Pursuit |

## Nodes

| Node | Role |
|------|------|
| `localisation` | Odom passthrough + `/autocar/pose_correction` from map matcher |
| `global_planner_lidar` | Mode switch; publishes `/autocar/goals` and `/autocar/nav_mode` |
| `map_saver` | Saves BOF map to `data/track_map.pkl` when lap 1 completes |
| `local_planner` | Cubic spline + curvature speed (slower in exploration mode) |
| `path_tracker` | Pure Pursuit controller (from `autocar_nav_pure_pursuit`) |
| `bof` | LiDAR occupancy mapping (`autocar_map`) |

## Launch

```bash
colcon build --packages-select autocar_nav_pure_pursuit_lidar autocar_map launches
source install/setup.bash

# F1 fenced track (bollards visible to LiDAR)
ros2 launch launches race_pure_pursuit_lidar_launch.py track:=f1_circuit_fenced line:=racing
```

**First session:** lap 1 builds the map and follows the LiDAR-derived corridor centerline.
When lap 1 finishes, `track_map.pkl` is saved under the package `data/` directory.

**Second session onward:** if `track_map.pkl` exists at launch, lap 2+ uses scan matching
against that map and follows the racing line from `autocar_racing_line`.

## Topics

| Topic | Description |
|-------|-------------|
| `/autocar/nav_mode` | `0` = exploration, `1` = racing |
| `/autocar/pose_correction` | Map-localization offset (lap 2+) |
| `/autocar/lap_count` | From `lap_timer`; triggers mode switch |

## Parameters

See `config/navigation_params.yaml`. Key tuning:

- `exploration_velocity` — lap-1 cruise speed (default 4 m/s)
- `exploration_goal_step` / `exploration_goal_count` — local centerline horizon
- `map_localize_search_xy` — scan-matching search radius (lap 2+)
