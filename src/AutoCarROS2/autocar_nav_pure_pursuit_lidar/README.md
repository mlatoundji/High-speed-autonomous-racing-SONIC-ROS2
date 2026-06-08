# autocar_nav_pure_pursuit_lidar

LiDAR-driven Pure Pursuit stack with a two-phase race strategy:

| Phase | Trigger | Pipeline |
|-------|---------|----------|
| **Lap 1 — exploration** | `lap_count < 1` | LiDAR + SLAM `/map` → local centerline → cubic spline → Pure Pursuit |
| **Lap 2+ — racing** | `lap_count >= 1` | SLAM TF pose (in-memory map) → precomputed racing line → Pure Pursuit |

## Nodes

| Node | Role |
|------|------|
| `localisation` | SLAM-corrected pose via `map`/`odom` TF → `/autocar/state2D` |
| `global_planner_lidar` | Mode switch; publishes `/autocar/goals` and `/autocar/nav_mode` |
| `local_planner` | Cubic spline + curvature speed (slower in exploration mode) |
| `path_tracker` | Pure Pursuit controller (from `autocar_nav_pure_pursuit`) |
| `slam_toolbox` | Online async SLAM; map stays in memory (no disk save) |

## Launch

```bash
sudo apt install ros-foxy-slam-toolbox   # if not already installed

colcon build --packages-select autocar_nav_pure_pursuit_lidar launches
source install/setup.bash

ros2 launch launches race_pure_pursuit_lidar_launch.py track:=f1_circuit_fenced line:=racing
```

**Same session:** lap 1 builds the SLAM map in memory and follows the LiDAR-derived
corridor centerline. Lap 2+ reuses that live map for localization via TF (no `track_map.pkl`).

## Topics

| Topic | Description |
|-------|-------------|
| `/autocar/nav_mode` | `0` = exploration, `1` = racing |
| `/map` | Occupancy grid from `slam_toolbox` |
| `/autocar/lap_count` | From `lap_timer`; triggers mode switch |

## Parameters

See `config/navigation_params.yaml` and `config/slam_toolbox.yaml`. Key tuning:

- `exploration_velocity` — lap-1 cruise speed (default 3 m/s)
- `exploration_goal_step` / `exploration_goal_count` — local centerline horizon
- `use_slam` — enable SLAM TF pose in `localisation` (default `true`)
