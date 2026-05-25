# Pure Pursuit — `autocar_nav_pure_pursuit` vs Stanley baseline

Compared to [`REPORT_BASELINE.md`](REPORT_BASELINE.md) / [`lap_times_baseline.csv`](../results/lap_times_baseline.csv).  
Recorded **2026-05-24**, session `2026-05-24T17-00-14`. Only **lap 1** is logged so far.

## Configuration


| Item               | Pure Pursuit                                                                                                              | Stanley baseline                                                                                         |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| World / trajectory | Same `race_circuit.world`, centerline `waypoints.csv`                                                                     | Same                                                                                                     |
| Controller         | Pure Pursuit [`tracker.py`](../src/AutoCarROS2/autocar_nav_pure_pursuit/nodes/tracker.py)                                 | Stanley                                                                                                  |
| Local planner      | Cubic spline + curvature speed limit                                                                                      | Linear path, fixed 6.0 m/s                                                                               |
| Launch             | `ros2 launch launches race_pure_pursuit_launch.py`                                                                        | `race_launch.py`                                                                                         |
| Config             | [`autocar_nav_pure_pursuit/.../navigation_params.yaml`](../src/AutoCarROS2/autocar_nav_pure_pursuit/config/navigation_params.yaml) | [`autocar_nav/.../navigation_params.yaml`](../src/AutoCarROS2/autocar_nav/config/navigation_params.yaml) |


### What changes vs baseline


| Parameter        | Pure Pursuit                                              | Stanley           |
| ---------------- | --------------------------------------------------------- | ----------------- |
| Cruise target    | **8.0 m/s**                                               | 6.0 m/s           |
| Speed in corners | `min(v_cruise, √(a_lat_max / \|κ\|))`, `a_lat_max = 5 m/s²` | Fixed 6.0 m/s     |
| Local path       | Cubic spline                                              | Linear            |
| Lateral control  | Pure Pursuit, dynamic lookahead @ 50 Hz                   | Stanley reactive  |
| Lookahead        | `gain=0.55`, `min=2.8 m`, `max=11.0 m` (speed-scaled)    | —                 |
| Wheelbase        | 2.966 m                                                   | —                 |


Peak speed (**7.64 m/s**) exceeds Stanley and MPC (~5.9 m/s): the speed profile uses more of the straight-line capability, while lap time is still set mainly by **average speed** through corners.

## Results

### Session lap


| Lap | Time         | Avg speed | Peak speed | Distance |
| --- | ------------ | --------- | ---------- | -------- |
| 1   | **121.60 s** | 5.38 m/s  | 7.64 m/s   | 654.4 m  |


### Lap 1 — all stacks

Stanley: [`REPORT_BASELINE.md`](REPORT_BASELINE.md) / 2026-05-20. MPC: [`REPORT_MPC.md`](REPORT_MPC.md) / 2026-05-24. Pure Pursuit: lap 1 below.


| Metric     | Stanley | MPC          | Pure Pursuit |
| ---------- | ------- | ------------ | ------------ |
| Lap time   | 190.90 s | **115.70 s** | 121.60 s     |
| Avg speed  | 3.42 m/s | **5.64 m/s** | 5.38 m/s     |
| Peak speed | 5.85 m/s | 5.87 m/s     | **7.64 m/s** |
| Distance   | 652.66 m | 652.72 m     | 654.38 m     |


MPC and Pure Pursuit share the same local planner; MPC is ~6 s faster on lap 1 despite a lower peak. Pure Pursuit uses more straight-line speed but loses time in cornering average.

### Roadmap (lap 1)


| Target              | Goal      | Pure Pursuit | Status      |
| ------------------- | --------- | ------------ | ----------- |
| Beat Stanley        | < 190.9 s | 121.6 s      | Met         |
| Pure Pursuit target | < 120 s   | 121.6 s      | Not met (+1.6 s) |
| Racing line target  | < 90 s    | 121.6 s      | Not met     |


## Reproduce

```bash
colcon build --packages-select autocar_nav autocar_nav_pure_pursuit launches --symlink-install
source install/setup.bash
ros2 launch launches race_pure_pursuit_launch.py
```

## Data

[`../results/`](../results/) — each run is `results/pure_pursuit_<run_id>/` (`params.yml`, `lap_times.csv`).
