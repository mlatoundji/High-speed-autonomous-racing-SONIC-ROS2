# MPC — `autocar_nav_mpc` vs Stanley baseline

Compared to [`REPORT_BASELINE.md`](REPORT_BASELINE.md) / [`lap_times_baseline.csv`](../results/lap_times_baseline.csv).  
Recorded **2026-05-24**, session `2026-05-24T11-37-34`. Tables below use **lap 1** unless noted.

## Configuration


| Item               | MPC                                                                                                              | Stanley baseline                                                                                         |
| ------------------ | ---------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| World / trajectory | Same `race_circuit.world`, centerline `waypoints.csv`                                                            | Same                                                                                                     |
| Controller         | Frenet linear MPC `[tracker.py](../src/AutoCarROS2/autocar_nav_mpc/nodes/tracker.py)`                            | Stanley                                                                                                  |
| Local planner      | Cubic spline + curvature speed limit                                                                             | Linear path, fixed 6.0 m/s                                                                               |
| Launch             | `ros2 launch launches race_mpc_launch.py`                                                                        | `race_launch.py`                                                                                         |
| Config             | `[autocar_nav_mpc/.../navigation_params.yaml](../src/AutoCarROS2/autocar_nav_mpc/config/navigation_params.yaml)` | `[autocar_nav/.../navigation_params.yaml](../src/AutoCarROS2/autocar_nav/config/navigation_params.yaml)` |


### What changes vs baseline


| Parameter             | MPC                                                       | Stanley           |
| --------------------- | --------------------------------------------------------- | ----------------- |
| Cruise target         | **8.0 m/s**                                               | 6.0 m/s           |
| Speed in corners      | `min(v_cruise, √(a_lat_max / |κ|))`, `a_lat_max = 5 m/s²` | Fixed 6.0 m/s     |
| Local path            | Cubic spline                                              | Linear            |
| Lateral control       | MPC, horizon 15 @ 50 Hz                                   | Stanley reactive  |
| MPC weights           | `q_ey=120`, `q_epsi=25`, `r_δ=0.08`, `r_Δδ=1.2`           | `k=1`, `k_soft=1` |
| Wheelbase (MPC model) | 2.966 m                                                   | —                 |


Same peak speed (~5.9 m/s) on both stacks: gains come from **higher average speed**, not a shorter track.

## Results

### Session laps


| Lap | Time         | Avg speed | Peak speed | Distance |
| --- | ------------ | --------- | ---------- | -------- |
| 1   | **115.70 s** | 5.64 m/s  | 5.87 m/s   | 652.7 m  |
| 2   | 113.20 s     | 5.76 m/s  | 5.89 m/s   | 651.9 m  |
| 3   | 114.60 s     | 5.69 m/s  | 5.91 m/s   | 652.2 m  |


Laps 2–3 show repeatability (mean 114.5 s, spread 2.5 s).

### Lap 1 — all stacks

Stanley: [`REPORT_BASELINE.md`](REPORT_BASELINE.md) / 2026-05-20. MPC & Pure Pursuit: lap 1, 2026-05-24.


| Metric     | Stanley | MPC          | Pure Pursuit |
| ---------- | ------- | ------------ | ------------ |
| Lap time   | 190.90 s | **115.70 s** | 121.60 s     |
| Avg speed  | 3.42 m/s | **5.64 m/s** | 5.38 m/s     |
| Peak speed | 5.85 m/s | 5.87 m/s     | **7.64 m/s** |
| Distance   | 652.66 m | 652.72 m     | 654.38 m     |


MPC and Pure Pursuit share the same local planner (cubic spline, 8 m/s cruise, curvature speed cap). MPC leads on lap time and cornering average; Pure Pursuit hits the highest peak on straights but is ~6 s slower overall.

### Roadmap (lap 1)


| Target              | Goal      | MPC     | Status  |
| ------------------- | --------- | ------- | ------- |
| Beat Stanley        | < 190.9 s | 115.7 s | Met     |
| Pure Pursuit target | < 120 s   | 115.7 s | Met     |
| Racing line target  | < 90 s    | 115.7 s | Not met |


## Reproduce

```bash
colcon build --packages-select autocar_nav autocar_nav_mpc launches --symlink-install
source install/setup.bash
ros2 launch launches race_mpc_launch.py
```

## Data

[`../results/`](../results/) — each run is `results/mpc_<run_id>/` (`params.yml`, `lap_times.csv`). Legacy flat `lap_times_mpc.csv` may still exist from older sessions.