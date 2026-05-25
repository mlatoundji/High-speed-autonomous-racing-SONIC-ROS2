# Baseline — Stanley, conservative settings

Reference lap for the project. Recorded **2026-05-20**, session `2026-05-20T22-07-12`, lap 1. All later controllers are compared against this run.

## Configuration


| Item       | Value                                                                                                          |
| ---------- | -------------------------------------------------------------------------------------------------------------- |
| World      | `[race_circuit.world](../src/AutoCarROS2/autocar_gazebo/worlds/race_circuit.world)` — ~103 m radius, 16 m wide |
| Vehicle    | `autocar`, 1580 kg, Ackermann                                                                                  |
| Trajectory | Centerline `[waypoints.csv](../src/AutoCarROS2/autocar_nav/data/waypoints.csv)`                                |
| Controller | Stanley `[tracker.py](../src/AutoCarROS2/autocar_nav/nodes/tracker.py)`                                        |
| Launch     | `ros2 launch launches race_launch.py`                                                                          |
| Config     | `[navigation_params.yaml](../src/AutoCarROS2/autocar_nav/config/navigation_params.yaml)`                       |


### Key parameters


| Module             | Setting                                                  |
| ------------------ | -------------------------------------------------------- |
| Cruise speed       | **6.0 m/s** (`CRUISE_VEL` in `localplanner.py`)          |
| Stanley gains      | `k = 1.0`, `k_soft = 1.0`, max steer **0.95 rad**, 50 Hz |
| Global planner     | `waypoints_ahead = 3`, `behind = 2`, centerline only     |
| Local path         | Linear segments between goals                            |
| Obstacle avoidance | Lateral offsets up to ±6 m (unused on this run)          |
| Lap timer          | Line at `x = 103.67`, ±8 m width, +Y crossing            |


Steering: `δ = ψ + atan(k · e / (k_soft + v))`.

## Result (lap 1)


| Metric        | Value                        |
| ------------- | ---------------------------- |
| Lap time      | **190.90 s** (3 min 11 s)    |
| Average speed | 3.42 m/s                     |
| Peak speed    | 5.85 m/s                     |
| Distance      | 652.66 m (~650 m centerline) |


Distance matches the centerline: lateral tracking is good. Lap time is limited mainly by **slow longitudinal acceleration** (Ackermann PID on 1580 kg): the car rarely holds 6.0 m/s through turns.

## Roadmap targets


| Target                         | Goal        |
| ------------------------------ | ----------- |
| Pure Pursuit + speed profile   | < **120 s** |
| Racing line + tuned controller | < **90 s**  |


## Reproduce

```bash
colcon build --packages-select autocar_nav launches
source install/setup.bash
ros2 launch launches race_launch.py
```

## Data

Frozen: `[../results/lap_times_baseline.csv](../results/lap_times_baseline.csv)`  
Live Stanley logs: `[../results/`](../results/) — `results/stanley_<run_id>/` per session