# Pure Pursuit — `autocar_nav_pure_pursuit` vs Stanley baseline

Compared to [`REPORT_BASELINE.md`](REPORT_BASELINE.md) / [`baseline_2026-05-20T22-07-12`](../results/baseline_2026-05-20T22-07-12/lap_times.csv).  
Recorded **2026-05-25** — centerline [`pure_pursuit_2026-05-25T16-28-02`](../results/pure_pursuit_2026-05-25T16-28-02/), racing [`pure_pursuit_racing_2026-05-25T16-34-12`](../results/pure_pursuit_racing_2026-05-25T16-34-12/).  
Same-day Stanley: [`stanley_2026-05-25T13-55-37`](../results/stanley_2026-05-25T13-55-37/), racing [`stanley_racing_2026-05-25T13-48-10`](../results/stanley_racing_2026-05-25T13-48-10/). MPC: [`REPORT_MPC.md`](REPORT_MPC.md).

## Configuration


| Item               | Pure Pursuit                                                                                                              | Stanley baseline                                                                                         |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| World / trajectory | Same `race_circuit.world`, centerline or racing waypoints                                                                 | Same                                                                                                     |
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


Peak speed on these runs is **~7.6–7.7 m/s** — higher than Stanley (~5.8 m/s) and similar to MPC. Lap time is still dominated by **average speed** through corners, not straight-line peak alone.

## Results

### Centerline — `pure_pursuit_2026-05-25T16-28-02`


| Lap | Time         | Avg speed | Peak speed | Distance |
| --- | ------------ | --------- | ---------- | -------- |
| 1   | 99.90 s      | 6.55 m/s  | 7.60 m/s   | 654.1 m  |
| 2   | 107.20 s     | 6.12 m/s  | 7.56 m/s   | 656.4 m  |
| 3   | 109.00 s     | 6.03 m/s  | 7.58 m/s   | 657.2 m  |


Lap 1 is **~7 s faster** than laps 2–3 (mean **108.1 s**). Likely a first-lap / session-start effect rather than the steady-state pace; use laps 2–3 for repeatability.

### Racing line — `pure_pursuit_racing_2026-05-25T16-34-12`


| Lap | Time         | Avg speed | Peak speed | Distance |
| --- | ------------ | --------- | ---------- | -------- |
| 1   | 96.80 s      | 6.51 m/s  | 7.52 m/s   | 629.7 m  |
| 2   | 95.70 s      | 6.58 m/s  | 7.50 m/s   | 629.5 m  |
| 3   | **92.50 s**  | 6.80 m/s  | **7.67 m/s** | 629.0 m  |


Racing line is **stable** on this session (contrast with earlier need_for_speed notes on PP + racing coupling). Best lap **92.5 s** — ~**14.7 s** faster than centerline lap 2, ~**80 s** faster than Stanley centerline (same day).

### Centerline — lap 1 vs other stacks (2026-05-25)


| Metric     | Stanley (13-55-37) | MPC (16-46-36) | Pure Pursuit (16-28-02) |
| ---------- | ------------------ | -------------- | ----------------------- |
| Lap time   | 191.30 s           | 113.20 s       | **99.90 s** (L1)        |
| Avg speed  | 3.41 m/s           | 5.78 m/s       | **6.55 m/s**            |
| Peak speed | 5.83 m/s           | 7.46 m/s       | **7.60 m/s**            |
| Distance   | 652.5 m            | 653.9 m        | 654.1 m                 |


On **warmed laps** (PP L2–3 vs MPC L2–3): PP **107–109 s**, MPC **109–111 s** — similar band; PP lap 1 alone is not a fair single-lap headline vs MPC.

### Racing line — best lap vs Stanley / MPC (2026-05-25)


| Metric     | Stanley racing | MPC racing | Pure Pursuit racing |
| ---------- | -------------- | ---------- | ------------------- |
| Best lap   | 172.60 s (L1)  | 108.50 s (L2) | **92.50 s** (L3) |
| Avg speed  | 3.64 m/s (L1)  | 5.80 m/s   | **6.80 m/s**        |
| Peak speed | 5.83 m/s       | 7.51 m/s   | **7.67 m/s**        |
| Distance   | 628.3 m        | 629.7 m    | 629.0 m             |


Pure Pursuit is the **fastest stack** on racing line in this dataset; MPC is next; Stanley remains ~80 s behind PP on lap time.

### Roadmap (best representative lap)


| Target              | Goal      | Pure Pursuit (2026-05-25) | Status        |
| ------------------- | --------- | ------------------------- | ------------- |
| Beat Stanley        | < 190.9 s | 92.5 s (racing)           | Met           |
| Pure Pursuit target | < 120 s   | 92.5 s (racing)           | Met           |
| Racing line target  | < 90 s    | 92.5 s (racing)           | Not met (+2.5 s) |


Centerline warmed laps (107–109 s) also beat the < 120 s PP target.

## Reproduce

From repo root (see [`docs/README.md`](README.md) for all stacks and batch configs).

**Pure Pursuit, centerline**:

```bash
colcon build --packages-select autocar_racing_line autocar_nav autocar_nav_pure_pursuit launches --symlink-install
source install/setup.bash
ros2 launch launches race_pure_pursuit_launch.py line:=centerline profile:=default latency_ms:=0 odom_noise_std:=0.0
```

**Pure Pursuit, racing line**:

```bash
ros2 launch launches race_pure_pursuit_launch.py line:=racing profile:=default latency_ms:=0 odom_noise_std:=0.0
```

**Batch**:

```bash
pip install -r scripts/requirements.txt
python3 scripts/benchmark.py --config scripts/configs/r1_pp_vs_stanley.yaml
python3 scripts/benchmark.py --config scripts/configs/r3_pp_racing_optional.yaml
```

## Data

| Session | Line | Folder |
| ------- | ---- | ------ |
| `2026-05-25T16-28-02` | centerline | [`results/pure_pursuit_2026-05-25T16-28-02/`](../results/pure_pursuit_2026-05-25T16-28-02/) |
| `2026-05-25T16-34-12` | racing | [`results/pure_pursuit_racing_2026-05-25T16-34-12/`](../results/pure_pursuit_racing_2026-05-25T16-34-12/) |

Each folder: `params.yaml`, `lap_times.csv`.
