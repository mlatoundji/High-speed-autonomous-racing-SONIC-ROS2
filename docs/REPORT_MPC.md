# MPC — `autocar_nav_mpc` vs Stanley baseline

Compared to [`REPORT_BASELINE.md`](REPORT_BASELINE.md) / [`baseline_2026-05-20T22-07-12`](../results/baseline_2026-05-20T22-07-12/lap_times.csv).  
Recorded **2026-05-25** — centerline [`mpc_2026-05-25T16-46-36`](../results/mpc_2026-05-25T16-46-36/), racing [`mpc_racing_2026-05-25T16-56-47`](../results/mpc_racing_2026-05-25T16-56-47/).  
Same-day Stanley references: centerline [`stanley_2026-05-25T13-55-37`](../results/stanley_2026-05-25T13-55-37/), racing [`stanley_racing_2026-05-25T13-48-10`](../results/stanley_racing_2026-05-25T13-48-10/).

## Configuration


| Item               | MPC                                                                                                              | Stanley baseline                                                                                         |
| ------------------ | ---------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| World / trajectory | Same `race_circuit.world`, centerline or racing waypoints                                                        | Same                                                                                                     |
| Controller         | Frenet linear MPC [`tracker.py`](../src/AutoCarROS2/autocar_nav_mpc/nodes/tracker.py)                            | Stanley                                                                                                  |
| Local planner      | Cubic spline + curvature speed limit                                                                             | Linear path, fixed 6.0 m/s                                                                               |
| Launch             | `ros2 launch launches race_mpc_launch.py`                                                                        | `race_launch.py`                                                                                         |
| Config             | [`autocar_nav_mpc/.../navigation_params.yaml`](../src/AutoCarROS2/autocar_nav_mpc/config/navigation_params.yaml) | [`autocar_nav/.../navigation_params.yaml`](../src/AutoCarROS2/autocar_nav/config/navigation_params.yaml) |


### What changes vs baseline


| Parameter             | MPC                                                       | Stanley           |
| --------------------- | --------------------------------------------------------- | ----------------- |
| Cruise target         | **8.0 m/s**                                               | 6.0 m/s           |
| Speed in corners      | `min(v_cruise, √(a_lat_max / \|κ\|))`, `a_lat_max = 5 m/s²` | Fixed 6.0 m/s     |
| Local path            | Cubic spline                                              | Linear            |
| Lateral control       | MPC, horizon 15 @ 50 Hz                                   | Stanley reactive  |
| MPC weights           | `q_ey=120`, `q_epsi=25`, `r_δ=0.08`, `r_Δδ=1.2`           | `k=1`, `k_soft=1` |
| Wheelbase (MPC model) | 2.966 m                                                   | —                 |


Gains vs Stanley come mainly from **higher average speed** on the spline speed profile; peak speed on these runs reaches **~7.5 m/s** (curvature cap + straights), not the ~5.9 m/s ceiling seen on the legacy Stanley baseline lap.

## Results

### Centerline — `mpc_2026-05-25T16-46-36`


| Lap | Time         | Avg speed | Peak speed | Distance |
| --- | ------------ | --------- | ---------- | -------- |
| 1   | 113.20 s     | 5.78 m/s  | 7.46 m/s   | 653.9 m  |
| 2   | 111.10 s     | 5.89 m/s  | 7.58 m/s   | 654.0 m  |
| 3   | **108.90 s** | 6.00 m/s  | 7.41 m/s   | 653.6 m  |


Laps 2–3 improve as the session warms up (mean **110.0 s**, spread 2.2 s). Best lap **108.9 s**.

### Racing line — `mpc_racing_2026-05-25T16-56-47`


| Lap | Time         | Avg speed | Peak speed | Distance |
| --- | ------------ | --------- | ---------- | -------- |
| 1   | 113.00 s     | 5.58 m/s  | 7.31 m/s   | 630.6 m  |
| 2   | **108.50 s** | 5.80 m/s  | 7.51 m/s   | 629.7 m  |
| 3   | 110.40 s     | 5.71 m/s  | 7.45 m/s   | 630.2 m  |
| 4   | 110.80 s     | 5.69 m/s  | 7.49 m/s   | 630.3 m  |
| 5   | 109.10 s     | 5.78 m/s  | 7.56 m/s   | 630.1 m  |


Racing line shortens the lap (~630 m vs ~654 m centerline). After lap 1, laps 2–5 cluster around **109.7 s** (best **108.5 s**), ~**4.4 s** faster than the best centerline lap in this session.

### Centerline — lap 1 vs other stacks (2026-05-25)


| Metric     | Stanley (13-55-37) | MPC (16-46-36) | Pure Pursuit (16-28-02) |
| ---------- | ------------------ | -------------- | ----------------------- |
| Lap time   | 191.30 s           | **113.20 s**   | 99.90 s                 |
| Avg speed  | 3.41 m/s           | **5.78 m/s**   | 6.55 m/s                |
| Peak speed | 5.83 m/s           | 7.46 m/s       | **7.60 m/s**            |
| Distance   | 652.5 m            | 653.9 m        | 654.1 m                 |


Frozen baseline Stanley (2026-05-20) remains **190.90 s** for historical comparison. Pure Pursuit’s lap 1 here is unusually fast vs its own laps 2–3 (~107–109 s); treat MPC vs PP on lap 1 with that caveat.

### Racing line — best lap vs Stanley racing (2026-05-25)


| Metric     | Stanley racing | MPC racing | Pure Pursuit racing |
| ---------- | -------------- | ---------- | ------------------- |
| Best lap   | 172.60 s (L1)  | **108.50 s** (L2) | **92.50 s** (L3) |
| Avg speed  | 3.64 m/s (L1)  | 5.80 m/s   | 6.80 m/s            |
| Peak speed | 5.83 m/s       | 7.51 m/s   | **7.67 m/s**        |
| Distance   | 628.3 m        | 629.7 m    | 629.0 m             |


Stanley racing gain over centerline (same day): **~19 s** (191.3 s → 172.6 s on lap 1). MPC racing gain over MPC centerline best: **~0.4 s** in this session — most of the MPC advantage is already in the spline planner, not the line switch.

### Roadmap (best lap, centerline / racing)


| Target              | Goal      | MPC (2026-05-25) | Status  |
| ------------------- | --------- | ---------------- | ------- |
| Beat Stanley        | < 190.9 s | 108.9 s (CL)     | Met     |
| Pure Pursuit target | < 120 s   | 108.9 s (CL)     | Met     |
| Racing line target  | < 90 s    | 108.5 s (racing) | Not met |


## Reproduce

From repo root (see [`docs/README.md`](README.md) for all stacks and batch configs).

**MPC, centerline**:

```bash
colcon build --packages-select autocar_racing_line autocar_nav autocar_nav_mpc launches --symlink-install
source install/setup.bash
ros2 launch launches race_mpc_launch.py line:=centerline profile:=default latency_ms:=0 odom_noise_std:=0.0
```

**MPC, racing line**:

```bash
ros2 launch launches race_mpc_launch.py line:=racing profile:=default latency_ms:=0 odom_noise_std:=0.0
```

**Batch** (3 laps, 1 warmup):

```bash
pip install -r scripts/requirements.txt
cat > /tmp/mpc_centerline.yaml <<'EOF'
- stack: mpc
  line: centerline
  lap_count: 3
  warmup_laps: 1
EOF
python3 scripts/benchmark.py --config /tmp/mpc_centerline.yaml
```

## Data

| Session | Line | Folder |
| ------- | ---- | ------ |
| `2026-05-25T16-46-36` | centerline | [`results/mpc_2026-05-25T16-46-36/`](../results/mpc_2026-05-25T16-46-36/) |
| `2026-05-25T16-56-47` | racing | [`results/mpc_racing_2026-05-25T16-56-47/`](../results/mpc_racing_2026-05-25T16-56-47/) |

Each folder: `params.yaml`, `lap_times.csv`. Older 2026-05-24 sessions may still exist under [`results/`](../results/) with legacy flat CSV names.
