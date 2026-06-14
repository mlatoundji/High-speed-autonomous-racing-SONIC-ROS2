# FAQ — `autocar_nav_pure_pursuit_lidar`

## What's the difference between `global_planner`, `local_planner`, `tracker` (Pure Pursuit), and the racing line?

In this LiDAR stack, four pieces sit in one pipeline at different layers. **Racing line** is not a node — it is data built by `global_planner_lidar`.

### Pipeline

```
Sensors + pose
      ↓
global_planner_lidar  →  /autocar/goals        (WHERE to go, short-term)
      ↓
local_planner         →  /autocar/path          (SMOOTH route + target speed)
      ↓
path_tracker          →  /autocar/auto_cmd_vel  (HOW to steer & throttle)
```

### Quick comparison


|                         | **global_planner** (`global_planner_lidar`) | **local_planner**                           | **tracker** (`path_tracker` / Pure Pursuit) | **racing line**                                       |
| ----------------------- | ------------------------------------------- | ------------------------------------------- | ------------------------------------------- | ----------------------------------------------------- |
| **What is it?**         | ROS node                                    | ROS node                                    | ROS node + `pure_pursuit.py` lib            | Cached polyline in memory (`rx_map`, `ry_map`)        |
| **Question it answers** | *Which waypoints ahead?*                    | *What smooth path & speed?*                 | *What steering & throttle now?*             | *What is the optimal full lap route?*                 |
| **Time horizon**        | Short window (~10 goals, ~30 m)             | Next ~10–20 m of spline                     | ~1.5–8 m lookahead                          | **Entire track** (closed loop)                        |
| **When active**         | Always (10 Hz)                              | Always (10 Hz path, 10 Hz speed)            | Always (50 Hz)                              | Built once at end of lap 1; used lap 2+               |
| **Uses sensors?**       | Yes — `/scan`, `/map`, pose                 | No — only goals + mode                      | No — only path + pose                       | Built from lap-1 driven path + SLAM map corridor      |
| **Output**              | `/autocar/goals`, `/autocar/nav_mode`       | `/autocar/path`, `/autocar/target_velocity` | `/autocar/auto_cmd_vel`                     | Internal; exposed as sliding `/autocar/goals` windows |
| **Plans steering?**     | No                                          | No                                          | **Yes** (Pure Pursuit)                      | No                                                    |
| **Sets target speed?**  | No                                          | **Yes** (mode cap + curvature limit)        | Follows `target_velocity` (± error scaling) | No                                                    |
| **Representation**      | Sparse **discrete** waypoints (~10+)        | Dense **discrete** points (~0.1 m spacing)  | Continuous `Twist` each tick                | Dense **discrete** polyline (map frame)               |


#### `/autocar/goals` vs `/autocar/path`

Both are **finite lists of points** in `Path2D` — not a continuous formula. `local_planner` fits a cubic spline through goals, then **samples** that smooth curve into a denser polyline.


|                       | `/autocar/goals`                          | `/autocar/path`                                                                  |
| --------------------- | ----------------------------------------- | -------------------------------------------------------------------------------- |
| **Publisher**         | `global_planner_lidar`                    | `local_planner`                                                                  |
| **Point count**       | Few (~10 anchors + goals)                 | Many (spline sample every `ds` ≈ 0.1 m)                                          |
| **Geometry**          | Polyline (straight segments if unsplined) | Polyline approximating a **smooth** spline                                       |
| **Looks continuous?** | No — sparse corners                       | Yes in RViz — dense enough to look like a curve                                  |
| **Pure Pursuit uses** | Indirectly (after splining)               | **Directly** — nearest point + arc-length lookahead (with segment interpolation) |


Denser `path` does not give Pure Pursuit more route “optimization” — it only tracks the upstream plan more faithfully. Tuning lap time or line shape happens in `global_planner_lidar` / racing line / `local_planner` speed caps, not in PP geometry.

### `global_planner_lidar` — strategic / waypoint planner

**Lap 1 (exploration):**

- Reads LiDAR + map → places a few corridor-centre goals ahead of the car.
- Records where the car actually drove (`_explore_path`).

**Lap 2+ (racing):**

- Uses the cached **racing line**.
- Publishes only a **small sliding window** of goals around the car (not the whole track every tick).

Think: **GPS giving the next few corners**, not redrawing the whole route every frame.

### `local_planner` — trajectory + speed planner

Takes sparse `/autocar/goals` and:

1. **Cubic spline** → dense smooth `/autocar/path` (~1 point per 0.1 m).
2. **Curvature limit** → `/autocar/target_velocity` (`exploration_velocity` or `cruise_velocity`).

Think: **draw a smooth racing line segment** and decide **how fast** it is safe to take it.

Does **not** steer the car.

### `path_tracker` (Pure Pursuit) — controller

Takes `/autocar/path` + `/autocar/target_velocity` and:

- Finds nearest point on path.
- Picks **one lookahead point** Ld ahead.
- Computes **steering** (`angular.z`) and **speed** (`linear.x`).

Think: **the driver** — hands on wheel, foot on pedal, following the line already drawn.

`pure_pursuit.py` is the math library; the **control loop runs in `path_tracker`**.

### Racing line — full-lap optimised route (data, not a node)

Built **once** when lap 1 finishes:

```
lap-1 driven path
  → close-loop resample (centerline)
  → SLAM map corridor bounds
  → min-curvature optimise
  → Laplacian smooth
  → snap + cleanup
  → cached in map frame (rx_map, ry_map)
```


|         | Exploration goals         | Racing line                     |
| ------- | ------------------------- | ------------------------------- |
| Scope   | ~30 m ahead, reactive     | Full circuit                    |
| Source  | Live LiDAR gaps           | Lap-1 trajectory + optimisation |
| Purpose | Find the track, build map | Fast, smooth lap 2+             |


`global_planner_lidar` **owns** the racing line. `local_planner` and `path_tracker` do not know whether goals came from LiDAR or the racing line — they spline and follow either way.

### Analogy


| Piece                    | Role                                       |
| ------------------------ | ------------------------------------------ |
| **Racing line**          | Pre-computed ideal lap (coach’s track map) |
| **global_planner**       | “Next 5 corners” navigator                 |
| **local_planner**        | Smooth the route + set corner speeds       |
| **Pure Pursuit tracker** | Driver executing steering & throttle       |


### Bottom line

- **global_planner** = *what waypoints to aim at*
- **local_planner** = *smooth path + target speed*
- **tracker** = *actual control commands*
- **racing line** = *the full optimised track*, built after lap 1 and fed through global_planner as goal windows on lap 2+

See [README.md](README.md) for full data-flow and lap-by-lap behaviour.