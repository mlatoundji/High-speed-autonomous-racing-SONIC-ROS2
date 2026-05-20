# Baseline -- Stanley controller, conservative settings

First reference lap-time, recorded before any racing-line / advanced
controller work. To beat this number is the goal of the rest of the
roadmap (sections 3-4 of the README).

## Setup

- World: [`race_circuit.world`](../src/AutoCarROS2/autocar_gazebo/worlds/race_circuit.world) -- closed-loop track, radius ~103 m, 16 m wide, 12 obstacle slots and 48 hay bales bordering the road.
- Vehicle: stock `autocar` (1580 kg chassis, Ackermann steering).
- Tracker: existing Stanley controller in [`tracker.py`](../src/AutoCarROS2/autocar_nav/nodes/tracker.py).
- Target speed: `CRUISE_VEL = 6.0` m/s in [`localplanner.py`](../src/AutoCarROS2/autocar_nav/nodes/localplanner.py).
- Obstacle avoidance: enabled in the local planner (no on-road obstacles in this run, so no deviations occurred).
- Launch: `ros2 launch launches race_launch.py`.

## Result (lap 1, session 2026-05-20T22-07-12)

| Metric                | Value           |
|-----------------------|-----------------|
| Lap time              | **190.90 s** (3 min 11 s) |
| Average speed         | 3.42 m/s (12.3 km/h) |
| Peak speed            | 5.85 m/s (21.1 km/h) |
| Distance travelled    | 652.66 m        |
| Theoretical centerline length | ~650 m  |

The travelled distance matches the centerline almost perfectly, meaning the
Stanley tracker holds the racing line tightly. The performance gap is in
**longitudinal acceleration**: the Ackermann plugin's PID gain of 4000 on a
1580 kg vehicle gives a sluggish ramp-up after every turn, so the car
spends most of the lap below `CRUISE_VEL` even though it tops out near it
in the few long straights.

## What to beat

- **Lap time** under **120 s** with a Pure-Pursuit controller + speed
  profile that holds the cruise target through turns (section 4 of
  README roadmap).
- **Lap time** under **90 s** with a proper racing line (section 3) +
  tuned controller (section 4) + acceleration ramp.

## Raw data

See [`baseline_lap_times.csv`](baseline_lap_times.csv). The active CSV log
lives at `~/.ros/autocar_lap_times.csv` and accumulates every future run;
this file is a frozen copy of the baseline for reproducibility.
