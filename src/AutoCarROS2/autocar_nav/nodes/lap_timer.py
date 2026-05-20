#!/usr/bin/env python3
"""Lap timer for the race-circuit baseline.

Detects when the car crosses the start/finish line at x=103.67, y=0 (the
first waypoint) and logs the lap time. The start line is a segment along
the +x axis spanning the road width, crossed in the +Y direction (which
is the loop's counter-clockwise tangent at that point).

Publishes:
    /autocar/lap_time          (Float64)  -- last completed lap, in s
    /autocar/current_lap_time  (Float64)  -- elapsed time in the running lap
    /autocar/lap_count         (Int32)    -- number of completed laps

Persists every completed lap as a CSV row in
    ~/.ros/autocar_lap_times.csv
The file is appended to across runs; each run is grouped by a single
session_id (ISO timestamp of when the node started).
"""

import csv
import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from autocar_msgs.msg import State2D


CSV_PATH = Path(os.path.expanduser('~/.ros/autocar_lap_times.csv'))
CSV_FIELDS = [
    'session_id', 'lap_number', 'timestamp_iso',
    'duration_s', 'avg_speed_mps', 'max_speed_mps', 'distance_m',
]


START_X = 103.67
ROAD_HALF_WIDTH = 8.0

MIN_LAP_TIME_S = 5.0

# Rate at which the live timer is republished for HUD consumers.
LIVE_TIMER_HZ = 10.0


class LapTimer(Node):

    def __init__(self):
        super().__init__('lap_timer')

        self.sub = self.create_subscription(
            State2D, '/autocar/state2D', self.state_cb, 10
        )
        self.lap_pub = self.create_publisher(Float64, '/autocar/lap_time', 10)
        self.current_pub = self.create_publisher(Float64, '/autocar/current_lap_time', 10)
        self.count_pub = self.create_publisher(Int32, '/autocar/lap_count', 10)

        self.prev_x = None
        self.prev_y = None
        self.lap_count = 0
        self.lap_start_time = None
        self.dist_accum = 0.0
        self.max_speed = 0.0
        self.best_lap = None  # seconds

        self.timer = self.create_timer(1.0 / LIVE_TIMER_HZ, self._publish_live)

        # CSV setup. Tag every row in this run with one session_id so we
        # can group laps per launch when analysing the file later.
        self.session_id = datetime.now().strftime('%Y-%m-%dT%H-%M-%S')
        CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
        if not CSV_PATH.exists():
            with CSV_PATH.open('w', newline='') as f:
                csv.writer(f).writerow(CSV_FIELDS)

        self.get_logger().info(
            f'Lap timer armed. Start/finish line: x={START_X:.2f}, '
            f'y in [{-ROAD_HALF_WIDTH:+.1f}, {ROAD_HALF_WIDTH:+.1f}], '
            f'crossing direction: +Y. CSV log: {CSV_PATH}'
        )

    def state_cb(self, msg: State2D):
        x = msg.pose.x
        y = msg.pose.y
        speed = math.hypot(msg.twist.x, msg.twist.y)

        now = self.get_clock().now()

        if self.prev_x is None:
            self.prev_x, self.prev_y = x, y
            return

        self.dist_accum += math.hypot(x - self.prev_x, y - self.prev_y)
        if speed > self.max_speed:
            self.max_speed = speed

        if self.prev_y < 0.0 <= y and abs(x - START_X) < ROAD_HALF_WIDTH * 2:
            t = -self.prev_y / (y - self.prev_y) if (y - self.prev_y) != 0 else 0.0
            cross_x = self.prev_x + t * (x - self.prev_x)
            if cross_x > START_X - ROAD_HALF_WIDTH * 2:
                self._on_crossing(now)

        self.prev_x, self.prev_y = x, y

    def _publish_live(self):
        # Always publish the lap count so the HUD shows 0 before the first
        # crossing, then 1, 2, ...
        count = Int32()
        count.data = self.lap_count
        self.count_pub.publish(count)

        if self.lap_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.lap_start_time).nanoseconds * 1e-9
        msg = Float64()
        msg.data = elapsed
        self.current_pub.publish(msg)

    def _on_crossing(self, now):
        if self.lap_start_time is None:
            self.lap_start_time = now
            self.dist_accum = 0.0
            self.max_speed = 0.0
            self.get_logger().info('Start line crossed -- lap 1 begins.')
            return

        elapsed = (now - self.lap_start_time).nanoseconds * 1e-9
        if elapsed < MIN_LAP_TIME_S:
            return

        self.lap_count += 1
        avg_speed = self.dist_accum / elapsed if elapsed > 0 else 0.0
        if self.best_lap is None or elapsed < self.best_lap:
            self.best_lap = elapsed
            best_tag = '  [NEW BEST]'
        else:
            best_tag = f'  (best {self.best_lap:.2f}s)'

        self.get_logger().info(
            f'Lap {self.lap_count} completed: {elapsed:.2f} s '
            f'(avg {avg_speed:.2f} m/s, max {self.max_speed:.2f} m/s, '
            f'dist {self.dist_accum:.1f} m){best_tag}'
        )

        msg = Float64()
        msg.data = elapsed
        self.lap_pub.publish(msg)

        # Append to CSV.
        with CSV_PATH.open('a', newline='') as f:
            csv.writer(f).writerow([
                self.session_id,
                self.lap_count,
                datetime.now().isoformat(timespec='seconds'),
                f'{elapsed:.3f}',
                f'{avg_speed:.3f}',
                f'{self.max_speed:.3f}',
                f'{self.dist_accum:.2f}',
            ])

        self.lap_start_time = now
        self.dist_accum = 0.0
        self.max_speed = 0.0


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LapTimer()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
