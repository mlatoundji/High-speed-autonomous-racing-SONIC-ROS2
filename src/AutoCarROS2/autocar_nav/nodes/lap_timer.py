#!/usr/bin/env python3
"""Lap timer for the race circuit.

Detects start/finish crossings and appends rows to
``results/<stack>_<run_id>/lap_times.csv``.

Publishes:
    /autocar/lap_time          (Float64)  -- last completed lap, in s
    /autocar/current_lap_time  (Float64)  -- elapsed time in the running lap
    /autocar/lap_count         (Int32)    -- number of completed laps
"""

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from autocar_msgs.msg import State2D
from autocar_nav.lap_times_paths import LAP_TIMES_CSV_FIELDS, init_lap_times_csv, lap_log_paths


CSV_FIELDS = list(LAP_TIMES_CSV_FIELDS)

START_X = 103.67
ROAD_HALF_WIDTH = 8.0

MIN_LAP_TIME_S = 5.0

LIVE_TIMER_HZ = 10.0


class LapTimer(Node):

    def __init__(self):
        super().__init__('lap_timer')

        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('stack', 'unknown', desc),
                ('run_id', '', desc),
                ('run_dir', '', desc),
                ('lap_times_csv', '', desc),
            ],
        )

        self.stack = str(self.get_parameter('stack').value)
        run_id = str(self.get_parameter('run_id').value).strip()
        run_dir = str(self.get_parameter('run_dir').value).strip()
        csv_override = str(self.get_parameter('lap_times_csv').value).strip()

        lap_times_csv = csv_override or None
        if not lap_times_csv and run_dir:
            lap_times_csv = str(Path(run_dir) / 'lap_times.csv')

        self._csv_targets, self._in_project_repo = lap_log_paths(
            self.stack,
            run_dir=run_dir or None,
            lap_times_csv=lap_times_csv,
        )

        if not self._csv_targets:
            raise RuntimeError(
                f'lap_timer: unknown stack {self.stack!r} or missing run_dir. '
                f'Use one of: stanley, mpc, pure_pursuit.')

        self.session_id = run_id or datetime.now().strftime('%Y-%m-%dT%H-%M-%S')
        for path in self._csv_targets:
            path.parent.mkdir(parents=True, exist_ok=True)
            init_lap_times_csv(path)

        if not self._in_project_repo:
            self.get_logger().warn(
                'Could not locate repo results/. '
                'Lap CSV: %s. Set AUTOCAR_REPO_ROOT to your repo root.' % (
                    self._csv_targets[0],
                ),
            )

        self.sub = self.create_subscription(
            State2D, '/autocar/state2D', self.state_cb, 10)
        self.lap_pub = self.create_publisher(Float64, '/autocar/lap_time', 10)
        self.current_pub = self.create_publisher(Float64, '/autocar/current_lap_time', 10)
        self.count_pub = self.create_publisher(Int32, '/autocar/lap_count', 10)

        self.prev_x = None
        self.prev_y = None
        self.lap_count = 0
        self.lap_start_time = None
        self.dist_accum = 0.0
        self.max_speed = 0.0
        self.best_lap = None

        self.timer = self.create_timer(1.0 / LIVE_TIMER_HZ, self._publish_live)

        run_label = self._csv_targets[0].parent.name
        self.get_logger().info(
            f'Lap timer armed (stack={self.stack}, run={run_label}). '
            f'Start/finish: x={START_X:.2f}, '
            f'y in [{-ROAD_HALF_WIDTH:+.1f}, {ROAD_HALF_WIDTH:+.1f}], +Y crossing. '
            f'CSV: {self._csv_targets[0]}'
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

        row = [
            self.session_id,
            self.lap_count,
            datetime.now().isoformat(timespec='seconds'),
            f'{elapsed:.3f}',
            f'{avg_speed:.3f}',
            f'{self.max_speed:.3f}',
            f'{self.dist_accum:.2f}',
        ]

        for csv_path in self._csv_targets:
            with csv_path.open('a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(row)

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
