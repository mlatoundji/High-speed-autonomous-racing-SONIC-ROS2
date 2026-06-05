#!/usr/bin/env python3
"""Lap timer and per-lap metrics for the race circuit.

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
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from autocar_msgs.msg import State2D
from autocar_nav.lap_times_paths import LAP_TIMES_CSV_FIELDS, init_lap_times_csv, lap_log_paths


CSV_FIELDS = list(LAP_TIMES_CSV_FIELDS)

DEFAULT_FINISH_MODE = 'pos_y'
DEFAULT_FINISH_LINE_X = 103.67
DEFAULT_FINISH_Y_CENTER = 0.0
DEFAULT_FINISH_Y_HALF_WIDTH = 8.0

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
                ('profile', 'default', desc),
                ('latency_ms', 0, desc),
                ('odom_noise_std', 0.0, desc),
                ('offtrack_threshold_m', 4.0, desc),
                ('finish_mode', DEFAULT_FINISH_MODE, desc),
                ('finish_line_x', DEFAULT_FINISH_LINE_X, desc),
                ('finish_y_center', DEFAULT_FINISH_Y_CENTER, desc),
                ('finish_y_half_width', DEFAULT_FINISH_Y_HALF_WIDTH, desc),
            ],
        )

        self.stack = str(self.get_parameter('stack').value)
        run_id = str(self.get_parameter('run_id').value).strip()
        run_dir = str(self.get_parameter('run_dir').value).strip()
        csv_override = str(self.get_parameter('lap_times_csv').value).strip()
        self.profile = str(self.get_parameter('profile').value)
        self.latency_ms = int(self.get_parameter('latency_ms').value)
        self.odom_noise_std = float(self.get_parameter('odom_noise_std').value)
        self.offtrack_threshold = float(self.get_parameter('offtrack_threshold_m').value)
        self.finish_mode = str(self.get_parameter('finish_mode').value)
        self.finish_line_x = float(self.get_parameter('finish_line_x').value)
        self.finish_y_center = float(self.get_parameter('finish_y_center').value)
        self.finish_y_half_width = float(self.get_parameter('finish_y_half_width').value)

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

        self.create_subscription(State2D, '/autocar/state2D', self.state_cb, 10)
        self.create_subscription(Float64, '/autocar/lateral_error', self.lateral_error_cb, 10)
        self.create_subscription(Twist, '/autocar/cmd_vel', self.cmd_vel_cb, 10)

        self.lap_pub = self.create_publisher(Float64, '/autocar/lap_time', 10)
        self.current_pub = self.create_publisher(Float64, '/autocar/current_lap_time', 10)
        self.count_pub = self.create_publisher(Int32, '/autocar/lap_count', 10)

        self.prev_x = None
        self.prev_y = None
        self.lap_count = 0
        self.lap_start_time = None
        self.best_lap = None
        self.prev_steer = None
        self.prev_steer_time = None

        self._reset_lap_accumulators()

        self.timer = self.create_timer(1.0 / LIVE_TIMER_HZ, self._publish_live)

        run_label = self._csv_targets[0].parent.name
        self.get_logger().info(
            f'Lap timer armed (stack={self.stack}, run={run_label}). '
            f'profile={self.profile} latency_ms={self.latency_ms} '
            f'odom_noise_std={self.odom_noise_std:.3f}. '
            f'Start/finish: mode={self.finish_mode}, '
            f'x={self.finish_line_x:.2f}, y_center={self.finish_y_center:.2f}, '
            f'y_half_width={self.finish_y_half_width:.1f}. '
            f'CSV: {self._csv_targets[0]}'
        )

    def _reset_lap_accumulators(self):
        self.dist_accum = 0.0
        self.max_speed = 0.0
        self.lat_err_sq_sum = 0.0
        self.lat_err_count = 0
        self.lat_err_max = 0.0
        self.steering_rate_max = 0.0
        self.offtrack_events = 0
        self._was_offtrack = False

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

        if self._finish_line_crossed(self.prev_x, self.prev_y, x, y):
            self._on_crossing(now)

        self.prev_x, self.prev_y = x, y

    def lateral_error_cb(self, msg: Float64):
        if self.lap_start_time is None:
            return
        e = float(msg.data)
        abs_e = abs(e)
        self.lat_err_sq_sum += e * e
        self.lat_err_count += 1
        if abs_e > self.lat_err_max:
            self.lat_err_max = abs_e

        is_offtrack = abs_e > self.offtrack_threshold
        if is_offtrack and not self._was_offtrack:
            self.offtrack_events += 1
        self._was_offtrack = is_offtrack

    def cmd_vel_cb(self, msg: Twist):
        steer = float(msg.angular.z)
        t = self.get_clock().now()
        if (
            self.prev_steer is not None
            and self.prev_steer_time is not None
            and self.lap_start_time is not None
        ):
            dt = (t - self.prev_steer_time).nanoseconds * 1e-9
            if dt > 1e-6:
                rate = abs(steer - self.prev_steer) / dt
                if rate > self.steering_rate_max:
                    self.steering_rate_max = rate
        self.prev_steer = steer
        self.prev_steer_time = t

    def _finish_line_crossed(self, prev_x, prev_y, x, y):
        tol = self.finish_y_half_width
        if self.finish_mode == 'neg_x':
            if not (prev_x > self.finish_line_x >= x):
                return False
            if abs(y - self.finish_y_center) > tol:
                return False
            if abs(prev_y - self.finish_y_center) > tol:
                return False
            return True

        if self.finish_mode == 'pos_x':
            if not (prev_x < self.finish_line_x <= x):
                return False
            if abs(y - self.finish_y_center) > tol:
                return False
            if abs(prev_y - self.finish_y_center) > tol:
                return False
            return True

        # Default: +Y crossing through y=finish_y_center near finish_line_x.
        if not (prev_y < self.finish_y_center <= y):
            return False
        t = -prev_y / (y - prev_y) if (y - prev_y) != 0 else 0.0
        cross_x = prev_x + t * (x - prev_x)
        return abs(cross_x - self.finish_line_x) < tol * 2

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
            self._reset_lap_accumulators()
            self.get_logger().info('Start line crossed -- lap 1 begins.')
            return

        elapsed = (now - self.lap_start_time).nanoseconds * 1e-9
        if elapsed < MIN_LAP_TIME_S:
            return

        self.lap_count += 1
        avg_speed = self.dist_accum / elapsed if elapsed > 0 else 0.0
        lat_rms = (
            math.sqrt(self.lat_err_sq_sum / self.lat_err_count)
            if self.lat_err_count else 0.0
        )

        if self.best_lap is None or elapsed < self.best_lap:
            self.best_lap = elapsed
            best_tag = '  [NEW BEST]'
        else:
            best_tag = f'  (best {self.best_lap:.2f}s)'

        self.get_logger().info(
            f'Lap {self.lap_count} completed: {elapsed:.2f} s '
            f'(avg {avg_speed:.2f} m/s, max {self.max_speed:.2f} m/s, '
            f'dist {self.dist_accum:.1f} m, lat_rms {lat_rms:.3f} m, '
            f'lat_max {self.lat_err_max:.3f} m, '
            f'steer_rate_max {self.steering_rate_max:.3f} rad/s, '
            f'offtrack {self.offtrack_events}){best_tag}'
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
            self.stack,
            self.profile,
            self.latency_ms,
            f'{self.odom_noise_std:.4f}',
            f'{lat_rms:.4f}',
            f'{self.lat_err_max:.4f}',
            f'{self.steering_rate_max:.4f}',
            self.offtrack_events,
        ]

        for csv_path in self._csv_targets:
            with csv_path.open('a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(row)

        self.lap_start_time = now
        self._reset_lap_accumulators()


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
