#!/usr/bin/env python3
"""Race telemetry recorder (originally just a lap timer).

Detects when the car crosses the start/finish line at x=103.67, y=0 and
aggregates per-lap performance metrics. One row per completed lap is
appended to a single CSV that is the project's source of truth for
experimental results.

Topics in:
    /autocar/state2D         autocar_msgs/State2D  -- pose & twist
    /autocar/lateral_error   std_msgs/Float64      -- signed cross-track
                                                     error from controller
    /autocar/cmd_vel         geometry_msgs/Twist   -- to compute steering
                                                     rate stats

Topics out (live, for HUD/debug):
    /autocar/lap_time          Float64  -- last completed lap, in s
    /autocar/current_lap_time  Float64  -- elapsed time in the running lap
    /autocar/lap_count         Int32    -- number of completed laps

Parameters:
    controller             str   (e.g. 'stanley' | 'pure_pursuit' | 'mpc')
    profile                str   (e.g. 'default' | 'conservative' | ...)
    latency_ms             int   (artificial latency applied, 0 if off)
    odom_noise_std         float (std on /autocar/state2D, 0 if off)
    offtrack_threshold_m   float (default 4.0 m, |lateral_error| above
                                  which a rising-edge is counted)

CSV: ~/.ros/autocar_lap_times.csv (append-only). The legacy 7-column
schema is migrated to the new 15-column schema once on startup; historical
rows are padded with empty cells so the file stays uniform.
"""

import csv
import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from autocar_msgs.msg import State2D


CSV_PATH = Path(os.path.expanduser('~/.ros/autocar_lap_times.csv'))

LEGACY_FIELDS = [
    'session_id', 'lap_number', 'timestamp_iso',
    'duration_s', 'avg_speed_mps', 'max_speed_mps', 'distance_m',
]

EXTRA_FIELDS = [
    'controller', 'profile', 'latency_ms', 'odom_noise_std',
    'lateral_error_rms', 'lateral_error_max',
    'steering_rate_max', 'offtrack_events',
]

CSV_FIELDS = LEGACY_FIELDS + EXTRA_FIELDS


START_X = 103.67
ROAD_HALF_WIDTH = 8.0

MIN_LAP_TIME_S = 5.0

LIVE_TIMER_HZ = 10.0


class LapTimer(Node):

    def __init__(self):
        super().__init__('lap_timer')

        self.create_subscription(State2D, '/autocar/state2D', self.state_cb, 10)
        self.create_subscription(Float64, '/autocar/lateral_error', self.lateral_error_cb, 10)
        self.create_subscription(Twist, '/autocar/cmd_vel', self.cmd_vel_cb, 10)

        self.lap_pub = self.create_publisher(Float64, '/autocar/lap_time', 10)
        self.current_pub = self.create_publisher(Float64, '/autocar/current_lap_time', 10)
        self.count_pub = self.create_publisher(Int32, '/autocar/lap_count', 10)

        # Sane defaults so the node also works under the legacy launch.
        self.declare_parameter('controller', 'stanley')
        self.declare_parameter('profile', 'default')
        self.declare_parameter('latency_ms', 0)
        self.declare_parameter('odom_noise_std', 0.0)
        self.declare_parameter('offtrack_threshold_m', 4.0)

        self.controller = str(self.get_parameter('controller').value)
        self.profile = str(self.get_parameter('profile').value)
        self.latency_ms = int(self.get_parameter('latency_ms').value)
        self.odom_noise_std = float(self.get_parameter('odom_noise_std').value)
        self.offtrack_threshold = float(self.get_parameter('offtrack_threshold_m').value)

        self.session_id = datetime.now().strftime('%Y-%m-%dT%H-%M-%S')
        self.prev_x = None
        self.prev_y = None
        self.lap_count = 0
        self.lap_start_time = None
        self.best_lap = None

        self._reset_lap_accumulators()

        # Steering-rate state must persist across lap boundaries, otherwise
        # the very first message after a lap reset would look like a huge jump.
        self.prev_steer = None
        self.prev_steer_time = None

        self.timer = self.create_timer(1.0 / LIVE_TIMER_HZ, self._publish_live)

        CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
        self._init_csv()

        self.get_logger().info(
            f'Race recorder armed. session={self.session_id} '
            f'controller={self.controller} profile={self.profile} '
            f'latency_ms={self.latency_ms} odom_noise_std={self.odom_noise_std:.3f}. '
            f'Start line: x={START_X:.2f}, width +/-{ROAD_HALF_WIDTH:.1f}, direction +Y. '
            f'CSV: {CSV_PATH}'
        )

    # ------------------------------------------------------------------
    # CSV: one-shot migration from legacy 7-col schema
    # ------------------------------------------------------------------
    def _init_csv(self):
        if not CSV_PATH.exists():
            with CSV_PATH.open('w', newline='') as f:
                csv.writer(f).writerow(CSV_FIELDS)
            return

        with CSV_PATH.open('r', newline='') as f:
            reader = csv.reader(f)
            try:
                header = next(reader)
            except StopIteration:
                header = []
            rows = list(reader)

        if header == CSV_FIELDS:
            return

        if header == LEGACY_FIELDS:
            self.get_logger().info(
                f'Migrating CSV from legacy 7-col to {len(CSV_FIELDS)}-col schema; '
                f'{len(rows)} historical row(s) preserved.'
            )
            pad = [''] * len(EXTRA_FIELDS)
            with CSV_PATH.open('w', newline='') as f:
                w = csv.writer(f)
                w.writerow(CSV_FIELDS)
                for r in rows:
                    w.writerow(r + pad)
            return

        # Unknown header: do not destroy the file. Append-only with the new
        # schema. Downstream readers will see a ragged file but no data is lost.
        self.get_logger().warning(
            f'CSV header mismatch (got {header!r}); appending with new schema without migrating.'
        )

    # ------------------------------------------------------------------
    # Per-lap accumulators
    # ------------------------------------------------------------------
    def _reset_lap_accumulators(self):
        self.dist_accum = 0.0
        self.max_speed = 0.0
        self.lat_err_sq_sum = 0.0
        self.lat_err_count = 0
        self.lat_err_max = 0.0
        self.steering_rate_max = 0.0
        self.offtrack_events = 0
        self._was_offtrack = False

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
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

    def lateral_error_cb(self, msg: Float64):
        if self.lap_start_time is None:
            return
        e = float(msg.data)
        abs_e = abs(e)
        self.lat_err_sq_sum += e * e
        self.lat_err_count += 1
        if abs_e > self.lat_err_max:
            self.lat_err_max = abs_e

        # Off-track event: rising edge above threshold.
        is_offtrack = abs_e > self.offtrack_threshold
        if is_offtrack and not self._was_offtrack:
            self.offtrack_events += 1
        self._was_offtrack = is_offtrack

    def cmd_vel_cb(self, msg: Twist):
        steer = float(msg.angular.z)
        t = self.get_clock().now()
        if self.prev_steer is not None and self.prev_steer_time is not None and self.lap_start_time is not None:
            dt = (t - self.prev_steer_time).nanoseconds * 1e-9
            if dt > 1e-6:
                rate = abs(steer - self.prev_steer) / dt
                if rate > self.steering_rate_max:
                    self.steering_rate_max = rate
        self.prev_steer = steer
        self.prev_steer_time = t

    # ------------------------------------------------------------------
    # Live HUD
    # ------------------------------------------------------------------
    def _publish_live(self):
        count = Int32()
        count.data = self.lap_count
        self.count_pub.publish(count)

        if self.lap_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.lap_start_time).nanoseconds * 1e-9
        m = Float64()
        m.data = elapsed
        self.current_pub.publish(m)

    # ------------------------------------------------------------------
    # Lap crossing -> commit a CSV row
    # ------------------------------------------------------------------
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
        lat_rms = math.sqrt(self.lat_err_sq_sum / self.lat_err_count) if self.lat_err_count else 0.0

        if self.best_lap is None or elapsed < self.best_lap:
            self.best_lap = elapsed
            best_tag = '  [NEW BEST]'
        else:
            best_tag = f'  (best {self.best_lap:.2f}s)'

        self.get_logger().info(
            f'Lap {self.lap_count}: {elapsed:.2f} s '
            f'(avg {avg_speed:.2f} m/s, max {self.max_speed:.2f} m/s, '
            f'dist {self.dist_accum:.1f} m, lat_rms {lat_rms:.3f} m, '
            f'lat_max {self.lat_err_max:.3f} m, '
            f'steer_rate_max {self.steering_rate_max:.3f} rad/s, '
            f'offtrack {self.offtrack_events}){best_tag}'
        )

        msg = Float64()
        msg.data = elapsed
        self.lap_pub.publish(msg)

        with CSV_PATH.open('a', newline='') as f:
            csv.writer(f).writerow([
                self.session_id,
                self.lap_count,
                datetime.now().isoformat(timespec='seconds'),
                f'{elapsed:.3f}',
                f'{avg_speed:.3f}',
                f'{self.max_speed:.3f}',
                f'{self.dist_accum:.2f}',
                self.controller,
                self.profile,
                self.latency_ms,
                f'{self.odom_noise_std:.4f}',
                f'{lat_rms:.4f}',
                f'{self.lat_err_max:.4f}',
                f'{self.steering_rate_max:.4f}',
                self.offtrack_events,
            ])

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
