#!/usr/bin/env python3
"""Diagnostic logger to pinpoint WHAT limits the car in tight corners.

Records a time-series CSV of speed / target speed / steering command /
steering rate / lateral error / position, so we can tell apart:
  - planner over-slowing      -> target_vel collapses before the corner
  - steering rate-limited      -> steer_rate pinned at +/- steering_rate_limit
  - steering saturation        -> steer_cmd pinned at +/- steering_limits
  - tracking blow-out          -> lateral_error spikes

Run it (ROS sourced) WHILE a race/benchmark is running, in a 2nd terminal:
    python3 scripts/diag_metrics.py
    python3 scripts/diag_metrics.py --duration 180 --out /tmp/diag.csv

Stop with Ctrl+C (or it auto-stops after --duration). Then analyse the CSV.
"""

import argparse
import csv
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


class DiagLogger(Node):
    def __init__(self, out_path, rate_hz, duration_s):
        super().__init__('diag_metrics')
        self.x = self.y = 0.0
        self.speed = 0.0
        self.steer = 0.0
        self.throttle = 0.0
        self.target_vel = 0.0
        self.lat_err = 0.0
        self.prev_steer = 0.0
        self.t0 = None
        self.duration_s = duration_s
        self.dt = 1.0 / rate_hz
        self.n = 0

        self.create_subscription(Odometry, '/autocar/odom', self._odom_cb, 10)
        self.create_subscription(Twist, '/autocar/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Float64, '/autocar/target_velocity', self._tv_cb, 10)
        self.create_subscription(Float64, '/autocar/lateral_error', self._le_cb, 10)

        self.f = open(out_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow([
            't_s', 'x', 'y', 'speed_mps', 'target_vel_mps',
            'steer_cmd_rad', 'steer_rate_radps', 'throttle_cmd', 'lateral_error_m',
        ])
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'diag_metrics: logging to {out_path} at {rate_hz:.0f} Hz')

    def _odom_cb(self, m):
        s = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        if math.isfinite(s):
            self.speed = s
        if math.isfinite(m.pose.pose.position.x):
            self.x = m.pose.pose.position.x
            self.y = m.pose.pose.position.y

    def _cmd_cb(self, m):
        if math.isfinite(m.angular.z):
            self.steer = m.angular.z
        if math.isfinite(m.linear.x):
            self.throttle = m.linear.x

    def _tv_cb(self, m):
        if math.isfinite(m.data):
            self.target_vel = m.data

    def _le_cb(self, m):
        if math.isfinite(m.data):
            self.lat_err = m.data

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        steer_rate = (self.steer - self.prev_steer) / self.dt
        self.prev_steer = self.steer

        self.w.writerow([
            f'{t:.3f}', f'{self.x:.2f}', f'{self.y:.2f}',
            f'{self.speed:.3f}', f'{self.target_vel:.3f}',
            f'{self.steer:.4f}', f'{steer_rate:.3f}',
            f'{self.throttle:.4f}', f'{self.lat_err:.4f}',
        ])
        self.n += 1
        if self.n % 25 == 0:
            self.f.flush()
        if self.n % 50 == 0:  # ~1 s live readout
            self.get_logger().info(
                f't={t:6.1f}s v={self.speed:4.1f} tgt={self.target_vel:4.1f} '
                f'steer={self.steer:+.2f} rate={steer_rate:+5.1f} '
                f'lat_err={self.lat_err:+.2f}')
        if self.duration_s and t >= self.duration_s:
            self.get_logger().info('diag_metrics: duration reached, stopping.')
            raise SystemExit

    def destroy_node(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='/tmp/diag_metrics.csv')
    p.add_argument('--rate', type=float, default=50.0, help='sampling Hz')
    p.add_argument('--duration', type=float, default=0.0,
                   help='auto-stop after N seconds (0 = until Ctrl+C)')
    args = p.parse_args()

    rclpy.init()
    node = DiagLogger(args.out, args.rate, args.duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print(f'\n[diag_metrics] CSV written: {args.out}')


if __name__ == '__main__':
    main()
