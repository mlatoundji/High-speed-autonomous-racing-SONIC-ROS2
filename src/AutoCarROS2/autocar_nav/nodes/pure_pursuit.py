#!/usr/bin/env python3
"""Pure Pursuit controller for the SONIC-ROS2 racing project.

Same topic contract as `tracker.py` (Stanley) so the two are interchangeable
through the `controller:=` launch argument and the `lap_timer` metrics
recorder sees no difference.

Algorithm
---------
1. Compute the dynamic lookahead distance Ld = k_v * v + Ld_min,
   where v is the current longitudinal speed.
2. From the closest path point to the rear axle, walk forward along the
   path accumulating arc length until cumulative distance >= Ld. That is
   the lookahead point.
3. Express the lookahead point in the vehicle frame. Let alpha be the
   angle from the vehicle heading to that point.
4. Output steering: delta = atan2(2 * L * sin(alpha), Ld), saturated to
   +/- max_steer.

Frame convention (inherited from the codebase, NOT standard ROS REP-103)
-----------------------------------------------------------------------
The State2D yaw in this project is measured such that the vehicle forward
direction is (-sin(yaw), +cos(yaw)). This is consistent with how
`tracker.py` offsets the front axle: `fx = x - cg2front * sin(yaw)`,
`fy = y + cg2front * cos(yaw)`. The right-hand direction
(+cos(yaw), +sin(yaw)) is what `tracker.py` calls `front_axle_vec`. We
mirror this convention so cross-track signs match between the two
controllers and the metrics CSV stays comparable.

Topics in:
    /autocar/state2D         autocar_msgs/State2D
    /autocar/path            autocar_msgs/Path2D
    /autocar/target_velocity std_msgs/Float64

Topics out:
    /autocar/cmd_vel         geometry_msgs/Twist
                             (linear.x = velocity, angular.z = steering angle)
    /autocar/lateral_error   std_msgs/Float64  (signed, same definition as tracker.py)
    /autocar/lateral_ref     geometry_msgs/PoseStamped  (debug viz of the lookahead point)
"""

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64

from autocar_msgs.msg import Path2D, State2D
from autocar_nav import yaw_to_quaternion


class PurePursuit(Node):

    def __init__(self):
        super().__init__('path_tracker')  # keep the existing node name -> same YAML section

        self.cmd_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)
        self.lateral_error_pub = self.create_publisher(Float64, '/autocar/lateral_error', 10)

        self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.create_subscription(Path2D, '/autocar/path', self.path_cb, 10)
        self.create_subscription(Float64, '/autocar/target_velocity', self.target_vel_cb, 10)

        # Declare parameters with sane defaults so launching without the
        # navigation_params.yaml still works for quick tests.
        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_frequency', 50.0, desc),
                ('lookahead_min_m', 1.5, desc),
                ('lookahead_gain', 0.4, desc),
                ('wheelbase_m', 2.966, desc),
                ('steering_limits', 0.95, desc),
                ('centreofgravity_to_frontaxle', 1.483, desc),
                ('centreofgravity_to_rearaxle', 1.483, desc),
            ],
        )

        self.frequency = float(self.get_parameter('update_frequency').value)
        self.ld_min = float(self.get_parameter('lookahead_min_m').value)
        self.ld_gain = float(self.get_parameter('lookahead_gain').value)
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.max_steer = float(self.get_parameter('steering_limits').value)
        self.cg2front = float(self.get_parameter('centreofgravity_to_frontaxle').value)
        self.cg2rear = float(self.get_parameter('centreofgravity_to_rearaxle').value)

        # Mutable state.
        self.x = None
        self.y = None
        self.yaw = None
        self.v = 0.0
        self.target_vel = 0.0
        self.path_x = np.array([])
        self.path_y = np.array([])
        self.path_yaw = np.array([])

        self.lock = threading.Lock()

        self.dt = 1.0 / self.frequency
        self.timer = self.create_timer(self.dt, self.control_step)

        self.get_logger().info(
            f'Pure Pursuit armed. Ld = {self.ld_gain:.3f}*v + {self.ld_min:.2f} m, '
            f'wheelbase = {self.wheelbase:.3f} m, max_steer = {self.max_steer:.3f} rad.'
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def vehicle_state_cb(self, msg: State2D):
        with self.lock:
            self.x = msg.pose.x
            self.y = msg.pose.y
            self.yaw = msg.pose.theta
            self.v = float(np.hypot(msg.twist.x, msg.twist.y))

    def path_cb(self, msg: Path2D):
        with self.lock:
            self.path_x = np.array([p.x for p in msg.poses])
            self.path_y = np.array([p.y for p in msg.poses])
            self.path_yaw = np.array([p.theta for p in msg.poses])

    def target_vel_cb(self, msg: Float64):
        self.target_vel = float(msg.data)

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    def _forward(self, yaw):
        # Codebase convention (see module docstring).
        return -np.sin(yaw), np.cos(yaw)

    def _right(self, yaw):
        return np.cos(yaw), np.sin(yaw)

    def _rear_axle(self):
        fwd_x, fwd_y = self._forward(self.yaw)
        return self.x - self.cg2rear * fwd_x, self.y - self.cg2rear * fwd_y

    def _front_axle(self):
        fwd_x, fwd_y = self._forward(self.yaw)
        return self.x + self.cg2front * fwd_x, self.y + self.cg2front * fwd_y

    def _lateral_error_front_axle(self, fx, fy):
        # Same definition as tracker.py: signed projection of
        # (front_axle - nearest_path_point) on the vehicle right vector.
        if self.path_x.size == 0:
            return 0.0
        dx = fx - self.path_x
        dy = fy - self.path_y
        d = np.hypot(dx, dy)
        i = int(np.argmin(d))
        rx, ry = self._right(self.yaw)
        return float(dx[i] * rx + dy[i] * ry)

    def _find_lookahead_point(self, rx, ry, ld):
        """Return (idx, lookahead_xy) or (None, None) if path is too short."""
        if self.path_x.size == 0:
            return None, None

        # 1) closest path index to the rear axle
        d = np.hypot(self.path_x - rx, self.path_y - ry)
        i0 = int(np.argmin(d))

        # 2) walk forward, accumulating arc length, until total >= ld
        n = self.path_x.size
        acc = float(d[i0])  # start by counting the gap from rear axle to i0
        i = i0
        while i + 1 < n:
            seg = float(np.hypot(self.path_x[i + 1] - self.path_x[i],
                                 self.path_y[i + 1] - self.path_y[i]))
            acc += seg
            i += 1
            if acc >= ld:
                return i, (float(self.path_x[i]), float(self.path_y[i]))
        # Path was too short. Return the last point so we still steer somewhere.
        return n - 1, (float(self.path_x[-1]), float(self.path_y[-1]))

    # ------------------------------------------------------------------
    # Control step
    # ------------------------------------------------------------------
    def control_step(self):
        with self.lock:
            if self.x is None or self.path_x.size == 0:
                return

            # Dynamic lookahead. v is the measured speed (not target):
            # the natural damping comes from "I am going this fast NOW".
            ld = self.ld_gain * self.v + self.ld_min

            rx, ry = self._rear_axle()
            idx, lookahead = self._find_lookahead_point(rx, ry, ld)
            if lookahead is None:
                return
            lx, ly = lookahead

            # Express the lookahead point in the vehicle frame.
            dx, dy = lx - rx, ly - ry
            fwd_x, fwd_y = self._forward(self.yaw)
            rgt_x, rgt_y = self._right(self.yaw)
            longitudinal = dx * fwd_x + dy * fwd_y
            lateral = dx * rgt_x + dy * rgt_y
            alpha = float(np.arctan2(lateral, longitudinal))

            # True chord length from rear axle to the lookahead point.
            chord = float(np.hypot(dx, dy))
            chord = max(chord, 1e-3)  # avoid div-by-zero at standstill

            # Classic bicycle-model Pure Pursuit steering law.
            # Sign convention of the simulator: empirically, Stanley emits
            # positive sigma when it needs the car to turn one way and the
            # Ackermann plugin obeys. Our raw PP delta is sign-inverted
            # relative to that. We negate to align with the codebase
            # convention; without this, the car drives straight off the
            # outside of the first turn (verified on 2026-05-23).
            delta = -float(np.arctan2(2.0 * self.wheelbase * np.sin(alpha), chord))

            # Saturate to physical limits.
            if delta > self.max_steer:
                delta = self.max_steer
            elif delta < -self.max_steer:
                delta = -self.max_steer

            # Publish the lookahead point as a debug pose (RViz).
            ref = PoseStamped()
            ref.header.frame_id = 'odom'
            ref.header.stamp = self.get_clock().now().to_msg()
            ref.pose.position.x = lx
            ref.pose.position.y = ly
            ref.pose.orientation = yaw_to_quaternion(float(self.path_yaw[idx]))
            self.lateral_ref_pub.publish(ref)

            # Lateral error (for the metrics CSV) is computed at the front
            # axle to match `tracker.py`'s definition exactly.
            fx, fy = self._front_axle()
            lat_err = self._lateral_error_front_axle(fx, fy)
            err_msg = Float64()
            err_msg.data = lat_err
            self.lateral_error_pub.publish(err_msg)

            # Drive command.
            cmd = Twist()
            cmd.linear.x = float(self.target_vel)
            cmd.angular.z = delta
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PurePursuit()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
