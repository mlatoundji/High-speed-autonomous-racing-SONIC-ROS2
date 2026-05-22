#!/usr/bin/env python3

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64

from autocar_msgs.msg import Path2D, State2D
from autocar_nav_pure_pursuit.pure_pursuit import (
    closest_path_index,
    dynamic_lookahead,
    front_axle_pose,
    limit_steering_rate,
    lookahead_point,
    pure_pursuit_steering,
)
from autocar_nav_pure_pursuit.yaw_to_quaternion import yaw_to_quaternion


class PurePursuitTracker(Node):

    def __init__(self):

        super().__init__('path_tracker')

        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)

        self.localisation_sub = self.create_subscription(
            State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(
            Path2D, '/autocar/path', self.path_cb, 10)
        self.target_vel_sub = self.create_subscription(
            Float64, '/autocar/target_velocity', self.target_vel_cb, 10)

        try:
            desc = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None, desc),
                    ('centreofgravity_to_frontaxle', None, desc),
                    ('wheelbase', None, desc),
                    ('lookahead_gain', None, desc),
                    ('lookahead_min', None, desc),
                    ('lookahead_max', None, desc),
                    ('closest_search_ahead', None, desc),
                    ('steering_limits', None, desc),
                    ('steering_rate_limit', None, desc),
                    ('velocity_gain', None, desc),
                ],
            )

            self.frequency = float(self.get_parameter('update_frequency').value)
            self.cg2frontaxle = float(self.get_parameter('centreofgravity_to_frontaxle').value)
            self.wheelbase = float(self.get_parameter('wheelbase').value)
            self.ld_gain = float(self.get_parameter('lookahead_gain').value)
            self.ld_min = float(self.get_parameter('lookahead_min').value)
            self.ld_max = float(self.get_parameter('lookahead_max').value)
            self.search_ahead = int(self.get_parameter('closest_search_ahead').value)
            self.max_steer = float(self.get_parameter('steering_limits').value)
            self.steer_rate_limit = float(self.get_parameter('steering_rate_limit').value)
            self.velocity_gain = float(self.get_parameter('velocity_gain').value)

        except ValueError:
            raise Exception('Missing ROS parameters. Check the configuration file.')

        self.x = None
        self.y = None
        self.yaw = None
        self.vel = 0.0
        self.target_vel = 0.0

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.closest_idx = 0
        self.prev_steer = None

        self.lock = threading.Lock()
        self.dt = 1.0 / self.frequency

        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        self.pure_pursuit_control()

    def vehicle_state_cb(self, msg):
        self.lock.acquire()
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = np.sqrt(msg.twist.x ** 2.0 + msg.twist.y ** 2.0)
        self.lock.release()

    def path_cb(self, msg):
        self.lock.acquire()
        self.cx = [p.x for p in msg.poses]
        self.cy = [p.y for p in msg.poses]
        self.cyaw = [p.theta for p in msg.poses]
        self.closest_idx = 0
        self.lock.release()

    def target_vel_cb(self, msg):
        self.target_vel = msg.data

    def pure_pursuit_control(self):
        self.lock.acquire()

        if self.x is None or not self.cx:
            self.lock.release()
            return

        fx, fy = front_axle_pose(self.x, self.y, self.yaw, self.cg2frontaxle)

        self.closest_idx = closest_path_index(
            fx, fy, self.cx, self.cy,
            start_idx=self.closest_idx,
            search_ahead=self.search_ahead,
        )

        ld = dynamic_lookahead(self.vel, self.ld_gain, self.ld_min, self.ld_max)

        tx, ty, la_idx = lookahead_point(
            fx, fy, self.cx, self.cy, self.closest_idx, ld)

        steer = pure_pursuit_steering(
            fx, fy, self.yaw, tx, ty, ld, self.wheelbase)

        steer = limit_steering_rate(
            steer, self.prev_steer, self.dt, self.steer_rate_limit)
        self.prev_steer = steer

        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        ref_yaw = self.cyaw[la_idx] if la_idx < len(self.cyaw) else self.yaw
        self._publish_lateral_ref(tx, ty, ref_yaw)

        cmd_vel = self.target_vel * self.velocity_gain
        self._publish_command(cmd_vel, steer)

        self.lock.release()

    def _publish_lateral_ref(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        self.lateral_ref_pub.publish(pose)

    def _publish_command(self, velocity, steering_angle):
        drive = Twist()
        drive.linear.x = velocity
        drive.angular.z = steering_angle
        self.tracker_pub.publish(drive)


def main(args=None):
    rclpy.init(args=args)
    try:
        tracker = PurePursuitTracker()
        rclpy.spin(tracker)
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
