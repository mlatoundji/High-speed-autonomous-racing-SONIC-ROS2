#!/usr/bin/env python3

import os

import numpy as np
import pandas as pd
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from autocar_msgs.msg import Path2D, State2D


class GlobalPathPlanner(Node):

    def __init__(self):

        super().__init__('global_planner')

        self.goals_pub = self.create_publisher(Path2D, '/autocar/goals', 10)
        self.goals_viz_pub = self.create_publisher(PoseArray, '/autocar/viz_goals', 10)
        self.localisation_sub = self.create_subscription(
            State2D, '/autocar/state2D', self.vehicle_state_cb, 10)

        try:
            desc = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('waypoints_ahead', None, desc),
                    ('waypoints_behind', None, desc),
                    ('passed_threshold', None, desc),
                    ('waypoints', None, desc),
                    ('centreofgravity_to_frontaxle', None, desc),
                ],
            )

            self.wp_ahead = int(self.get_parameter('waypoints_ahead').value)
            self.wp_behind = int(self.get_parameter('waypoints_behind').value)
            self.passed_threshold = float(self.get_parameter('passed_threshold').value)
            self.cg2frontaxle = float(self.get_parameter('centreofgravity_to_frontaxle').value)

        except ValueError:
            raise Exception('Missing ROS parameters. Check the configuration file.')

        dir_path = os.path.join(
            get_package_share_directory('autocar_nav_pure_pursuit'),
            'data', 'waypoints.csv')
        df = pd.read_csv(dir_path)

        self.ax = df['X-axis'].values.tolist()
        self.ay = df['Y-axis'].values.tolist()

        self.waypoints = min(len(self.ax), len(self.ay))
        self.wp_published = self.wp_ahead + self.wp_behind

        self.x = None
        self.y = None
        self.theta = None

    def vehicle_state_cb(self, msg):
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta
        self.set_waypoints()

    def set_waypoints(self):
        fx = self.x + self.cg2frontaxle * -np.sin(self.theta)
        fy = self.y + self.cg2frontaxle * np.cos(self.theta)

        dx = [fx - icx for icx in self.ax]
        dy = [fy - icy for icy in self.ay]

        d = np.hypot(dx, dy)
        closest_id = np.argmin(d)

        transform = self.frame_transform(
            self.ax[closest_id], self.ay[closest_id], fx, fy, self.theta)

        if closest_id < 2:
            self.get_logger().info(f'Closest Waypoint #{closest_id} (Starting Path)')
            px = self.ax[0:self.wp_published]
            py = self.ay[0:self.wp_published]

        elif closest_id > (self.waypoints - self.wp_published):
            self.get_logger().info(f'Closest Waypoint #{closest_id} (Terminating Path)')
            px = self.ax[-self.wp_published:]
            py = self.ay[-self.wp_published:]

        elif transform[1] < (0.0 - self.passed_threshold):
            self.get_logger().info(f'Closest Waypoint #{closest_id} (Passed)')
            px = self.ax[closest_id - (self.wp_behind - 1):closest_id + (self.wp_ahead + 1)]
            py = self.ay[closest_id - (self.wp_behind - 1):closest_id + (self.wp_ahead + 1)]

        else:
            self.get_logger().info(f'Closest Waypoint #{closest_id} (Approaching)')
            px = self.ax[(closest_id - self.wp_behind):(closest_id + self.wp_ahead)]
            py = self.ay[(closest_id - self.wp_behind):(closest_id + self.wp_ahead)]

        self.publish_goals(px, py)

    def frame_transform(self, point_x, point_y, axle_x, axle_y, theta):
        c = np.cos(-theta)
        s = np.sin(-theta)
        R = np.array(((c, -s), (s, c)))

        p = np.array(((point_x), (point_y)))
        v = np.array(((axle_x), (axle_y)))
        vp = p - v
        transform = R.dot(vp)

        return transform

    def publish_goals(self, px, py):
        waypoints = min(len(px), len(py))
        goals = Path2D()

        viz_goals = PoseArray()
        viz_goals.header.frame_id = 'odom'
        viz_goals.header.stamp = self.get_clock().now().to_msg()

        for i in range(waypoints):
            goal = Pose2D()
            goal.x = px[i]
            goal.y = py[i]
            goals.poses.append(goal)

            vpose = Pose()
            vpose.position.x = px[i]
            vpose.position.y = py[i]
            vpose.position.z = 0.0
            viz_goals.poses.append(vpose)

        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz_goals)


def main(args=None):
    rclpy.init(args=args)
    try:
        global_planner = GlobalPathPlanner()
        rclpy.spin(global_planner)
    finally:
        global_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
