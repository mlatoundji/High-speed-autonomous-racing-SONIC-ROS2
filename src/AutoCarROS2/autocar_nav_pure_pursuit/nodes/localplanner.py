#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64

from autocar_msgs.msg import Path2D, State2D
from autocar_nav_pure_pursuit import generate_cubic_path, yaw_to_quaternion
from autocar_nav_pure_pursuit.pure_pursuit import (
    apply_speed_ramp,
    closest_path_index,
    curvature_speed_limit,
    front_axle_pose,
    peak_curvature,
)

LATERAL_OFFSETS = [0.0, 1.5, -1.5, 3.0, -3.0, 4.5, -4.5, 6.0, -6.0]
OCCUPANCY_THRESHOLD = 50


class LocalPathPlanner(Node):

    def __init__(self):

        super().__init__('local_planner')

        self.local_planner_pub = self.create_publisher(Path2D, '/autocar/path', 10)
        self.path_viz_pub = self.create_publisher(Path, '/autocar/viz_path', 10)
        self.target_vel_pub = self.create_publisher(Float64, '/autocar/target_velocity', 10)

        self.goals_sub = self.create_subscription(
            Path2D, '/autocar/goals', self.goals_cb, 10)
        self.localisation_sub = self.create_subscription(
            State2D, '/autocar/state2D', self.vehicle_state_cb, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        try:
            desc = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None, desc),
                    ('frame_id', None, desc),
                    ('car_width', None, desc),
                    ('centreofgravity_to_frontaxle', None, desc),
                    ('cruise_velocity', None, desc),
                    ('avoid_velocity', None, desc),
                    ('max_lateral_accel', None, desc),
                    ('min_curvature', None, desc),
                    ('curvature_lookahead', None, desc),
                    ('curvature_smooth_window', None, desc),
                    ('accel_rate', None, desc),
                    ('decel_rate', None, desc),
                ],
            )

            self.frequency = float(self.get_parameter('update_frequency').value)
            self.frame_id = str(self.get_parameter('frame_id').value)
            self.car_width = float(self.get_parameter('car_width').value)
            self.cg2frontaxle = float(self.get_parameter('centreofgravity_to_frontaxle').value)
            self.cruise_vel = float(self.get_parameter('cruise_velocity').value)
            self.avoid_vel = float(self.get_parameter('avoid_velocity').value)
            self.max_lat_accel = float(self.get_parameter('max_lateral_accel').value)
            self.min_kappa = float(self.get_parameter('min_curvature').value)
            self.curv_lookahead = int(self.get_parameter('curvature_lookahead').value)
            self.curv_smooth = int(self.get_parameter('curvature_smooth_window').value)
            self.accel_rate = float(self.get_parameter('accel_rate').value)
            self.decel_rate = float(self.get_parameter('decel_rate').value)

        except ValueError:
            raise Exception('Missing ROS parameters. Check the configuration file.')

        self.ds = 1.0 / self.frequency

        self.target_vel = self.cruise_vel
        self.ramped_vel = self.cruise_vel
        self.ax = []
        self.ay = []

        self.path_cx = []
        self.path_cy = []
        self.path_ck = []
        self.closest_idx = 0

        self.grid = None
        self.grid_info = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(self.ds, self.timer_cb)

    def timer_cb(self):
        self._update_target_velocity()
        msg = Float64()
        msg.data = self.ramped_vel
        self.target_vel_pub.publish(msg)

    def _update_target_velocity(self):
        if not self.path_cx:
            return

        fx, fy = front_axle_pose(self.x, self.y, self.yaw, self.cg2frontaxle)
        self.closest_idx = closest_path_index(
            fx, fy, self.path_cx, self.path_cy,
            start_idx=self.closest_idx,
            search_ahead=60,
        )

        end_idx = min(len(self.path_ck), self.closest_idx + self.curv_lookahead)
        k_peak = peak_curvature(
            self.path_ck, self.closest_idx, end_idx, self.curv_smooth)

        v_curve = curvature_speed_limit(
            k_peak, self.max_lat_accel, self.min_kappa)
        v_target = min(self.target_vel, v_curve)

        self.ramped_vel = apply_speed_ramp(
            self.ramped_vel, v_target, self.ds,
            self.accel_rate, self.decel_rate)

    def map_cb(self, msg: OccupancyGrid):
        self.grid_info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def vehicle_state_cb(self, msg):
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def goals_cb(self, msg):
        self.ax = [p.x for p in msg.poses]
        self.ay = [p.y for p in msg.poses]
        self.publish_path()

    def _world_to_grid(self, x, y):
        info = self.grid_info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        if 0 <= col < info.width and 0 <= row < info.height:
            return col, row
        return None

    def path_is_blocked(self, cx, cy):
        if self.grid is None or self.grid_info is None:
            return False

        res = self.grid_info.resolution
        step = max(1, int(np.floor(res / self.ds)))
        for i in range(0, len(cx), step):
            cg = self._world_to_grid(cx[i], cy[i])
            if cg is None:
                continue
            col, row = cg
            if self.grid[row, col] >= OCCUPANCY_THRESHOLD:
                return True
        return False

    def _shift_waypoints(self, offset):
        ax = np.asarray(self.ax, dtype=float)
        ay = np.asarray(self.ay, dtype=float)
        if len(ax) < 2 or offset == 0.0:
            return ax.tolist(), ay.tolist()

        dx = np.gradient(ax)
        dy = np.gradient(ay)
        norm = np.hypot(dx, dy)
        norm[norm < 1e-9] = 1.0
        nx = -dy / norm
        ny = dx / norm
        return (ax + offset * nx).tolist(), (ay + offset * ny).tolist()

    def publish_path(self):
        if len(self.ax) < 2:
            return

        chosen_cx, chosen_cy, chosen_cyaw, chosen_ck = None, None, None, None
        chosen_offset = None

        for offset in LATERAL_OFFSETS:
            sx, sy = self._shift_waypoints(offset)
            cx, cy, cyaw, ck = generate_cubic_path(sx, sy, self.ds)
            n = min(len(cx), len(cy), len(cyaw), len(ck))
            cx, cy, cyaw, ck = cx[:n], cy[:n], cyaw[:n], ck[:n]
            if not self.path_is_blocked(cx, cy):
                chosen_cx, chosen_cy, chosen_cyaw, chosen_ck = cx, cy, cyaw, ck
                chosen_offset = offset
                break

        if chosen_cx is None:
            sx, sy = self._shift_waypoints(0.0)
            chosen_cx, chosen_cy, chosen_cyaw, chosen_ck = generate_cubic_path(sx, sy, self.ds)
            n = min(len(chosen_cx), len(chosen_cy), len(chosen_cyaw), len(chosen_ck))
            chosen_cx = chosen_cx[:n]
            chosen_cy = chosen_cy[:n]
            chosen_cyaw = chosen_cyaw[:n]
            chosen_ck = chosen_ck[:n]
            chosen_offset = 0.0
            self.target_vel = self.avoid_vel * 0.5
            self.get_logger().warn('All lateral offsets blocked -- slowing to crawl.')
        elif chosen_offset != 0.0:
            self.target_vel = self.avoid_vel
            self.get_logger().info(f'Path blocked, deviating by {chosen_offset:+.1f} m')
        else:
            self.target_vel = self.cruise_vel

        self.path_cx = chosen_cx
        self.path_cy = chosen_cy
        self.path_ck = chosen_ck
        self.closest_idx = 0

        target_path = Path2D()
        viz_path = Path()
        viz_path.header.frame_id = 'odom'
        viz_path.header.stamp = self.get_clock().now().to_msg()

        for n in range(len(chosen_cx)):
            npose = Pose2D()
            npose.x = chosen_cx[n]
            npose.y = chosen_cy[n]
            npose.theta = chosen_cyaw[n]
            target_path.poses.append(npose)

            vpose = PoseStamped()
            vpose.header.frame_id = 'odom'
            vpose.header.stamp = self.get_clock().now().to_msg()
            vpose.pose.position.x = chosen_cx[n]
            vpose.pose.position.y = chosen_cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = yaw_to_quaternion(np.pi * 0.5 - chosen_cyaw[n])
            viz_path.poses.append(vpose)

        self.local_planner_pub.publish(target_path)
        self.path_viz_pub.publish(viz_path)


def main(args=None):
    rclpy.init(args=args)
    try:
        local_planner = LocalPathPlanner()
        rclpy.spin(local_planner)
    finally:
        local_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
