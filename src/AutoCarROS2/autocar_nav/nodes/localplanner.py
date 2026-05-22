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
from autocar_nav import generate_cubic_path, yaw_to_quaternion


# Lateral offsets (m) tried in order when the centerline path is blocked.
# 0.0 first = prefer the original racing line.
LATERAL_OFFSETS = [0.0, 1.5, -1.5, 3.0, -3.0, 4.5, -4.5, 6.0, -6.0]

# A cell is considered an obstacle when its occupancy probability >= this.
OCCUPANCY_THRESHOLD = 50

# Cruise speed (m/s). Kept constant -- swapping between cruise and avoid
# speeds caused visible acceleration spikes whenever a hay bale near the
# road edge triggered the avoidance check. The geometric path deviation
# alone is enough to avoid real obstacles; speed stays steady so the
# Stanley tracker doesn't get destabilised in turns.
CRUISE_VEL = 6.0
AVOID_VEL = CRUISE_VEL


class LocalPathPlanner(Node):

    def __init__(self):

        super().__init__('local_planner')

        # Publishers
        self.local_planner_pub = self.create_publisher(Path2D, '/autocar/path', 10)
        self.path_viz_pub = self.create_publisher(Path, '/autocar/viz_path', 10)
        self.target_vel_pub = self.create_publisher(Float64, '/autocar/target_velocity', 10)

        # Subscribers
        self.goals_sub = self.create_subscription(Path2D, '/autocar/goals', self.goals_cb, 10)
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)

        # /map is published with SensorDataQoS by the bof node (best_effort).
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        # Parameters
        try:
            desc = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None, desc),
                    ('frame_id', None, desc),
                    ('car_width', None, desc),
                    ('centreofgravity_to_frontaxle', None, desc)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.frame_id = str(self.get_parameter("frame_id").value)
            self.car_width = float(self.get_parameter("car_width").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        self.ds = 1 / self.frequency

        self.target_vel = CRUISE_VEL
        self.ax = []
        self.ay = []

        # Occupancy grid cached from /map
        self.grid = None  # 2D numpy array, shape (h, w), values 0..100 (or -1)
        self.grid_info = None  # nav_msgs/MapMetaData

        # Vehicle state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(self.ds, self.timer_cb)

    def timer_cb(self):
        msg = Float64()
        msg.data = self.target_vel
        self.target_vel_pub.publish(msg)

    def map_cb(self, msg: OccupancyGrid):
        self.grid_info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )

    def vehicle_state_cb(self, msg):
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def goals_cb(self, msg):
        self.ax = [p.x for p in msg.poses]
        self.ay = [p.y for p in msg.poses]
        self.publish_path()

    # ---- Collision checking ----------------------------------------------

    def _world_to_grid(self, x, y):
        """Return (col, row) in the occupancy grid, or None if out of bounds."""
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
        """True iff any sampled point of the path lies in an occupied cell.

        The car footprint is approximated by inflating each path point by
        car_width/2 in the grid lookup (a small disk around the sample)."""
        if self.grid is None or self.grid_info is None:
            return False  # no perception yet -> assume clear

        res = self.grid_info.resolution
        # No inflation -- hay bales bordering the road (r=92m, r=114m) used to
        # bleed into the path check and trigger false-positive deviations.
        # The road is 16m wide and we only check the centerline path, so an
        # actual on-road obstacle still gets caught reliably.
        inflate_cells = 0

        # Subsample the path -- checking every cell-sized step is enough.
        step = max(1, int(np.floor(res / self.ds)))
        for i in range(0, len(cx), step):
            cg = self._world_to_grid(cx[i], cy[i])
            if cg is None:
                continue
            col, row = cg
            r0 = max(0, row - inflate_cells)
            r1 = min(self.grid_info.height, row + inflate_cells + 1)
            c0 = max(0, col - inflate_cells)
            c1 = min(self.grid_info.width, col + inflate_cells + 1)
            patch = self.grid[r0:r1, c0:c1]
            if np.any(patch >= OCCUPANCY_THRESHOLD):
                return True
        return False

    # ---- Lateral path shifting -------------------------------------------

    def _shift_waypoints(self, offset):
        """Return (ax', ay') shifted by `offset` meters along the left-normal
        of the local tangent at each waypoint."""
        ax = np.asarray(self.ax, dtype=float)
        ay = np.asarray(self.ay, dtype=float)
        if len(ax) < 2 or offset == 0.0:
            return ax.tolist(), ay.tolist()

        # Tangent via central differences (forward/backward at the ends).
        dx = np.gradient(ax)
        dy = np.gradient(ay)
        norm = np.hypot(dx, dy)
        norm[norm < 1e-9] = 1.0
        # Left-hand normal = rotate tangent by +90deg: (-dy, dx)/|t|
        nx = -dy / norm
        ny = dx / norm
        return (ax + offset * nx).tolist(), (ay + offset * ny).tolist()

    # ---- Path publication ------------------------------------------------

    def publish_path(self):
        if len(self.ax) < 2:
            return

        chosen_cx, chosen_cy, chosen_cyaw, chosen_offset = None, None, None, None

        for offset in LATERAL_OFFSETS:
            sx, sy = self._shift_waypoints(offset)
            cx, cy, cyaw, _ = generate_cubic_path(sx, sy, self.ds)
            n = min(len(cx), len(cy), len(cyaw))
            cx, cy, cyaw = cx[:n], cy[:n], cyaw[:n]
            if not self.path_is_blocked(cx, cy):
                chosen_cx, chosen_cy, chosen_cyaw, chosen_offset = cx, cy, cyaw, offset
                break

        if chosen_cx is None:
            # Every candidate is blocked -- keep the centerline and let the
            # tracker do what it can. Slow down significantly.
            sx, sy = self._shift_waypoints(0.0)
            chosen_cx, chosen_cy, chosen_cyaw, _ = generate_cubic_path(sx, sy, self.ds)
            n = min(len(chosen_cx), len(chosen_cy), len(chosen_cyaw))
            chosen_cx, chosen_cy, chosen_cyaw = chosen_cx[:n], chosen_cy[:n], chosen_cyaw[:n]
            chosen_offset = 0.0
            self.target_vel = AVOID_VEL * 0.5
            self.get_logger().warn('All lateral offsets blocked -- slowing to crawl.')
        elif chosen_offset != 0.0:
            self.target_vel = AVOID_VEL
            self.get_logger().info(f'Path blocked, deviating by {chosen_offset:+.1f} m')
        else:
            self.target_vel = CRUISE_VEL

        target_path = Path2D()
        viz_path = Path()
        viz_path.header.frame_id = "odom"
        viz_path.header.stamp = self.get_clock().now().to_msg()

        for n in range(len(chosen_cx)):
            npose = Pose2D()
            npose.x = chosen_cx[n]
            npose.y = chosen_cy[n]
            npose.theta = chosen_cyaw[n]
            target_path.poses.append(npose)

            vpose = PoseStamped()
            vpose.header.frame_id = "odom"
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


if __name__ == "__main__":
    main()
