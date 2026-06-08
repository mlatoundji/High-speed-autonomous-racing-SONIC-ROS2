#!/usr/bin/env python3
"""Save the BOF track map when the first exploration lap completes."""

import os
import pickle

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32


class MapSaver(Node):

    def __init__(self):
        super().__init__('map_saver')

        self.declare_parameter('map_save_path', '')
        path = str(self.get_parameter('map_save_path').value).strip()
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(
                get_package_share_directory('autocar_nav_pure_pursuit_lidar'),
                'data', 'track_map.pkl')
        self.save_path = path

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, map_qos)
        self.lap_sub = self.create_subscription(
            Int32, '/autocar/lap_count', self.lap_cb, 10)

        self.latest_map = None
        self.saved = False
        self.lap_count = 0

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        self.get_logger().info(f'Track map will be saved to: {self.save_path}')

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def lap_cb(self, msg: Int32):
        if msg.data > self.lap_count:
            self.lap_count = msg.data
            if self.lap_count >= 1 and not self.saved:
                self._save_map()

    def _save_map(self):
        if self.latest_map is None:
            self.get_logger().warn('Lap 1 complete but no /map received yet.')
            return

        msg = self.latest_map
        payload = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'data': list(msg.data),
        }
        with open(self.save_path, 'wb') as f:
            pickle.dump(payload, f)

        self.saved = True
        self.get_logger().info(
            f'Track map saved ({msg.info.width}x{msg.info.height}) -> {self.save_path}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapSaver()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
