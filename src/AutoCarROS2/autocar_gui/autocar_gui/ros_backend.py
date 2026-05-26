"""ROS 2 backend for the AutoCar control panel."""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String

from autocar_gui.backend import ControlBackend
from autocar_gui.image_utils import image_msg_to_qimage
from autocar_msgs.msg import State2D


class RosBackend(ControlBackend, Node):
    name = 'ROS'
    camera_available = True

    def __init__(self):
        ControlBackend.__init__(self)
        if not rclpy.ok():
            rclpy.init()
        Node.__init__(self, 'autocar_control_panel')

        self._status = {}

        self._mode_pub = self.create_publisher(String, '/autocar/control_mode', 10)
        self._stop_pub = self.create_publisher(Bool, '/autocar/stop', 10)
        self._resume_pub = self.create_publisher(Bool, '/autocar/resume_auto', 10)
        self._manual_pub = self.create_publisher(Twist, '/autocar/manual_cmd_vel', 10)

        self.create_subscription(
            Image, '/autocar/third_person_camera/image_raw', self._image_cb, 10)
        self.create_subscription(
            String, '/autocar/control_status', self._status_cb, 10)
        self.create_subscription(
            State2D, '/autocar/state2D', self._state_cb, 10)
        self.create_subscription(
            Float64, '/autocar/lateral_error', self._lateral_error_cb, 10)

        self._emit_info('Backend ROS actif (topics locaux).')

    def start(self) -> None:
        return

    def shutdown(self) -> None:
        self.destroy_node()

    def tick(self) -> None:
        rclpy.spin_once(self, timeout_sec=0)

    def set_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def stop_vehicle(self) -> None:
        msg = Bool()
        msg.data = True
        self._stop_pub.publish(msg)

    def resume_auto(self) -> None:
        msg = Bool()
        msg.data = True
        self._resume_pub.publish(msg)

    def publish_manual(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._manual_pub.publish(msg)

    def _status_cb(self, msg: String) -> None:
        try:
            self._status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._status = {'mode': 'unknown', 'raw': msg.data}
        self._emit_status(dict(self._status))

    def _state_cb(self, msg: State2D) -> None:
        if 'speed' not in self._status:
            self._status['speed'] = float(msg.twist.x)
            self._emit_status(dict(self._status))

    def _lateral_error_cb(self, msg: Float64) -> None:
        self._status['lateral_error'] = float(msg.data)
        if self.on_status_update is not None and self._status:
            self._emit_status(dict(self._status))

    def _image_cb(self, msg: Image) -> None:
        qimage = image_msg_to_qimage(msg)
        if qimage is None:
            self.get_logger().warning(
                f'Unsupported image encoding: {msg.encoding}')
            return
        self._emit_image(qimage)
