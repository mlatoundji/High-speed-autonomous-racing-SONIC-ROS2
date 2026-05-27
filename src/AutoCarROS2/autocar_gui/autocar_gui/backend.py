"""Backend abstraction for ROS and HTTP control paths."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable, Optional


StatusCallback = Callable[[dict], None]
ImageCallback = Callable[[object], None]
InfoCallback = Callable[[str], None]


class ControlBackend(ABC):
    """Interface shared by ROS and HTTP backends."""

    name = 'unknown'
    camera_available = False

    def __init__(self):
        self.on_status_update: Optional[StatusCallback] = None
        self.on_image_update: Optional[ImageCallback] = None
        self.on_backend_info: Optional[InfoCallback] = None

    @abstractmethod
    def start(self) -> None:
        """Prepare subscriptions or polling."""

    @abstractmethod
    def shutdown(self) -> None:
        """Release backend resources."""

    def tick(self) -> None:
        """Called periodically from the Qt event loop."""

    @abstractmethod
    def set_mode(self, mode: str) -> None:
        """Switch manual, semi or auto mode."""

    @abstractmethod
    def stop_vehicle(self) -> None:
        """Latch vehicle stop."""

    @abstractmethod
    def resume_auto(self) -> None:
        """Clear stop/collision and resume auto mode."""

    @abstractmethod
    def publish_manual(self, linear_x: float, angular_z: float) -> None:
        """Publish a manual drive command."""

    def _emit_status(self, status: dict) -> None:
        if self.on_status_update is not None:
            self.on_status_update(status)

    def _emit_image(self, image) -> None:
        if self.on_image_update is not None:
            self.on_image_update(image)

    def _emit_info(self, message: str) -> None:
        if self.on_backend_info is not None:
            self.on_backend_info(message)


def probe_ros_topics() -> bool:
    """Return True when core AutoCar topics are visible on the ROS graph."""
    try:
        import rclpy

        if not rclpy.ok():
            rclpy.init()

        node = rclpy.create_node('_autocar_gui_probe')
        topic_names = {name for name, _types in node.get_topic_names_and_types()}
        node.destroy_node()
        return (
            '/autocar/control_status' in topic_names
            or '/autocar/third_person_camera/image_raw' in topic_names
        )
    except Exception:
        return False


def create_backend(
    backend: str,
    api_url: str,
) -> ControlBackend:
    """Create a backend according to backend selection mode."""
    from autocar_gui.http_backend import HttpBackend
    from autocar_gui.ros_backend import RosBackend

    if backend == 'ros':
        return RosBackend()
    if backend == 'http':
        return HttpBackend(api_url)

    if probe_ros_topics():
        return RosBackend()
    return HttpBackend(api_url)
