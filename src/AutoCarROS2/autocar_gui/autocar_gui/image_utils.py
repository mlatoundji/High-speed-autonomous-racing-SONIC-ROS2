"""Convert sensor_msgs/Image messages to QImage without cv_bridge."""

from __future__ import annotations

from PySide6.QtGui import QImage
from sensor_msgs.msg import Image


def image_msg_to_qimage(msg: Image) -> QImage | None:
    """Build a QImage from a ROS image message."""
    if msg.width == 0 or msg.height == 0 or not msg.data:
        return None

    encoding = msg.encoding.lower()
    if encoding == 'rgb8':
        image_format = QImage.Format.Format_RGB888
        bytes_per_pixel = 3
    elif encoding == 'bgr8':
        image_format = QImage.Format.Format_BGR888
        bytes_per_pixel = 3
    elif encoding in ('mono8', '8uc1'):
        image_format = QImage.Format.Format_Grayscale8
        bytes_per_pixel = 1
    else:
        return None

    bytes_per_line = msg.step if msg.step > 0 else msg.width * bytes_per_pixel
    qimage = QImage(
        bytes(msg.data),
        msg.width,
        msg.height,
        bytes_per_line,
        image_format,
    )
    if qimage.isNull():
        return None
    return qimage.copy()
