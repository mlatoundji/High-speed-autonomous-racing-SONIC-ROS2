"""Pose helpers for SLAM map/odom TF composition."""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped

from autocar_nav_pure_pursuit.normalise_angle import normalise_angle


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _transform_to_2d(t: TransformStamped) -> tuple[float, float, float]:
    tr = t.transform.translation
    rot = t.transform.rotation
    return tr.x, tr.y, _yaw_from_quat(rot.x, rot.y, rot.z, rot.w)


def slam_pose_in_odom(
        map_to_odom: TransformStamped,
        map_to_base: TransformStamped) -> tuple[float, float, float]:
    """Return base_link pose in the fixed odom/world frame via SLAM TF."""
    mx, my, myaw = _transform_to_2d(map_to_base)
    ox, oy, oyaw = _transform_to_2d(map_to_odom)

    dx = mx - ox
    dy = my - oy
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    x = c * dx + s * dy
    y = -s * dx + c * dy
    yaw = normalise_angle(myaw - oyaw)
    return x, y, yaw


def slam_pose_in_map(map_to_base: TransformStamped) -> tuple[float, float, float]:
    """Return base_link pose expressed in the SLAM map frame."""
    x, y, yaw = _transform_to_2d(map_to_base)
    return x, y, normalise_angle(yaw)
