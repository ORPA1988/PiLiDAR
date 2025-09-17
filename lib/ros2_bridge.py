"""Optional ROS2 bridge used to publish LiDAR data."""

from __future__ import annotations

import json
import importlib.util
from dataclasses import dataclass
from threading import Lock
from typing import Any, Dict, Optional

import numpy as np

_rclpy_spec = importlib.util.find_spec("rclpy")
_sensor_spec = importlib.util.find_spec("sensor_msgs.msg")
_std_spec = importlib.util.find_spec("std_msgs.msg")

if _rclpy_spec and _sensor_spec and _std_spec:  # pragma: no cover - depends on ROS2
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header, String
else:  # pragma: no cover - ROS2 not available
    rclpy = None  # type: ignore
    Node = None  # type: ignore
    PointCloud2 = None  # type: ignore
    PointField = None  # type: ignore
    Header = None  # type: ignore
    String = None  # type: ignore


def _serialise_value(value: Any) -> Any:
    """Convert numpy values into JSON serialisable objects."""

    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, float)):
        return float(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def _concatenate_attributes(points: np.ndarray, attributes: Dict[str, np.ndarray]) -> tuple[np.ndarray, list]:
    """Combine point coordinates with additional attributes for ROS2 publishing."""

    if points.size == 0:
        return points.astype(np.float32), []

    points = np.asarray(points, dtype=np.float32)
    data_columns = [points]
    fields = [
        ("x", 0),
        ("y", 4),
        ("z", 8),
    ]
    offset = 12

    for name, values in attributes.items():
        array = np.asarray(values, dtype=np.float32)
        if array.ndim == 1:
            array = array[:, np.newaxis]
        if array.shape[0] != points.shape[0]:
            continue
        data_columns.append(array)
        for column in range(array.shape[1]):
            field_name = name if array.shape[1] == 1 else f"{name}_{column}"
            fields.append((field_name, offset))
            offset += 4

    data = np.hstack(data_columns)
    return data.astype(np.float32), fields


@dataclass
class _Publishers:
    raw: Any
    cloud: Any


class ROS2LidarBridge:
    """Singleton managing the optional ROS2 publishers."""

    _instance: Optional["ROS2LidarBridge"] = None
    _lock: Lock = Lock()

    def __init__(self, enabled: bool, frame_id: str = "pilidar") -> None:
        self.enabled = bool(enabled and rclpy and PointCloud2 and String)
        self.frame_id = frame_id
        self.node: Optional[Node] = None
        self.publishers: Optional[_Publishers] = None
        if not self.enabled:
            return

        rclpy.init(args=None)  # pragma: no cover - requires ROS2 runtime
        self.node = rclpy.create_node("piliDAR_lidar")
        raw_pub = self.node.create_publisher(String, "pilidar/raw_package", 10)
        cloud_pub = self.node.create_publisher(PointCloud2, "pilidar/pointcloud", 10)
        self.publishers = _Publishers(raw=raw_pub, cloud=cloud_pub)

    @classmethod
    def get_instance(cls, enabled: bool, frame_id: str = "pilidar") -> "ROS2LidarBridge":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls(enabled=enabled, frame_id=frame_id)
        return cls._instance

    @classmethod
    def shutdown_global(cls) -> None:
        with cls._lock:
            if cls._instance is not None:
                cls._instance.shutdown()
            cls._instance = None

    def shutdown(self) -> None:
        if not self.enabled:
            return
        if self.node is not None:
            self.node.destroy_node()
        if rclpy is not None and rclpy.ok():  # pragma: no cover - ROS2 runtime
            rclpy.shutdown()
        self.node = None
        self.publishers = None

    def publish_raw_package(self, package: Dict[str, Any]) -> None:
        if not self.enabled or self.publishers is None:
            return
        message = String()
        message.data = json.dumps(package, default=_serialise_value)
        self.publishers.raw.publish(message)

    def publish_pointcloud(self, pointcloud: Any) -> None:
        if not self.enabled or self.publishers is None or PointCloud2 is None:
            return
        points = getattr(pointcloud, "points", None)
        attributes = getattr(pointcloud, "attributes", {})
        if points is None:
            return

        attributes = dict(attributes)
        data, fields = _concatenate_attributes(points, attributes)
        if data.size == 0:
            return

        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        if self.node is not None:  # pragma: no cover - ROS2 runtime
            msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.height = 1
        msg.width = data.shape[0]
        msg.is_bigendian = False
        msg.is_dense = False
        msg.point_step = data.shape[1] * 4
        msg.row_step = msg.point_step * msg.width
        msg.data = data.tobytes()

        msg.fields = []
        for name, offset in fields:
            field = PointField()
            field.name = name
            field.offset = offset
            field.count = 1
            field.datatype = PointField.FLOAT32
            msg.fields.append(field)

        self.publishers.cloud.publish(msg)


__all__ = ["ROS2LidarBridge"]
