"""Utilities to process PiLiDAR data using NumPy point clouds."""

from __future__ import annotations

import math
import os
import pickle
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import matplotlib
import numpy as np
from scipy.spatial import cKDTree

from .ros2_bridge import ROS2LidarBridge


@dataclass
class NumpyPointCloud:
    """Light-weight representation of a point cloud backed by NumPy arrays."""

    points: np.ndarray
    attributes: Dict[str, np.ndarray] = field(default_factory=dict)
    normals: Optional[np.ndarray] = None

    def copy(self) -> "NumpyPointCloud":
        return NumpyPointCloud(
            points=self.points.copy(),
            attributes={name: values.copy() for name, values in self.attributes.items()},
            normals=None if self.normals is None else self.normals.copy(),
        )

    def set_attribute(self, name: str, values: np.ndarray) -> None:
        self.attributes[name] = np.asarray(values)

    def get_attribute(self, name: str) -> Optional[np.ndarray]:
        return self.attributes.get(name)

    @property
    def colors(self) -> Optional[np.ndarray]:
        return self.attributes.get("color")

    def __len__(self) -> int:  # pragma: no cover - trivial
        return int(self.points.shape[0])


def get_scan_dict(
    z_angles,
    angular_list: Optional[Sequence] = None,
    cartesian_list: Optional[Sequence] = None,
    scan_id: Optional[str] = None,
    device_id: Optional[str] = None,
    sensor: Optional[str] = None,
    hardware: Optional[str] = None,
    location: Optional[str] = None,
    author: Optional[str] = None,
    packages: Optional[List[Dict[str, np.ndarray]]] = None,
):
    """Create a dictionary containing all raw LiDAR measurements."""

    raw_scan = {
        "header": {
            "scan_id": scan_id,
            "device_id": device_id,
            "sensor": sensor,
            "hardware": hardware,
            "location": location,
            "author": author,
        },
        "z_angles": z_angles,
        "angular": angular_list,
        "cartesian": cartesian_list,
        "packages": packages,
    }
    return raw_scan


def save_raw_scan(path: str, data: Dict[str, object]) -> None:
    if isinstance(data, dict):
        with open(path, "wb") as file:
            pickle.dump(data, file)


def load_raw_scan(path: str) -> Dict[str, object]:
    with open(path, "rb") as file:
        return pickle.load(file)


def process_raw(config, save: bool = True) -> NumpyPointCloud:
    """Convert raw lidar data into a NumPy based point cloud."""

    if not os.path.exists(config.raw_path):
        raise FileNotFoundError(f"Raw scan not found: {config.raw_path}")

    raw_scan = load_raw_scan(config.raw_path)

    points, attributes = merge_2D_points(
        raw_scan,
        position_offset=(0, config.get("3D", "Y_OFFSET"), 0),
        angle_offset=config.get("LIDAR", "LIDAR_OFFSET_ANGLE"),
        up_vector=(0, 0, 1),
    )

    pointcloud = NumpyPointCloud(points=points, attributes=attributes)

    # Apply vertical offset and scale to convert from millimetres to metres.
    pointcloud.points[:, 2] += config.get("3D", "Z_OFFSET")
    scale = config.get("3D", "SCALE")
    if scale != 1:
        pointcloud.points *= scale

    normal_radius = config.get("3D", "NORMAL_RADIUS")
    if normal_radius and len(pointcloud) > 0:
        estimate_point_normals(pointcloud, radius=normal_radius, max_nn=50)

    pano_path = config.pano_path
    if config.get("ENABLE_VERTEXCOLOUR") and os.path.exists(pano_path):
        angular = angular_from_cartesian(pointcloud.points)
        colors = angular_lookup(
            angular,
            cv2.imread(pano_path),
            scale=config.get("VERTEXCOLOUR", "SCALE"),
            z_rotate=config.get("VERTEXCOLOUR", "Z_ROTATE"),
        )
        pointcloud.set_attribute("color", colors)
    else:
        colormap_pcd(pointcloud, gamma=1, cmap="viridis")

    if save:
        save_pointcloud_threaded(pointcloud, config.pcd_path, ply_ascii=config.get("3D", "ASCII"))

    if config.get("ENABLE_FILTERING"):
        voxel_size = config.get("FILTERING", "VOXEL_SIZE")
        nb_points = config.get("FILTERING", "NB_POINTS")
        radius = config.get("FILTERING", "RADIUS")

        downsampled = downsample(pointcloud, voxel_size=voxel_size)
        filtered = filter_outliers(downsampled, nb_points=nb_points, radius=radius)
        refined = filter_by_reference(pointcloud, filtered, radius=radius)
        if save:
            save_pointcloud_threaded(refined, config.filtered_pcd_path, ply_ascii=config.get("3D", "ASCII"))
        pointcloud = refined

    ROS2LidarBridge.get_instance(config.get("ENABLE_ROS2")).publish_pointcloud(pointcloud)

    return pointcloud


def create_fusion_visualizations(config, pointcloud: NumpyPointCloud, pano_path: Optional[str] = None):
    """Generate images that overlay the LiDAR measurements with the panorama."""

    os.makedirs(config.fusion_dir, exist_ok=True)

    pano_path = pano_path or config.pano_path
    pano_image = cv2.imread(pano_path) if pano_path and os.path.exists(pano_path) else None

    pano_width = config.get("PANO", "PANO_WIDTH")
    pano_height = pano_width // 2
    lidar_projection = get_lidar_pano(pointcloud, image_width=pano_width, image_height=pano_height)
    cv2.imwrite(config.lidar_projection_path, lidar_projection)

    lidar_coloured = cv2.applyColorMap(lidar_projection, cv2.COLORMAP_TURBO)

    if pano_image is not None:
        pano_resized = cv2.resize(
            pano_image,
            (lidar_coloured.shape[1], lidar_coloured.shape[0]),
            interpolation=cv2.INTER_AREA,
        )
        fusion = cv2.addWeighted(pano_resized, 0.6, lidar_coloured, 0.4, 0)
    else:
        fusion = lidar_coloured

    cv2.imwrite(config.fusion_overlay_path, fusion)

    return {
        "lidar_projection": config.lidar_projection_path,
        "fusion": config.fusion_overlay_path,
    }


def save_pointcloud(
    pointcloud: NumpyPointCloud,
    filepath: str,
    ply_ascii: bool = True,
    csv_delimiter: str = ",",
) -> None:
    directory, filename = os.path.split(filepath)
    os.makedirs(directory, exist_ok=True)

    ext = os.path.splitext(filename)[1][1:].lower()
    if ext == "csv":
        attribute_values = [values for values in pointcloud.attributes.values()]
        array = np.column_stack([pointcloud.points] + attribute_values)
        np.savetxt(filepath, array, delimiter=csv_delimiter)
        return

    if ext != "ply":
        raise ValueError(f"Unsupported file extension: {ext}")

    colors = pointcloud.colors
    extra_attributes = {name: values for name, values in pointcloud.attributes.items() if name != "color"}

    header = [
        "ply",
        "format ascii 1.0" if ply_ascii else "format ascii 1.0",
        f"element vertex {len(pointcloud)}",
        "property float x",
        "property float y",
        "property float z",
    ]

    if colors is not None and colors.shape[1] == 3:
        header.extend([
            "property uchar red",
            "property uchar green",
            "property uchar blue",
        ])

    for name, values in extra_attributes.items():
        values = np.asarray(values)
        if values.ndim == 1:
            header.append(f"property float {name}")
        else:
            for index in range(values.shape[1]):
                header.append(f"property float {name}_{index}")

    header.append("end_header")

    with open(filepath, "w", encoding="utf-8") as file:
        for line in header:
            file.write(line + "\n")

        for idx in range(len(pointcloud)):
            row: List[str] = [
                f"{pointcloud.points[idx, 0]:.6f}",
                f"{pointcloud.points[idx, 1]:.6f}",
                f"{pointcloud.points[idx, 2]:.6f}",
            ]

            if colors is not None and colors.shape[0] > idx:
                rgb = np.clip(colors[idx] * 255, 0, 255).astype(np.uint8)
                row.extend(str(value) for value in rgb)

            for name, values in extra_attributes.items():
                value = values[idx]
                if np.isscalar(value):
                    row.append(f"{float(value):.6f}")
                elif np.asarray(value).ndim == 1:
                    row.extend(f"{float(component):.6f}" for component in value)
                else:
                    row.append(f"{float(value):.6f}")

            file.write(" ".join(row) + "\n")


def save_pointcloud_threaded(pointcloud: NumpyPointCloud, output_path: str, ply_ascii: bool = True) -> None:
    export_thread = threading.Thread(target=save_pointcloud, args=(pointcloud, output_path, ply_ascii))
    export_thread.start()


def remove_NaN(array: np.ndarray) -> np.ndarray:
    array = np.asarray(array)
    if array.ndim == 1:
        return array[~np.isnan(array)]
    return array[~np.isnan(array).any(axis=1)]


def merge_2D_points(
    raw_scan: Dict[str, object],
    z_step: float = 1,
    ccw: bool = False,
    position_offset: Sequence[float] = (0, 0, 0),
    angle_offset: float = 0,
    up_vector: Sequence[float] = (0, 0, 1),
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    z_angles = raw_scan.get("z_angles") or []
    cartesian_list: Sequence[np.ndarray] = raw_scan.get("cartesian") or []

    points_3d: List[np.ndarray] = []
    distance_list: List[np.ndarray] = []
    intensity_list: List[np.ndarray] = []
    angle_list: List[np.ndarray] = []
    speed_list: List[np.ndarray] = []
    timestamp_list: List[np.ndarray] = []
    z_angle_list: List[np.ndarray] = []

    z_angle_accumulator = 0.0

    for index, plane in enumerate(cartesian_list):
        plane = np.asarray(plane, dtype=np.float32)
        if plane.size == 0:
            continue

        if plane.shape[1] < 3:
            raise ValueError("Each plane must contain at least three columns")

        # Legacy data only contained x, y, intensity. Fill missing columns.
        if plane.shape[1] < 8:
            missing = 8 - plane.shape[1]
            plane = np.column_stack((plane, np.zeros((plane.shape[0], missing), dtype=np.float32)))

        local_points = np.column_stack((plane[:, 0], np.zeros(len(plane)), plane[:, 1]))

        if z_angles:
            z_angle = float(z_angles[index])
        else:
            z_angle_accumulator += -z_step if ccw else z_step
            z_angle = z_angle_accumulator

        rotated = rotate_3D(
            local_points,
            rotation_degrees=angle_offset,
            rotation_axis=(0, 1, 0),
        )
        rotated = rotate_3D(
            rotated,
            rotation_degrees=-z_angle,
            rotation_axis=up_vector,
            translation_vector=position_offset,
        )

        points_3d.append(rotated)
        distance_list.append(plane[:, 2])
        intensity_list.append(plane[:, 3])
        angle_list.append(plane[:, 4])
        speed_list.append(plane[:, 5])
        timestamp_list.append(plane[:, 6])
        z_angle_list.append(np.full(len(plane), z_angle, dtype=np.float32))

    if not points_3d:
        return np.empty((0, 3), dtype=np.float32), {}

    points = np.vstack(points_3d).astype(np.float32)
    distances = np.concatenate(distance_list)
    intensities = np.concatenate(intensity_list)
    angles = np.concatenate(angle_list)
    speeds = np.concatenate(speed_list)
    timestamps = np.concatenate(timestamp_list)
    z_angle_values = np.concatenate(z_angle_list)

    mask = ~np.isnan(points).any(axis=1)
    points = points[mask]

    attributes = {
        "distance": distances[mask].astype(np.float32),
        "intensity": intensities[mask].astype(np.float32),
        "angle": angles[mask].astype(np.float32),
        "speed": speeds[mask].astype(np.float32),
        "timestamp": timestamps[mask].astype(np.float32),
        "z_angle": z_angle_values[mask].astype(np.float32),
    }

    return points, attributes


def rotate_3D(
    points3d: np.ndarray,
    rotation_degrees: float,
    translation_vector: Sequence[float] = (0, 0, 0),
    rotation_axis: Sequence[float] = (0, 0, 1),
) -> np.ndarray:
    axis = np.asarray(rotation_axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm == 0:
        return points3d
    axis = axis / norm
    theta = math.radians(rotation_degrees)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    ux, uy, uz = axis
    rotation_matrix = np.array(
        [
            [
                cos_theta + ux * ux * (1 - cos_theta),
                ux * uy * (1 - cos_theta) - uz * sin_theta,
                ux * uz * (1 - cos_theta) + uy * sin_theta,
            ],
            [
                uy * ux * (1 - cos_theta) + uz * sin_theta,
                cos_theta + uy * uy * (1 - cos_theta),
                uy * uz * (1 - cos_theta) - ux * sin_theta,
            ],
            [
                uz * ux * (1 - cos_theta) - uy * sin_theta,
                uz * uy * (1 - cos_theta) + ux * sin_theta,
                cos_theta + uz * uz * (1 - cos_theta),
            ],
        ]
    )

    translated = points3d + np.asarray(translation_vector, dtype=np.float64)
    rotated = translated @ rotation_matrix.T
    return rotated.astype(np.float32)


def downsample(pointcloud: NumpyPointCloud, voxel_size: float = 0.02) -> NumpyPointCloud:
    if len(pointcloud) == 0 or voxel_size <= 0:
        return pointcloud.copy()

    coords = np.floor(pointcloud.points / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(coords, axis=0, return_index=True)
    unique_indices = np.sort(unique_indices)

    downsampled = pointcloud.points[unique_indices]
    attributes = {
        name: values[unique_indices]
        for name, values in pointcloud.attributes.items()
    }

    result = NumpyPointCloud(points=downsampled.astype(np.float32), attributes=attributes)
    if pointcloud.normals is not None:
        result.normals = pointcloud.normals[unique_indices]
    return result


def filter_outliers(pointcloud: NumpyPointCloud, nb_points: int = 20, radius: float = 0.5) -> NumpyPointCloud:
    if len(pointcloud) == 0:
        return pointcloud.copy()

    tree = cKDTree(pointcloud.points)
    mask = np.zeros(len(pointcloud), dtype=bool)
    for index, point in enumerate(pointcloud.points):
        neighbours = tree.query_ball_point(point, radius)
        mask[index] = len(neighbours) >= nb_points

    filtered_points = pointcloud.points[mask]
    attributes = {
        name: values[mask]
        for name, values in pointcloud.attributes.items()
    }
    result = NumpyPointCloud(points=filtered_points.astype(np.float32), attributes=attributes)
    if pointcloud.normals is not None:
        result.normals = pointcloud.normals[mask]
    return result


def filter_by_reference(
    pointcloud: NumpyPointCloud,
    reference: NumpyPointCloud,
    radius: float = 0.02,
) -> NumpyPointCloud:
    if len(pointcloud) == 0 or len(reference) == 0:
        return pointcloud.copy()

    tree = cKDTree(reference.points)
    mask = np.zeros(len(pointcloud), dtype=bool)
    for index, point in enumerate(pointcloud.points):
        neighbours = tree.query_ball_point(point, radius)
        mask[index] = len(neighbours) > 0

    filtered_points = pointcloud.points[mask]
    attributes = {
        name: values[mask]
        for name, values in pointcloud.attributes.items()
    }
    result = NumpyPointCloud(points=filtered_points.astype(np.float32), attributes=attributes)
    if pointcloud.normals is not None:
        result.normals = pointcloud.normals[mask]
    return result


def get_lidar_pano(pointcloud: NumpyPointCloud, image_width: int, image_height: int) -> np.ndarray:
    intensity = pointcloud.get_attribute("intensity")
    if intensity is None or len(intensity) == 0:
        return np.zeros((image_height, image_width), dtype=np.uint8)

    angular_points = angular_from_cartesian(pointcloud.points)
    image_x, image_y = get_sampling_coordinates(angular_points, (image_height, image_width))

    panorama = np.zeros((image_height, image_width), dtype=np.uint8)
    values = np.clip(intensity, 0, 255).astype(np.uint8)
    panorama[image_y, image_x] = values
    panorama = cv2.medianBlur(panorama, 3)
    return panorama


def colormap_pcd(pointcloud: NumpyPointCloud, cmap: str = "viridis", gamma: float = 2.2) -> NumpyPointCloud:
    intensity = pointcloud.get_attribute("intensity")
    if intensity is None or len(intensity) == 0:
        colors = np.zeros((len(pointcloud), 3), dtype=np.float32)
        pointcloud.set_attribute("color", colors)
        return pointcloud

    intensities = np.asarray(intensity, dtype=np.float32)
    min_val = intensities.min()
    max_val = intensities.max()
    if max_val - min_val < 1e-9:
        normalized = np.zeros_like(intensities)
    else:
        normalized = (intensities - min_val) / (max_val - min_val)
    corrected = np.power(normalized, gamma)
    colors = matplotlib.colormaps[cmap](corrected)
    if colors.shape[1] == 4:
        colors = colors[:, :3]
    pointcloud.set_attribute("color", colors.astype(np.float32))
    return pointcloud


def estimate_point_normals(
    pointcloud: NumpyPointCloud,
    radius: float = 1.0,
    max_nn: int = 30,
    center: Sequence[float] = (0, 0, 0),
) -> NumpyPointCloud:
    if len(pointcloud) == 0:
        return pointcloud

    tree = cKDTree(pointcloud.points)
    normals = np.zeros_like(pointcloud.points)
    center = np.asarray(center, dtype=np.float32)

    for index, point in enumerate(pointcloud.points):
        indices = tree.query_ball_point(point, radius)
        if len(indices) < 3:
            normals[index] = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            continue
        if len(indices) > max_nn:
            indices = indices[:max_nn]
        neighbours = pointcloud.points[indices]
        mean = neighbours.mean(axis=0)
        centred = neighbours - mean
        covariance = centred.T @ centred / max(len(indices), 1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        if np.dot(normal, point - center) > 0:
            normal = -normal
        normals[index] = normal

    pointcloud.normals = normals.astype(np.float32)
    return pointcloud


def transform(
    pointcloud: NumpyPointCloud,
    transformation: Optional[np.ndarray] = None,
    translate: Optional[Sequence[float]] = None,
    scale: Optional[float] = None,
    euler_rotate_deg: Optional[Sequence[float]] = None,
    pivot: Sequence[float] = (0, 0, 0),
) -> NumpyPointCloud:
    result = pointcloud.copy()

    if transformation is not None:
        matrix = np.asarray(transformation, dtype=np.float64)
        homogenous = np.column_stack((result.points, np.ones(len(result))))
        transformed = homogenous @ matrix.T
        result.points = transformed[:, :3].astype(np.float32)

    if translate is not None:
        result.points += np.asarray(translate, dtype=np.float32)

    if euler_rotate_deg is not None:
        angles = np.deg2rad(np.asarray(euler_rotate_deg, dtype=np.float64))
        rx, ry, rz = angles
        Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
        Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
        Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
        rotation_matrix = Rz @ Ry @ Rx
        pivot_vec = np.asarray(pivot, dtype=np.float32)
        result.points = (result.points - pivot_vec) @ rotation_matrix.T + pivot_vec

    if scale is not None:
        pivot_vec = np.asarray(pivot, dtype=np.float32)
        result.points = (result.points - pivot_vec) * scale + pivot_vec

    return result


def angular_from_cartesian(cartesian_points: np.ndarray) -> np.ndarray:
    r = np.sqrt(np.sum(cartesian_points**2, axis=1)) + 1e-10
    theta = np.arccos(cartesian_points[:, 2] / r)
    phi = np.arctan2(cartesian_points[:, 1], cartesian_points[:, 0])
    return np.stack([theta, r, phi], axis=1)


def get_sampling_coordinates(angular_points: np.ndarray, img_shape: Tuple[int, int], z_rotate: float = 0):
    image_height, image_width = img_shape

    longitude = angular_points[:, 2] + np.deg2rad(90 + z_rotate)
    longitude = (longitude + 2 * np.pi) % (2 * np.pi)
    image_x = (2 * np.pi - longitude) / (2 * np.pi) * image_width
    image_x = np.round(image_x).astype(int)
    image_x = np.clip(image_x, 0, image_width - 1)

    latitude = np.pi / 2 - angular_points[:, 0]
    latitude = (latitude + np.pi / 2) % np.pi
    image_y = (1 - latitude / np.pi) * image_height
    image_y = np.round(image_y).astype(int)
    image_y = np.clip(image_y, 0, image_height - 1)

    return image_x, image_y


def angular_lookup(
    angular_points: np.ndarray,
    pano: np.ndarray,
    scale: float = 1,
    degrees: bool = False,
    z_rotate: float = 0,
    as_float: bool = True,
):
    if degrees:
        angular_points = np.deg2rad(angular_points)

    if pano is None:
        return np.zeros((angular_points.shape[0], 3), dtype=np.float32)

    image_height, image_width, _ = pano.shape
    pano_rgb = cv2.cvtColor(pano, cv2.COLOR_BGR2RGB)

    if scale != 1:
        image_height = int(image_height * scale)
        image_width = int(image_height * 2)
        pano_rgb = cv2.resize(pano_rgb, (image_width, image_height), interpolation=cv2.INTER_AREA)

    image_x, image_y = get_sampling_coordinates(angular_points, (image_height, image_width), z_rotate=z_rotate)
    colors = pano_rgb[image_y, image_x]

    if as_float:
        colors = colors.astype(np.float32) / 255
    return colors


__all__ = [
    "NumpyPointCloud",
    "angular_from_cartesian",
    "angular_lookup",
    "colormap_pcd",
    "create_fusion_visualizations",
    "downsample",
    "estimate_point_normals",
    "filter_by_reference",
    "filter_outliers",
    "get_lidar_pano",
    "get_sampling_coordinates",
    "get_scan_dict",
    "load_raw_scan",
    "merge_2D_points",
    "process_raw",
    "remove_NaN",
    "rotate_3D",
    "save_pointcloud",
    "save_pointcloud_threaded",
    "save_raw_scan",
    "transform",
]
