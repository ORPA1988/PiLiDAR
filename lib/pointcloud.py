"""Point cloud processing helpers built around the Point Cloud Library (PCL)."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from threading import Thread
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from .pcl_bindings import (
    PointCloudProtocol,
    apply_colormap,
    create_point_cloud,
    estimate_normals,
    radius_outlier_removal,
    voxel_down_sample,
    write_ply,
)


@dataclass
class ProcessedPointClouds:
    """Container returned by :func:`process_raw`."""

    intensity: Optional[PointCloudProtocol]
    color: Optional[PointCloudProtocol]
    filtered: Optional[PointCloudProtocol]


def get_scan_dict(
    z_angles: Iterable[float],
    angular_list: Optional[Iterable[np.ndarray]] = None,
    cartesian_list: Optional[Iterable[np.ndarray]] = None,
    packages: Optional[Iterable[Dict[str, object]]] = None,
    metadata: Optional[Dict[str, object]] = None,
    **header,
) -> Dict[str, object]:
    data = {
        "header": header,
        "z_angles": list(z_angles) if z_angles is not None else [],
        "angular": list(angular_list) if angular_list is not None else [],
        "cartesian": list(cartesian_list) if cartesian_list is not None else [],
        "packages": list(packages) if packages is not None else [],
    }
    if metadata:
        data["header"].update(metadata)
    return data


def save_raw_scan(path: str, data: Dict[str, object]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as handle:
        np.save(handle, data, allow_pickle=True)


def load_raw_scan(path: str) -> Dict[str, object]:
    with open(path, "rb") as handle:
        loaded = np.load(handle, allow_pickle=True)
    return loaded.item()


def process_raw(config, save: bool = True) -> ProcessedPointClouds:
    if not os.path.exists(config.raw_path):
        raise FileNotFoundError(f"Raw LiDAR data not found at {config.raw_path}")

    raw_scan = load_raw_scan(config.raw_path)
    merged = merge_2d_slices(
        raw_scan,
        position_offset=(0, config.get("3D", "Y_OFFSET"), 0),
        angle_offset=config.get("LIDAR", "LIDAR_OFFSET_ANGLE"),
        up_vector=(0, 0, 1),
    )

    if merged.size == 0:
        return ProcessedPointClouds(intensity=None, color=None, filtered=None)

    xyz = merged[:, :3]
    intensities = merged[:, 3]
    intensity_norm = normalise_intensity(intensities)

    cloud = create_point_cloud(xyz, intensities=intensity_norm[:, None])
    estimate_normals(cloud, radius=config.get("3D", "NORMAL_RADIUS"), max_nn=50)

    translate = config.get("3D", "Z_OFFSET")
    if translate:
        cloud.translate((0.0, 0.0, translate))
    scale = config.get("3D", "SCALE")
    if scale and scale != 1:
        cloud.scale(scale)

    intensity_colors = apply_colormap(intensity_norm)
    intensity_cloud = create_point_cloud(np.copy(cloud.points), colors=intensity_colors, intensities=intensity_norm[:, None])

    color_cloud = None
    if config.get("ENABLE_VERTEXCOLOUR") and os.path.exists(config.pano_path):
        colors = map_colors_from_panorama(cloud.points, config.pano_path, config)
        if colors is not None:
            color_cloud = create_point_cloud(np.copy(cloud.points), colors=colors, intensities=intensity_norm[:, None])

    filtered_cloud = None
    if config.get("ENABLE_FILTERING"):
        voxel_size = config.get("FILTERING", "VOXEL_SIZE")
        nb_points = config.get("FILTERING", "NB_POINTS")
        radius = config.get("FILTERING", "RADIUS")

        low_density = voxel_down_sample(intensity_cloud, voxel_size)
        filtered_low = radius_outlier_removal(low_density, radius=radius, min_neighbours=nb_points)
        filtered_cloud = filter_by_reference(intensity_cloud, filtered_low, radius=radius)

    if save:
        save_cloud_async(intensity_cloud, config.intensity_pcd_path)
        if color_cloud is not None:
            save_cloud_async(color_cloud, config.vertex_pcd_path)
        if filtered_cloud is not None:
            save_cloud_async(filtered_cloud, config.filtered_pcd_path)

    return ProcessedPointClouds(
        intensity=intensity_cloud,
        color=color_cloud,
        filtered=filtered_cloud,
    )


def merge_2d_slices(
    raw_scan: Dict[str, object],
    position_offset: Tuple[float, float, float],
    angle_offset: float,
    up_vector: Tuple[float, float, float],
) -> np.ndarray:
    z_angles = raw_scan.get("z_angles") or []
    cartesian_list: List[np.ndarray] = raw_scan.get("cartesian", [])
    if not cartesian_list:
        return np.empty((0, 4), dtype=np.float32)

    pointcloud = []
    last_angle = 0.0
    for index, points in enumerate(cartesian_list):
        if not len(points):
            continue

        slice_points = np.insert(np.asarray(points, dtype=np.float32), 1, 0.0, axis=1)
        z_angle = z_angles[index] if index < len(z_angles) else last_angle
        last_angle = z_angle

        rotated = rotate_points(slice_points, angle_offset, axis=np.array((0, 1, 0)))
        rotated = rotate_points(
            rotated,
            -z_angle,
            translation=np.asarray(position_offset, dtype=np.float32),
            axis=np.asarray(up_vector, dtype=np.float32),
        )
        pointcloud.append(rotated)

    if not pointcloud:
        return np.empty((0, 4), dtype=np.float32)

    merged = np.concatenate(pointcloud, axis=0)
    merged = merged[~np.isnan(merged).any(axis=1)]
    return merged


def rotate_points(
    points: np.ndarray,
    angle_deg: float,
    axis: np.ndarray,
    translation: Optional[np.ndarray] = None,
) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    rotation_vector = np.deg2rad(angle_deg) * axis / np.linalg.norm(axis)
    matrix = Rotation.from_rotvec(rotation_vector).as_matrix()

    coords = points[:, :3]
    if translation is not None:
        coords = coords + translation
    rotated = (matrix @ coords.T).T
    result = np.column_stack((rotated, points[:, 3]))
    return result


def normalise_intensity(values: np.ndarray) -> np.ndarray:
    values = values.astype(np.float32)
    if not values.size:
        return values
    min_val = np.nanmin(values)
    max_val = np.nanmax(values)
    if max_val - min_val == 0:
        return np.zeros_like(values)
    return (values - min_val) / (max_val - min_val)


def map_colors_from_panorama(points: np.ndarray, pano_path: str, config) -> Optional[np.ndarray]:
    try:
        cv2 = _require_cv2()
    except RuntimeError:
        return None

    pano = cv2.imread(pano_path)
    if pano is None:
        return None

    angular = angular_from_cartesian(points)
    colors = angular_lookup(
        angular,
        pano,
        scale=config.get("VERTEXCOLOUR", "SCALE"),
        z_rotate=config.get("VERTEXCOLOUR", "Z_ROTATE"),
    )
    return colors.astype(np.float32) / 255.0


def angular_from_cartesian(cartesian: np.ndarray) -> np.ndarray:
    r = np.linalg.norm(cartesian, axis=1) + 1e-9
    theta = np.arccos(np.clip(cartesian[:, 2] / r, -1, 1))
    phi = np.arctan2(cartesian[:, 1], cartesian[:, 0])
    return np.column_stack((theta, r, phi))


def angular_lookup(
    angular_points: np.ndarray,
    pano: np.ndarray,
    scale: float = 1.0,
    z_rotate: float = 0.0,
) -> np.ndarray:
    cv2 = _require_cv2()
    image_height, image_width, _ = pano.shape
    pano_rgb = cv2.cvtColor(pano, cv2.COLOR_BGR2RGB)

    if scale != 1.0:
        image_height = int(image_height * scale)
        image_width = int(image_height * 2)
        pano_rgb = cv2.resize(pano_rgb, (image_width, image_height), interpolation=cv2.INTER_AREA)

    longitude = angular_points[:, 2] + np.deg2rad(90 + z_rotate)
    longitude = (longitude + 2 * np.pi) % (2 * np.pi)
    image_x = (2 * np.pi - longitude) / (2 * np.pi) * image_width
    image_x = np.clip(np.round(image_x).astype(int), 0, image_width - 1)

    latitude = np.pi / 2 - angular_points[:, 0]
    latitude = (latitude + np.pi / 2) % np.pi
    image_y = (1 - latitude / np.pi) * image_height
    image_y = np.clip(np.round(image_y).astype(int), 0, image_height - 1)

    return pano_rgb[image_y, image_x]


def filter_by_reference(
    source: PointCloudProtocol,
    reference: PointCloudProtocol,
    radius: float,
) -> PointCloudProtocol:
    from scipy.spatial import cKDTree

    tree = cKDTree(reference.points)
    mask = []
    for point in source.points:
        idx = tree.query_ball_point(point, radius)
        mask.append(len(idx) > 0)
    mask = np.asarray(mask)
    return create_point_cloud(
        points=source.points[mask],
        colors=source.colors[mask],
        intensities=source.intensities[mask],
    )


def save_cloud_async(cloud: PointCloudProtocol, path: str) -> None:
    thread = Thread(target=write_ply, args=(path, cloud))
    thread.daemon = True
    thread.start()


def load_pointcloud(path: str) -> PointCloudProtocol:
    with open(path, "rb") as handle:
        raise NotImplementedError("Loading PLY files is handled by the C++ backend on the Pi")


def save_pointcloud(cloud: PointCloudProtocol, path: str) -> None:
    write_ply(path, cloud)


def _require_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:  # pragma: no cover - optional dependency missing
        raise RuntimeError("OpenCV wird für die Farbprojektion benötigt.") from exc
    return cv2


def downsample(cloud: PointCloudProtocol, voxel_size: float) -> PointCloudProtocol:
    return voxel_down_sample(cloud, voxel_size)


def filter_outliers(cloud: PointCloudProtocol, nb_points: int, radius: float) -> PointCloudProtocol:
    return radius_outlier_removal(cloud, radius=radius, min_neighbours=nb_points)


def print_stats(cloud: PointCloudProtocol, txt: str = "") -> None:
    mins = np.min(cloud.points, axis=0)
    maxs = np.max(cloud.points, axis=0)
    extent = np.round(maxs - mins, 3)
    print(f"{txt} points: {len(cloud.points)}, bbox_extent: {tuple(extent)}")


def estimate_point_normals(cloud: PointCloudProtocol, radius: float, max_nn: int) -> PointCloudProtocol:
    estimate_normals(cloud, radius=radius, max_nn=max_nn)
    return cloud


def transform(
    cloud: PointCloudProtocol,
    translate: Optional[Tuple[float, float, float]] = None,
    scale: Optional[float] = None,
    rotation_matrix: Optional[np.ndarray] = None,
) -> PointCloudProtocol:
    if translate is not None:
        cloud.translate(translate)
    if rotation_matrix is not None:
        cloud.rotate(rotation_matrix)
    if scale is not None:
        cloud.scale(scale)
    return cloud


def save_summary(path: str, summary: Dict[str, object]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)


__all__ = [
    "ProcessedPointClouds",
    "get_scan_dict",
    "save_raw_scan",
    "load_raw_scan",
    "process_raw",
    "merge_2d_slices",
    "angular_from_cartesian",
    "angular_lookup",
    "downsample",
    "filter_outliers",
    "filter_by_reference",
    "estimate_point_normals",
    "transform",
    "save_pointcloud",
    "save_summary",
]

