"""Thin abstraction layer around the Point Cloud Library (PCL).

The real project uses a custom C++ extension (``pilidar_pcl``) that exposes
just the tiny subset of PCL functionality required for the Raspberry Pi
scanner.  The module is compiled with :mod:`pybind11` and ships with the
project when deployed to the Pi.  Unit tests inside this kata however run on a
generic Linux container that does not provide PCL.  To keep the Python code
fully testable we fall back to small NumPy based implementations when the
native bindings are not available.

The fallback mirrors the public API that the rest of the project uses.  When
``pilidar_pcl`` is installed the ``PointCloud`` class defined in the pybind
module is used directly.  Otherwise a pure Python stand-in is instantiated.
The pure Python version only implements the features that are required by the
scanner pipeline (voxel grid down sampling, radius outlier removal, normal
estimation and exporting to ``PLY``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np


try:  # pragma: no cover - exercised on the Raspberry Pi
    from pilidar_pcl import PointCloud as _PCLPointCloud  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - executed in the test env
    _PCLPointCloud = None


def _ensure_array(data: Optional[np.ndarray], dtype=np.float32) -> np.ndarray:
    if data is None:
        return np.empty((0, 1), dtype=dtype)
    array = np.asarray(data, dtype=dtype)
    if array.ndim == 1:
        array = array[:, None]
    return array


@dataclass
class _FallbackPointCloud:
    """Light‑weight point cloud representation used during tests."""

    points: np.ndarray
    colors: np.ndarray
    intensities: np.ndarray
    normals: np.ndarray

    def __post_init__(self) -> None:
        self.points = _ensure_array(self.points, dtype=np.float32)
        if self.points.shape[1] != 3:
            raise ValueError("points must be shaped Nx3")

        self.colors = _ensure_array(self.colors, dtype=np.float32)
        if self.colors.size == 0:
            self.colors = np.zeros_like(self.points)
        if self.colors.shape[1] != 3:
            raise ValueError("colors must be shaped Nx3")

        self.intensities = _ensure_array(self.intensities, dtype=np.float32)
        if self.intensities.size == 0:
            self.intensities = np.zeros((self.points.shape[0], 1), dtype=np.float32)

        self.normals = _ensure_array(self.normals, dtype=np.float32)
        if self.normals.size == 0:
            self.normals = np.zeros_like(self.points)

    # ------------------------------------------------------------------
    # PCL like operations
    # ------------------------------------------------------------------
    def estimate_normals(self, radius: float, max_nn: int) -> None:
        from scipy.spatial import cKDTree

        tree = cKDTree(self.points)
        normals = np.zeros_like(self.points)
        for idx, point in enumerate(self.points):
            k = min(max_nn, len(self.points))
            _, neighbours = tree.query(point, k=k)
            if np.isscalar(neighbours):
                neighbours = [neighbours]
            neighbour_pts = self.points[neighbours]
            centred = neighbour_pts - neighbour_pts.mean(axis=0)
            cov = centred.T @ centred
            eigvals, eigvecs = np.linalg.eigh(cov)
            normals[idx] = eigvecs[:, np.argmin(eigvals)]
        self.normals = normals

    def voxel_down_sample(self, voxel_size: float) -> "_FallbackPointCloud":
        if voxel_size <= 0:
            return self

        quantized = np.floor(self.points / voxel_size)
        _, unique_idx = np.unique(quantized, axis=0, return_index=True)
        unique_idx = np.sort(unique_idx)
        return _FallbackPointCloud(
            points=self.points[unique_idx],
            colors=self.colors[unique_idx],
            intensities=self.intensities[unique_idx],
            normals=self.normals[unique_idx],
        )

    def radius_outlier_removal(self, radius: float, min_neighbours: int) -> "_FallbackPointCloud":
        from scipy.spatial import cKDTree

        if radius <= 0:
            return self

        tree = cKDTree(self.points)
        mask = []
        for point in self.points:
            idx = tree.query_ball_point(point, r=radius)
            mask.append(len(idx) >= min_neighbours)
        mask = np.asarray(mask)
        return _FallbackPointCloud(
            points=self.points[mask],
            colors=self.colors[mask],
            intensities=self.intensities[mask],
            normals=self.normals[mask],
        )

    def translate(self, vector: Iterable[float]) -> None:
        self.points += np.asarray(vector, dtype=np.float32)

    def scale(self, factor: float) -> None:
        self.points *= factor

    def rotate(self, rotation_matrix: np.ndarray) -> None:
        self.points = (rotation_matrix @ self.points.T).T


def create_point_cloud(
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
    intensities: Optional[np.ndarray] = None,
) -> "PointCloudProtocol":
    if _PCLPointCloud is not None:  # pragma: no cover - requires the pybind module
        cloud = _PCLPointCloud(points)
        if colors is not None:
            cloud.set_colors(colors.astype(np.float32))
        if intensities is not None:
            cloud.set_intensities(intensities.astype(np.float32))
        return cloud
    return _FallbackPointCloud(points=points, colors=colors, intensities=intensities, normals=None)


def apply_colormap(intensities: np.ndarray, colormap: str = "viridis") -> np.ndarray:
    from matplotlib import colormaps

    norm = np.clip(intensities, 0, 1)
    cmap = colormaps[colormap]
    return cmap(norm)[..., :3].astype(np.float32)


def estimate_normals(cloud: "PointCloudProtocol", radius: float, max_nn: int) -> None:
    cloud.estimate_normals(radius=radius, max_nn=max_nn)


def voxel_down_sample(cloud: "PointCloudProtocol", voxel_size: float) -> "PointCloudProtocol":
    return cloud.voxel_down_sample(voxel_size)


def radius_outlier_removal(
    cloud: "PointCloudProtocol", radius: float, min_neighbours: int
) -> "PointCloudProtocol":
    return cloud.radius_outlier_removal(radius=radius, min_neighbours=min_neighbours)


def write_ply(
    path: str,
    cloud: "PointCloudProtocol",
    include_intensity: bool = True,
) -> None:
    points = np.asarray(cloud.points, dtype=np.float32)
    colors = np.asarray(getattr(cloud, "colors", np.zeros_like(points)), dtype=np.float32)
    intensities = np.asarray(getattr(cloud, "intensities", np.zeros((points.shape[0], 1))), dtype=np.float32)

    header = [
        "ply",
        "format ascii 1.0",
        f"element vertex {points.shape[0]}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
    ]
    if include_intensity:
        header.append("property float intensity")
    header.append("end_header")

    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(header) + "\n")
        for point, color, intensity in zip(points, colors, intensities):
            r, g, b = (np.clip(color, 0, 1) * 255).astype(np.uint8)
            if include_intensity:
                handle.write(
                    f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {r} {g} {b} {float(intensity)}\n"
                )
            else:
                handle.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {r} {g} {b}\n")


class PointCloudProtocol:
    """Type alias used by :mod:`typing` for the public API."""

    points: np.ndarray
    colors: np.ndarray
    intensities: np.ndarray

    def estimate_normals(self, radius: float, max_nn: int) -> None: ...
    def voxel_down_sample(self, voxel_size: float) -> "PointCloudProtocol": ...
    def radius_outlier_removal(self, radius: float, min_neighbours: int) -> "PointCloudProtocol": ...
    def translate(self, vector: Iterable[float]) -> None: ...
    def scale(self, factor: float) -> None: ...
    def rotate(self, rotation_matrix: np.ndarray) -> None: ...


__all__ = [
    "PointCloudProtocol",
    "create_point_cloud",
    "estimate_normals",
    "voxel_down_sample",
    "radius_outlier_removal",
    "write_ply",
    "apply_colormap",
]

