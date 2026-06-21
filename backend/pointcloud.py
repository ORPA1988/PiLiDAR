"""Server-seitige Referenz der Punktwolken-Berechnung.

Identisch zur Client-Implementierung (frontend/scan.worker.js). Wird für den
optionalen Server-Export (PLY/XYZ/E57) und für die Kalibrierung/QA genutzt.

Geometrie (wie im Original-PiLiDAR verifiziert):
  * Der 2D-LiDAR-Scan liegt in der vertikalen X-Z-Ebene.
  * angle_offset kippt die Ebene um die Y-Achse (mechanische Korrektur).
  * position_offset verschiebt den Sensor relativ zur Drehachse; danach wird die
    Ebene um die senkrechte Z-Achse um -z_angle revolviert.
"""

from __future__ import annotations

import numpy as np


def polar_to_plane(angles_deg: np.ndarray, distances_mm: np.ndarray) -> np.ndarray:
    """Polarwerte -> Punkte (x, y, z) in der vertikalen Scan-Ebene (y=0)."""
    a = np.radians(angles_deg)
    x = distances_mm * np.cos(a)
    z = distances_mm * np.sin(a)
    y = np.zeros_like(x)
    return np.column_stack((x, y, z))


def _rot_matrix(axis, deg) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) or 1.0)
    th = np.radians(deg)
    c, s = np.cos(th), np.sin(th)
    x, y, zc = axis
    return np.array([
        [c + x * x * (1 - c),     x * y * (1 - c) - zc * s, x * zc * (1 - c) + y * s],
        [y * x * (1 - c) + zc * s, c + y * y * (1 - c),     y * zc * (1 - c) - x * s],
        [zc * x * (1 - c) - y * s, zc * y * (1 - c) + x * s, c + zc * zc * (1 - c)],
    ])


def transform_frame(points_plane: np.ndarray, z_angle: float, angle_offset: float,
                    position_offset=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Eine Scan-Ebene in den 3D-Raum überführen."""
    pts = points_plane @ _rot_matrix((0, 1, 0), angle_offset).T
    pts = pts + np.asarray(position_offset, dtype=float)
    pts = pts @ _rot_matrix((0, 0, 1), -z_angle).T
    return pts


def build_pointcloud(frames, angle_offset: float, position_offset,
                     dist_min_mm: float, dist_max_mm: float):
    """frames: Liste von dicts mit angles[], distances[], intensities[], z_angle.

    Liefert (points Nx3 [mm], intensities N)."""
    all_pts = []
    all_int = []
    for fr in frames:
        ang = np.asarray(fr["angles"], dtype=float)
        dist = np.asarray(fr["distances"], dtype=float)
        inten = np.asarray(fr["intensities"], dtype=float)
        mask = (dist >= dist_min_mm) & (dist <= dist_max_mm)
        if not np.any(mask):
            continue
        plane = polar_to_plane(ang[mask], dist[mask])
        pts3d = transform_frame(plane, fr["z_angle"], angle_offset, position_offset)
        all_pts.append(pts3d)
        all_int.append(inten[mask])
    if not all_pts:
        return np.zeros((0, 3)), np.zeros((0,))
    return np.vstack(all_pts), np.concatenate(all_int)
