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
    z = -distances_mm * np.sin(a)  # 270°→+Z(oben)
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


def frames_to_arrays(frames) -> dict:
    """Frame-Liste (dicts) oder bereits fertige Arrays -> dict mit numpy-Arrays
    z_angle (N), angles (N,12), distances (N,12), intensities (N,12)."""
    if isinstance(frames, dict):
        return frames
    if not frames:
        e = np.zeros((0, 12))
        return {"z_angle": np.zeros(0), "angles": e, "distances": e, "intensities": e}
    n = len(frames)
    out = {
        "z_angle": np.fromiter((fr["z_angle"] for fr in frames), dtype=np.float64, count=n),
        "angles": np.asarray([fr["angles"] for fr in frames], dtype=np.float32),
        "distances": np.asarray([fr["distances"] for fr in frames], dtype=np.uint16),
        "intensities": np.asarray([fr["intensities"] for fr in frames], dtype=np.uint8),
    }
    # optionale Zusatzfelder (Drehzahl, LiDAR-Zeitstempel, Empfangszeit)
    for key, dt in (("speed_dps", np.float32), ("timestamp_ms", np.uint16), ("t", np.float64)):
        if key in frames[0]:
            out[key] = np.fromiter((fr.get(key, 0) for fr in frames), dtype=dt, count=n)
    return out


def select_frames(arrays: dict, z_lo: float, z_hi: float) -> dict:
    """Teilmenge der Frames mit z_lo <= z_angle <= z_hi."""
    m = (arrays["z_angle"] >= z_lo) & (arrays["z_angle"] <= z_hi)
    return {k: np.asarray(v)[m] for k, v in arrays.items()
            if k in ("z_angle", "angles", "distances", "intensities")}


def build_pointcloud(frames, angle_offset: float, position_offset,
                     dist_min_mm: float, dist_max_mm: float, z_scale: float = 1.0,
                     half_yaw: float = 0.0):
    """frames: Liste von dicts (angles[], distances[], intensities[], z_angle)
    oder dict von Arrays (siehe frames_to_arrays).

    Vollständig vektorisiert (früher Python-Schleife je Frame: 67 s statt 1,6 s
    für 200 000 Frames auf dem Pi 4). Rechnung identisch zu transform_frame.
    Liefert (points Nx3 [mm], intensities N)."""
    arr = frames_to_arrays(frames)
    ang = np.asarray(arr["angles"], dtype=np.float64)
    if ang.size == 0:
        return np.zeros((0, 3)), np.zeros((0,))
    per = ang.shape[1] if ang.ndim == 2 else 1
    ang = ang.ravel()
    dist = np.asarray(arr["distances"], dtype=np.float64).ravel()
    inten = np.asarray(arr["intensities"], dtype=np.float64).ravel()
    zz = np.repeat(np.asarray(arr["z_angle"], dtype=np.float64), per)
    m = (dist >= dist_min_mm) & (dist <= dist_max_mm)
    if not np.any(m):
        return np.zeros((0, 3)), np.zeros((0,))
    pts = polar_to_plane(ang[m], dist[m]) @ _rot_matrix((0, 1, 0), angle_offset).T
    # Hälften-Verdrehung (wie FAN_SPLIT im Original-PiLiDAR): die beiden radialen Hälften des
    # LiDAR-Fächers (x > 0 / x < 0 in der Scan-Ebene) sind um einen festen Winkel um die Drehachse
    # gegeneinander verdreht (Geräteeigenschaft). Vordere Hälfte +h/2, hintere −h/2.
    half = np.where(pts[:, 0] > 0, 0.5, -0.5) * float(half_yaw)
    pts = pts + np.asarray(position_offset, dtype=float)
    th = np.radians(-zz[m] * float(z_scale) + half)   # z_scale: Drehfaktor
    c, s_ = np.cos(th), np.sin(th)
    out = np.column_stack((pts[:, 0] * c - pts[:, 1] * s_, pts[:, 0] * s_ + pts[:, 1] * c, pts[:, 2]))
    return out, inten[m]
