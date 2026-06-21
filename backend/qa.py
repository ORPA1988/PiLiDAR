"""Fehlererkennung & Qualitätssicherung pro Scan.

Liefert eine Ampel (ok / warn / error) plus Einzelmetriken, die in meta.json
gespeichert und in der UI angezeigt werden.
"""

from __future__ import annotations

import numpy as np

from .pointcloud import build_pointcloud

try:
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover
    cKDTree = None


def rpm_stability(frames: list[dict]) -> dict:
    if not frames:
        return {"mean_dps": 0.0, "cv": 0.0}
    sp = np.array([fr.get("speed_dps", 0.0) for fr in frames], dtype=float)
    sp = sp[sp > 0]
    if sp.size == 0:
        return {"mean_dps": 0.0, "cv": 0.0}
    mean = float(sp.mean())
    cv = float(sp.std() / mean) if mean else 0.0
    return {"mean_dps": mean, "cv": cv, "rpm": mean / 6.0}


def loop_closure_residual(frames, angle_offset, position_offset, dist_min, dist_max,
                          scan_angle, overlap_deg, max_pts=4000) -> float | None:
    """Mittlerer Nächster-Nachbar-Abstand [mm] zwischen Anfangs- und
    Überlappungs-Sektor. Großer Wert ⇒ Schrittverlust/Schlupf/Fehlkalibrierung."""
    if cKDTree is None or not frames:
        return None
    z = np.array([fr["z_angle"] for fr in frames], dtype=float)
    start_sel = [fr for fr, zz in zip(frames, z) if 0.0 <= zz <= overlap_deg]
    # Überlappung am Ende: derselbe Raumbereich, ~180° weitergedreht
    end_lo, end_hi = scan_angle - overlap_deg, scan_angle
    end_sel = [fr for fr, zz in zip(frames, z) if end_lo <= zz <= end_hi]
    if len(start_sel) < 5 or len(end_sel) < 5:
        return None
    p_start, _ = build_pointcloud(start_sel, angle_offset, position_offset, dist_min, dist_max)
    p_end, _ = build_pointcloud(end_sel, angle_offset, position_offset, dist_min, dist_max)
    if len(p_start) < 5 or len(p_end) < 5:
        return None
    if len(p_start) > max_pts:
        p_start = p_start[np.random.choice(len(p_start), max_pts, replace=False)]
    tree = cKDTree(p_end)
    d, _ = tree.query(p_start, k=1)
    return float(np.median(d))


def compute_qa(frames, lidar_stats: dict, geom: dict, lidar_cfg: dict,
               scan_angle: float, overlap_deg: float) -> dict:
    pos = (0.0, geom.get("MODEL_Y_OFFSET", 0.0), geom.get("MODEL_Z_OFFSET", 0.0))
    qa = {
        "n_frames": len(frames),
        "n_points_raw": len(frames) * 12,
        "crc_error_rate": lidar_stats.get("crc_error_rate", 0.0),
        "packet_rate_hz": lidar_stats.get("packet_rate", 0.0),
        "rpm": rpm_stability(frames),
    }
    qa["loop_closure_mm"] = loop_closure_residual(
        frames,
        lidar_cfg.get("ANGLE_OFFSET", 0.0),
        pos,
        lidar_cfg.get("DISTANCE_MIN_MM", 30),
        lidar_cfg.get("DISTANCE_MAX_MM", 25000),
        scan_angle,
        overlap_deg,
    )

    # Ampel
    flags = []
    if qa["crc_error_rate"] > 0.02:
        flags.append(("error", "Hohe CRC-Fehlerrate (Verkabelung/Baudrate/EMV)"))
    elif qa["crc_error_rate"] > 0.005:
        flags.append(("warn", "Erhöhte CRC-Fehlerrate"))
    if qa["rpm"]["cv"] > 0.1:
        flags.append(("warn", "Instabile LiDAR-Drehzahl"))
    lc = qa["loop_closure_mm"]
    if lc is not None:
        if lc > 50:
            flags.append(("error", f"Großer Loop-Closure-Versatz ({lc:.0f} mm): "
                                    "Schrittverlust/Vibration/Fehlkalibrierung"))
        elif lc > 20:
            flags.append(("warn", f"Erhöhter Loop-Closure-Versatz ({lc:.0f} mm)"))

    status = "ok"
    if any(f[0] == "warn" for f in flags):
        status = "warn"
    if any(f[0] == "error" for f in flags):
        status = "error"
    qa["status"] = status
    qa["messages"] = [m for _, m in flags]
    return qa
