"""Fehlererkennung & Qualitätssicherung pro Scan.

Liefert eine Ampel (ok / warn / error) plus Einzelmetriken, die in meta.json
gespeichert und in der UI angezeigt werden.
"""

from __future__ import annotations

import numpy as np

from .pointcloud import build_pointcloud, frames_to_arrays, select_frames

try:
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover
    cKDTree = None


def rpm_stability(frames) -> dict:
    arr = frames_to_arrays(frames)
    sp = np.asarray(arr.get("speed_dps", np.zeros(0)), dtype=float)
    if sp.size == 0:
        return {"mean_dps": 0.0, "cv": 0.0}
    sp = sp[sp > 0]
    if sp.size == 0:
        return {"mean_dps": 0.0, "cv": 0.0}
    mean = float(sp.mean())
    cv = float(sp.std() / mean) if mean else 0.0
    return {"mean_dps": mean, "cv": cv, "rpm": mean / 6.0}


def loop_closure_residual(frames, angle_offset, position_offset, dist_min, dist_max,
                          scan_angle, overlap_deg, max_pts=4000) -> float | None:
    """Median-Nächster-Nachbar-Abstand [mm] zwischen Anfangs- und
    Überlappungs-Sektor. Großer Wert ⇒ Schrittverlust/Schlupf/Fehlkalibrierung.

    Die Scan-Ebene enthält die Drehachse: nach 180° wird dieselbe Ebene erneut
    erfasst. Sinnvoll daher ab scan_angle >= 180° + Überlappung (Modus B: 184°)."""
    if cKDTree is None or frames is None or len(frames) == 0 or overlap_deg <= 0:
        return None
    if scan_angle < 180.0 + overlap_deg - 1e-6:
        return None  # kein räumlicher Überlapp
    arr = frames_to_arrays(frames)
    start_sel = select_frames(arr, 0.0, overlap_deg)
    # Überlappung am Ende: dieselbe Ebene, 180° weitergedreht
    end_sel = select_frames(arr, scan_angle - overlap_deg, scan_angle)
    if len(start_sel["z_angle"]) < 5 or len(end_sel["z_angle"]) < 5:
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
    # y (Abstand Scan-Ebene <-> Drehachse) bewusst 0: Bei y != 0 tasten Anfangs- und End-Sektor
    # physikalisch zwei parallele Ebenen im Abstand 2·y ab. Mit y im Modell lägen die Punkte dann
    # korrekt 2·y auseinander und der Punktabstand wäre immer >= 2·y. Mit y = 0 werden beide in
    # dieselbe Ebene gelegt; das verschiebt nur quer zur Ebene, die Geometrie IN der Ebene
    # (Winkel, Schrittverlust) bleibt vergleichbar. y selbst ist so nicht messbar -> von Hand.
    pos = (0.0, 0.0, geom.get("MODEL_Z_OFFSET", 0.0))
    frames = frames_to_arrays(frames)
    n = len(frames["z_angle"])
    qa = {
        "n_frames": n,
        "n_points_raw": n * 12,
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

    # Datenlücken: Pausen im LiDAR-Strom während des Laufs (Kopf dreht weiter -> Streifen ohne Punkte)
    t = np.asarray(frames.get("t", np.zeros(0)), dtype=float)
    gaps = []
    if t.size > 1:
        dt = np.diff(t)
        idx = np.nonzero(dt > 0.05)[0]
        z = np.asarray(frames["z_angle"], dtype=float)
        gaps = [{"z_deg": round(float(z[i]), 2), "ms": round(float(dt[i]) * 1000)} for i in idx[:20]]
    qa["data_gaps"] = gaps

    # Ampel
    flags = []
    if gaps:
        worst = max(g["ms"] for g in gaps)
        flags.append(("warn" if worst < 200 else "error",
                      f"{len(gaps)} Datenlücke(n) im LiDAR-Strom, längste {worst} ms (Streifen ohne Punkte)"))
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
