"""Prüft den Versatz y (Scan-Ebene <-> Drehachse) an einem gespeicherten Scan.

Anfangs- (0..ü°) und End-Sektor (Scanwinkel-ü..Scanwinkel) sehen dieselben Flächen.
Bei y != 0 liegen die beiden Durchgänge physikalisch in zwei parallelen Ebenen 2·y
auseinander. Ein Punkt-zu-Punkt-Vergleich (QA-Überlappung) bevorzugt deshalb immer y = 0.
Hier wird stattdessen der Abstand jedes Anfangspunkts zur lokalen FLÄCHE des End-Sektors
gemessen (Ebene aus den Nachbarn, PCA). Der ist bei richtigem y am kleinsten.

Aufruf (auf dem Pi, im Repo):  python3 scripts/offset_y_check.py <scan_id> [winkel]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from backend.config import Config  # noqa: E402
from backend.pointcloud import build_pointcloud, select_frames  # noqa: E402
from backend.storage import ScanStore  # noqa: E402


def plane_residual(p_a, p_b, radius=80.0, k=24):
    """Abstand [mm] jedes Punkts aus p_a zur lokalen Ebene von p_b (NaN, wenn dort keine
    flächige, ebene Nachbarschaft liegt)."""
    tree = cKDTree(p_b)
    dist, idx = tree.query(p_a, k=k, distance_upper_bound=radius)
    out = np.full(len(p_a), np.nan)
    for i in range(len(p_a)):
        ok = np.isfinite(dist[i])
        if ok.sum() < 8:
            continue
        nb = p_b[idx[i][ok]]
        c = nb.mean(axis=0)
        w, v = np.linalg.eigh(np.cov((nb - c).T))
        if w[1] < 4 * w[0] or w[1] < 25.0:     # linienförmig/zu klein -> keine Fläche
            continue
        out[i] = abs(float(np.dot(p_a[i] - c, v[:, 0])))
    return out


def main():
    scan_id = sys.argv[1]
    cfg = Config()
    lc = cfg.LIDAR
    ang = float(sys.argv[2]) if len(sys.argv) > 2 else lc["ANGLE_OFFSET"]
    st = ScanStore(cfg.STORAGE["BASE_DIR"])
    arr = st.load_raw_arrays(scan_id)
    meta = next((m for m in st.list_scans() if m["id"] == scan_id), {})
    sa = float(meta.get("scan_angle") or cfg.MODE_B_CONTINUOUS["SCAN_ANGLE"])
    ov = float(meta.get("overlap_deg") or cfg.MODE_B_CONTINUOUS["OVERLAP_DEG"])
    a_sel, b_sel = select_frames(arr, 0.0, ov), select_frames(arr, sa - ov, sa)
    print(f"Scan {scan_id}, Winkel {ang:.3f}°, Sektoren 0..{ov}° / {sa - ov}..{sa}°")
    rmin = float(sys.argv[3]) if len(sys.argv) > 3 else 1500.0
    ys = list(range(-80, 81, 10))
    # gleiche Anfangspunkte für alle y: Frame/Punkt-Auswahl hängt nicht von y ab -> Index stabil,
    # wenn build_pointcloud die Reihenfolge erhält; Distanzfilter auf den Rohwert
    res = {}
    for y in ys:
        pa, _ = build_pointcloud(a_sel, ang, (0.0, float(y), 0.0), max(lc["DISTANCE_MIN_MM"], rmin), 8000)
        pb, _ = build_pointcloud(b_sel, ang, (0.0, float(y), 0.0), lc["DISTANCE_MIN_MM"], 8000)
        pa = np.asarray(pa)
        if y == ys[0]:   # feste Stichprobe (Reihenfolge/Anzahl der Anfangspunkte hängt nicht von y ab)
            pick = np.random.default_rng(0).choice(len(pa), min(5000, len(pa)), replace=False)
        res[y] = plane_residual(pa[pick], np.asarray(pb))
    n = min(len(r) for r in res.values())
    common = np.ones(n, bool)
    for r in res.values():
        common &= np.isfinite(r[:n])
    print(f"Punkte ab {rmin:.0f} mm, gemeinsam auswertbar: {common.sum()} von {n}")
    print("   y[mm]   Median   Mittel(<=50)   90%")
    for y in ys:
        r = res[y][:n][common]
        if len(r) == 0:
            continue
        print(f"{y:8d} {np.median(r):8.2f} {np.mean(np.minimum(r, 50)):10.2f} {np.percentile(r, 90):8.1f}")


if __name__ == "__main__":
    main()
