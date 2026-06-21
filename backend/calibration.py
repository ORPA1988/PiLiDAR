"""Kalibrierung: Rotation (Schritte/Grad), Achsversatz + Winkel-Offset, Δ-Überlappung.

Alle Verfahren nutzen ausschließlich die LiDAR-Eigendaten eines Kalibrierscans –
kein Spezialwerkzeug nötig. Ergebnis kann in config.json übernommen werden.
"""

from __future__ import annotations

import numpy as np

from .pointcloud import build_pointcloud

try:
    from scipy.optimize import minimize
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover
    minimize = None
    cKDTree = None


def _profile(frames, n_bins=720):
    """Mittlere Distanz je Winkel-Bin über alle Frames – robustes 1D-Profil."""
    acc = np.zeros(n_bins)
    cnt = np.zeros(n_bins)
    for fr in frames:
        a = np.asarray(fr["angles"]) % 360.0
        d = np.asarray(fr["distances"], dtype=float)
        idx = (a / 360.0 * n_bins).astype(int) % n_bins
        for j, dd in zip(idx, d):
            if dd > 0:
                acc[j] += dd
                cnt[j] += 1
    prof = np.where(cnt > 0, acc / np.maximum(cnt, 1), 0.0)
    return prof


def calibrate_rotation(frames_360, nominal_gear_ratio: float) -> dict:
    """Schätzt das echte Getriebeverhältnis aus einem vollen 360°-Lauf.

    Idee: Bei korrekter Skalierung muss der bei z≈360° gemessene 2D-Querschnitt
    wieder dem bei z≈0° entsprechen. Wir korrelieren beide Profile und leiten
    daraus einen Skalierungsfaktor für die kommandierten gegen die realen Grad ab.
    """
    if not frames_360:
        return {"ok": False, "reason": "keine Daten"}
    z = np.array([fr["z_angle"] for fr in frames_360], dtype=float)
    zmax = float(z.max())
    if zmax < 300:
        return {"ok": False, "reason": "Kalibrierscan deckt keine ~360° ab"}
    early = [fr for fr, zz in zip(frames_360, z) if zz <= 10]
    late = [fr for fr, zz in zip(frames_360, z) if zz >= zmax - 10]
    if len(early) < 5 or len(late) < 5:
        return {"ok": False, "reason": "zu wenige Frames in Überlappung"}
    p0 = _profile(early)
    p1 = _profile(late)
    # zirkuläre Kreuzkorrelation -> Winkelversatz in Bins
    corr = np.fft.ifft(np.fft.fft(p0) * np.conj(np.fft.fft(p1))).real
    shift_bins = int(np.argmax(corr))
    n = len(p0)
    if shift_bins > n / 2:
        shift_bins -= n
    shift_deg = shift_bins / n * 360.0
    # realer Drehweg = kommandiert (zmax) + Restversatz; Skalierung des Getriebes
    real_travel = zmax - shift_deg
    scale = real_travel / zmax if zmax else 1.0
    return {
        "ok": True,
        "z_commanded_deg": zmax,
        "shift_deg": shift_deg,
        "scale": scale,
        "gear_ratio": nominal_gear_ratio * scale,
    }


def calibrate_offset(frames, angle_offset0, position_offset0, dist_min, dist_max,
                     scan_angle, overlap_deg) -> dict:
    """Optimiert Achsversatz (Y,Z) + angle_offset so, dass die in der
    Überlappungszone doppelt erfassten Flächen zusammenfallen."""
    if minimize is None or cKDTree is None:
        return {"ok": False, "reason": "scipy nicht verfügbar"}
    z = np.array([fr["z_angle"] for fr in frames], dtype=float)
    overlap_lo = scan_angle - overlap_deg
    a_sel = [fr for fr, zz in zip(frames, z) if 0 <= zz <= overlap_deg]
    b_sel = [fr for fr, zz in zip(frames, z) if overlap_lo <= zz <= scan_angle]
    if len(a_sel) < 5 or len(b_sel) < 5:
        return {"ok": False, "reason": "keine ausreichende Überlappung"}

    def residual(params):
        ao, oy, oz = params
        pos = (0.0, oy, oz)
        pa, _ = build_pointcloud(a_sel, ao, pos, dist_min, dist_max)
        pb, _ = build_pointcloud(b_sel, ao, pos, dist_min, dist_max)
        if len(pa) < 5 or len(pb) < 5:
            return 1e9
        if len(pa) > 2000:
            pa = pa[np.random.choice(len(pa), 2000, replace=False)]
        tree = cKDTree(pb)
        d, _ = tree.query(pa, k=1)
        return float(np.median(d))

    x0 = [angle_offset0, position_offset0[1], position_offset0[2]]
    res = minimize(residual, x0, method="Nelder-Mead",
                   options={"xatol": 0.05, "fatol": 0.5, "maxiter": 200})
    return {
        "ok": bool(res.success),
        "angle_offset": float(res.x[0]),
        "model_y_offset": float(res.x[1]),
        "model_z_offset": float(res.x[2]),
        "residual_mm": float(res.fun),
    }
