"""Kalibrierung: Rotation (Schritte/Grad), Achsversatz + Winkel-Offset, Δ-Überlappung.

Alle Verfahren nutzen ausschließlich die LiDAR-Eigendaten eines Kalibrierscans –
kein Spezialwerkzeug nötig. Ergebnis kann in config.json übernommen werden.
"""

from __future__ import annotations

import numpy as np

from .pointcloud import build_pointcloud, frames_to_arrays, select_frames

try:
    from scipy.optimize import minimize
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover
    minimize = None
    cKDTree = None


def _profile(frames, n_bins=720):
    """Mittlere Distanz je Winkel-Bin über alle Frames – robustes 1D-Profil."""
    arr = frames_to_arrays(frames)
    a = np.asarray(arr["angles"], dtype=float).ravel() % 360.0
    d = np.asarray(arr["distances"], dtype=float).ravel()
    m = d > 0
    idx = (a[m] / 360.0 * n_bins).astype(int) % n_bins
    acc = np.bincount(idx, weights=d[m], minlength=n_bins)
    cnt = np.bincount(idx, minlength=n_bins)
    return np.where(cnt > 0, acc / np.maximum(cnt, 1), 0.0)


def calibrate_rotation(frames_360, nominal_gear_ratio: float) -> dict:
    """Schätzt das echte Getriebeverhältnis aus einem vollen 360°-Lauf.

    Idee: Bei korrekter Skalierung muss der bei z≈360° gemessene 2D-Querschnitt
    wieder dem bei z≈0° entsprechen. Wir korrelieren beide Profile und leiten
    daraus einen Skalierungsfaktor für die kommandierten gegen die realen Grad ab.
    """
    arr = frames_to_arrays(frames_360)
    z = np.asarray(arr["z_angle"], dtype=float)
    if z.size == 0:
        return {"ok": False, "reason": "keine Daten"}
    zmax = float(z.max())
    if zmax < 300:
        return {"ok": False, "reason": "Kalibrierscan deckt keine ~360° ab"}
    early = select_frames(arr, -1e9, 10)
    late = select_frames(arr, zmax - 10, 1e9)
    if len(early["z_angle"]) < 5 or len(late["z_angle"]) < 5:
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


class CalibrationAborted(Exception):
    """Kalibrierung vom Benutzer abgebrochen."""


def calibrate_offset(frames, angle_offset0, position_offset0, dist_min, dist_max,
                     scan_angle, overlap_deg, should_abort=None) -> dict:
    """Optimiert angle_offset so, dass die in der Überlappungszone doppelt erfassten
    Flächen zusammenfallen. Achsversatz y und Höhe z sind so nicht bestimmbar (siehe
    qa.compute_qa) und werden unverändert zurückgegeben – sie kommen aus der Mechanik."""
    if minimize is None or cKDTree is None:
        return {"ok": False, "reason": "scipy nicht verfügbar"}
    arr = frames_to_arrays(frames)
    a_sel = select_frames(arr, 0.0, overlap_deg)
    b_sel = select_frames(arr, scan_angle - overlap_deg, scan_angle)
    if len(a_sel["z_angle"]) < 5 or len(b_sel["z_angle"]) < 5:
        return {"ok": False, "reason": "keine ausreichende Überlappung"}

    def residual(params):
        if should_abort is not None and should_abort():
            raise CalibrationAborted()
        ao = params[0]
        pos = (0.0, 0.0, position_offset0[2])            # y = 0 für den Vergleich, siehe Docstring
        pa, _ = build_pointcloud(a_sel, ao, pos, dist_min, dist_max)
        pb, _ = build_pointcloud(b_sel, ao, pos, dist_min, dist_max)
        if len(pa) < 5 or len(pb) < 5:
            return 1e9
        if len(pa) > 2000:  # feste Auswahl -> glatte Zielfunktion für Nelder-Mead
            pa = pa[np.random.default_rng(0).choice(len(pa), 2000, replace=False)]
        tree = cKDTree(pb)
        d, _ = tree.query(pa, k=1)
        return float(np.median(d))

    x0 = [angle_offset0]
    try:
        res = minimize(residual, x0, method="Nelder-Mead",
                       options={"xatol": 0.05, "fatol": 0.5, "maxiter": 200})
    except CalibrationAborted:
        return {"ok": False, "reason": "abgebrochen"}
    return {
        "ok": bool(res.success),
        "angle_offset": float(res.x[0]),
        "model_y_offset": float(position_offset0[1]),
        "model_z_offset": float(position_offset0[2]),
        "residual_mm": float(res.fun),
    }
