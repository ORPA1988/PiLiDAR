"""Datenablage: ein Ordner pro Scan mit Rohdaten, Punktwolken und Metadaten.

Struktur:
  scans/<id>/
    meta.json
    raw/        lidar_raw.bin (+ .npy), frames.jsonl
    pointcloud/ <id>.ply, <id>.xyz, ...
    images/     (später Kamera)
    sensors/    (später IMU)
    log.txt
"""

from __future__ import annotations

import io
import json
import zipfile
from pathlib import Path

import numpy as np


class ScanStore:
    def __init__(self, base_dir: Path | str):
        self.base = Path(base_dir)
        self.base.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    def list_scans(self) -> list[dict]:
        out = []
        for d in sorted(self.base.iterdir(), reverse=True):
            if not d.is_dir():
                continue
            meta = d / "meta.json"
            info = {"id": d.name}
            if meta.exists():
                try:
                    info.update(json.loads(meta.read_text(encoding="utf-8")))
                except Exception:
                    pass
            out.append(info)
        return out

    def scan_dir(self, scan_id: str) -> Path:
        return self.base / scan_id

    def create(self, scan_id: str) -> Path:
        d = self.scan_dir(scan_id)
        for sub in ("raw", "pointcloud", "images", "sensors"):
            (d / sub).mkdir(parents=True, exist_ok=True)
        return d

    # ------------------------------------------------------------------
    def write_meta(self, scan_id: str, meta: dict) -> None:
        d = self.scan_dir(scan_id)
        (d / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False),
                                     encoding="utf-8")

    def append_log(self, scan_id: str, line: str) -> None:
        with open(self.scan_dir(scan_id) / "log.txt", "a", encoding="utf-8") as fh:
            fh.write(line.rstrip() + "\n")

    # ------------------------------------------------------------------
    def save_raw_frames(self, scan_id: str, frames: list[dict], formats=("bin", "npy")) -> None:
        d = self.scan_dir(scan_id) / "raw"
        # JSONL ist immer hilfreich für Nachvollziehbarkeit
        with open(d / "frames.jsonl", "w", encoding="utf-8") as fh:
            for fr in frames:
                fh.write(json.dumps(fr, separators=(",", ":")) + "\n")
        if "npy" in formats:
            # kompaktes strukturiertes Array: z_angle + 12*(dist,intensity)
            n = len(frames)
            z = np.array([fr["z_angle"] for fr in frames], dtype=np.float32)
            dist = np.array([fr["distances"] for fr in frames], dtype=np.uint16)
            ang = np.array([fr["angles"] for fr in frames], dtype=np.float32)
            inten = np.array([fr["intensities"] for fr in frames], dtype=np.uint8)
            np.savez_compressed(d / "lidar_raw.npz", z_angle=z, angles=ang,
                                distances=dist, intensities=inten)
        if "bin" in formats:
            # rohe Aneinanderreihung: float32 z_angle, dann 12x(float32 angle, uint16 dist, uint8 int)
            with open(d / "lidar_raw.bin", "wb") as fh:
                for fr in frames:
                    fh.write(np.float32(fr["z_angle"]).tobytes())
                    fh.write(np.asarray(fr["angles"], dtype=np.float32).tobytes())
                    fh.write(np.asarray(fr["distances"], dtype=np.uint16).tobytes())
                    fh.write(np.asarray(fr["intensities"], dtype=np.uint8).tobytes())

    # ------------------------------------------------------------------
    def save_pointcloud(self, scan_id: str, points_mm, intensities, formats=("ply", "xyz")) -> None:
        d = self.scan_dir(scan_id) / "pointcloud"
        pts_m = np.asarray(points_mm, dtype=float) / 1000.0  # mm -> m
        inten = np.asarray(intensities, dtype=float)
        if "xyz" in formats:
            np.savetxt(d / f"{scan_id}.xyz", pts_m, fmt="%.4f")
        if "ply" in formats:
            self._write_ply(d / f"{scan_id}.ply", pts_m, inten)

    @staticmethod
    def _write_ply(path: Path, pts: np.ndarray, inten: np.ndarray) -> None:
        """Binäres PLY (little-endian) – kompakt und schnell auch bei Millionen Punkten."""
        n = len(pts)
        c = np.clip(inten, 0, 255).astype(np.uint8) if len(inten) == n else np.zeros(n, np.uint8)
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                          ("r", "u1"), ("g", "u1"), ("b", "u1")])
        arr = np.empty(n, dtype=dtype)
        arr["x"] = pts[:, 0]; arr["y"] = pts[:, 1]; arr["z"] = pts[:, 2]
        arr["r"] = c; arr["g"] = c; arr["b"] = c
        header = (
            "ply\nformat binary_little_endian 1.0\n"
            f"element vertex {n}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n"
        )
        with open(path, "wb") as fh:
            fh.write(header.encode("ascii"))
            fh.write(arr.tobytes())

    # ------------------------------------------------------------------
    def delete_scan(self, scan_id: str) -> None:
        import shutil
        d = self.scan_dir(scan_id)
        if d.exists() and d.parent.resolve() == self.base.resolve():
            shutil.rmtree(d)

    def update_annotation(self, scan_id: str, text: str) -> None:
        meta_path = self.scan_dir(scan_id) / "meta.json"
        if meta_path.exists():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        else:
            meta = {"id": scan_id}
        meta["annotation"] = text
        meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    # ------------------------------------------------------------------
    def zip_bytes(self, scan_id: str) -> bytes:
        d = self.scan_dir(scan_id)
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
            for p in d.rglob("*"):
                if p.is_file():
                    zf.write(p, p.relative_to(self.base))
        return buf.getvalue()
