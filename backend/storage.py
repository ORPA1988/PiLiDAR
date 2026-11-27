"""Datenablage: ein Ordner pro Scan mit Rohdaten, Punktwolken und Metadaten.

Struktur:
  scans/<id>/
    meta.json
    raw/        lidar_raw.npz (Standard; optional lidar_raw.bin, frames.jsonl)
    pointcloud/ <id>.ply (optional <id>.xyz)
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


_BIN_DTYPE = np.dtype([("z", "<f4"), ("a", "<f4", 12), ("d", "<u2", 12), ("i", "u1", 12)])


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
    def save_raw_frames(self, scan_id: str, frames, formats=("npy",)) -> None:
        """Rohdaten speichern. frames: Liste von dicts oder dict von Arrays
        (pointcloud.frames_to_arrays). Standard nur lidar_raw.npz (~10 MB je Scan):
        z_angle, angles, distances, intensities + speed_dps, timestamp_ms, t.
        "jsonl"/"bin" sind optional (groß und langsam: 80 MB / 9 s auf dem Pi)."""
        from .pointcloud import frames_to_arrays
        d = self.scan_dir(scan_id) / "raw"
        arr = frames_to_arrays(frames)
        if "npy" in formats or "npz" in formats:
            data = {"z_angle": np.asarray(arr["z_angle"], dtype=np.float32),
                    "angles": np.asarray(arr["angles"], dtype=np.float32),
                    "distances": np.asarray(arr["distances"], dtype=np.uint16),
                    "intensities": np.asarray(arr["intensities"], dtype=np.uint8)}
            for k in ("speed_dps", "timestamp_ms", "t"):
                if k in arr:
                    data[k] = arr[k]
            np.savez_compressed(d / "lidar_raw.npz", **data)
        if "jsonl" in formats:
            with open(d / "frames.jsonl", "w", encoding="utf-8") as fh:
                for i in range(len(arr["z_angle"])):
                    fh.write(json.dumps({"z_angle": float(arr["z_angle"][i]),
                                         "angles": [float(x) for x in arr["angles"][i]],
                                         "distances": [int(x) for x in arr["distances"][i]],
                                         "intensities": [int(x) for x in arr["intensities"][i]]},
                                        separators=(",", ":")) + "\n")
        if "bin" in formats:
            # je Frame: float32 z_angle, 12x float32 angle, 12x uint16 dist, 12x uint8 int
            n = len(arr["z_angle"])
            rec = np.empty(n, dtype=_BIN_DTYPE)
            rec["z"] = arr["z_angle"]; rec["a"] = arr["angles"]
            rec["d"] = arr["distances"]; rec["i"] = arr["intensities"]
            rec.tofile(d / "lidar_raw.bin")

    def load_raw_arrays(self, scan_id: str):
        """Rohdaten (dict von Arrays) laden oder None."""
        d = self.scan_dir(scan_id) / "raw"
        npz = d / "lidar_raw.npz"
        if npz.exists():
            with np.load(npz) as z:
                return {k: z[k] for k in z.files}
        binf = d / "lidar_raw.bin"
        if binf.exists():
            rec = np.fromfile(binf, dtype=_BIN_DTYPE)
            return {"z_angle": rec["z"], "angles": rec["a"], "distances": rec["d"], "intensities": rec["i"]}
        return None

    def load_raw_packed(self, scan_id: str) -> bytes | None:
        """Rohdaten als Binärpuffer für den Browser (der Worker rechnet die
        Punktwolke mit den AKTUELLEN Offsets). Layout little-endian:
          uint32 n | float32 z[n] | float32 angles[n*12] | uint16 dist[n*12] | uint8 int[n*12]"""
        arr = self.load_raw_arrays(scan_id)
        if arr is None:
            return None
        n = len(arr["z_angle"])
        return (np.asarray([n], dtype="<u4").tobytes()
                + np.ascontiguousarray(arr["z_angle"], dtype="<f4").tobytes()
                + np.ascontiguousarray(arr["angles"], dtype="<f4").tobytes()
                + np.ascontiguousarray(arr["distances"], dtype="<u2").tobytes()
                + np.ascontiguousarray(arr["intensities"], dtype=np.uint8).tobytes())

    # ------------------------------------------------------------------
    def save_pointcloud(self, scan_id: str, points_mm, intensities, formats=("ply",)) -> None:
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
    def load_pointcloud_arrays(self, scan_id: str):
        """Punktwolke als numpy-Arrays laden: (xyz Nx3 float32, inten N uint8).

        Liest bevorzugt die kompakte PLY-Datei (mit Intensität), fällt sonst auf
        die XYZ-Textdatei zurück. Gibt None zurück, wenn nichts vorhanden ist.
        """
        d = self.scan_dir(scan_id) / "pointcloud"
        ply = d / f"{scan_id}.ply"
        if ply.exists():
            return self._read_ply_arrays(ply)
        xyz = d / f"{scan_id}.xyz"
        if xyz.exists():
            pts = np.atleast_2d(np.loadtxt(xyz, dtype=np.float32))
            inten = np.full(len(pts), 180, dtype=np.uint8)
            return pts[:, :3].astype(np.float32), inten
        return None

    def load_pointcloud_packed(self, scan_id: str) -> bytes:
        """Kompakte Binärdarstellung für die 3D-Anzeige (klein & schnell zu parsen).

        Layout (little-endian):  uint32 n | float32[n*3] xyz [m] | uint8[n] intensität.
        Ersetzt die früher riesige JSON-Liste (bei Millionen Punkten unbrauchbar).
        Leerer Buffer (n=0), falls keine Punktwolke existiert.
        """
        data = self.load_pointcloud_arrays(scan_id)
        if data is None:
            return np.zeros(1, dtype="<u4").tobytes()
        xyz, inten = data
        n = len(xyz)
        return (np.asarray([n], dtype="<u4").tobytes()
                + np.ascontiguousarray(xyz, dtype="<f4").tobytes()
                + np.ascontiguousarray(inten, dtype=np.uint8).tobytes())

    @staticmethod
    def _read_ply_arrays(path: Path):
        """Binäres PLY (siehe _write_ply) in numpy-Arrays lesen."""
        with open(path, "rb") as fh:
            raw = fh.read()
        end = raw.find(b"end_header\n")
        if end < 0:
            return np.zeros((0, 3), np.float32), np.zeros(0, np.uint8)
        header = raw[:end].decode("ascii", "ignore")
        n = 0
        for line in header.splitlines():
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
        body = raw[end + len(b"end_header\n"):]
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                          ("r", "u1"), ("g", "u1"), ("b", "u1")])
        arr = np.frombuffer(body, dtype=dtype, count=n)
        xyz = np.empty((n, 3), dtype=np.float32)
        xyz[:, 0] = arr["x"]; xyz[:, 1] = arr["y"]; xyz[:, 2] = arr["z"]
        return xyz, arr["r"].copy()

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
