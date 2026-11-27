"""Kompakte Aufzeichnung der LiDAR-Frames während eines Laufs.

Früher wurde je Frame ein dict mit drei Listen angehängt (~200 000 Frames je Scan, über eine
Million Python-Objekte). Die zyklische Speicherbereinigung von Python durchläuft diese Objekte
regelmäßig und hielt dabei den Lese-Thread zunehmend lange an (gemessen 120, 300, 480 ms je
Scan) – der serielle Puffer lief über, im Scan fehlten Streifen. Hier landen die Werte direkt
in vorab angelegten numpy-Blöcken: kaum Objekte, keine Pausen.
"""

from __future__ import annotations

import threading

import numpy as np

_CHUNK = 20000          # Frames je Block (~1,8 MB), ein Scan braucht ~10 Blöcke
_PTS = 12


class FrameRecorder:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.clear()

    def clear(self) -> None:
        with getattr(self, "_lock", threading.Lock()):
            self._blocks: list[dict] = []
            self._n_in_block = _CHUNK   # erzwingt neuen Block beim ersten append
            self._total = 0

    def _new_block(self) -> None:
        self._blocks.append({
            "z_angle": np.empty(_CHUNK, np.float64),
            "angles": np.empty((_CHUNK, _PTS), np.float32),
            "distances": np.empty((_CHUNK, _PTS), np.uint16),
            "intensities": np.empty((_CHUNK, _PTS), np.uint8),
            "speed_dps": np.empty(_CHUNK, np.float32),
            "timestamp_ms": np.empty(_CHUNK, np.uint16),
            "t": np.empty(_CHUNK, np.float64),
        })
        self._n_in_block = 0

    def append(self, z_angle, angles, distances, intensities, speed_dps, timestamp_ms, t) -> None:
        with self._lock:
            if self._n_in_block >= _CHUNK:
                self._new_block()
            b, i = self._blocks[-1], self._n_in_block
            b["z_angle"][i] = z_angle
            b["angles"][i] = angles
            b["distances"][i] = distances
            b["intensities"][i] = intensities
            b["speed_dps"][i] = speed_dps
            b["timestamp_ms"][i] = timestamp_ms
            b["t"][i] = t
            self._n_in_block += 1
            self._total += 1

    def __len__(self) -> int:
        return self._total

    def arrays(self) -> dict:
        """Alle Frames als dict von Arrays (Kopie; Format wie pointcloud.frames_to_arrays)."""
        with self._lock:
            if not self._blocks:
                return {"z_angle": np.zeros(0), "angles": np.zeros((0, _PTS), np.float32),
                        "distances": np.zeros((0, _PTS), np.uint16),
                        "intensities": np.zeros((0, _PTS), np.uint8),
                        "speed_dps": np.zeros(0, np.float32), "timestamp_ms": np.zeros(0, np.uint16),
                        "t": np.zeros(0)}
            parts = [{k: v[:_CHUNK] for k, v in b.items()} for b in self._blocks[:-1]]
            parts.append({k: v[:self._n_in_block] for k, v in self._blocks[-1].items()})
            return {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
