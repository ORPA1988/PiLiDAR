"""Trockentests ohne Hardware: CRC8, Paket-Decode, Mock-Scan-Pipeline.

Aufruf:  python -m tests.test_core   (oder: pytest)
"""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from backend.hardware.lidar import HEADER, VERLEN, crc8, LidarReader


def build_packet(speed_dps=3600, fsa_deg=0.0, lsa_deg=22.0, dists=None, timestamp=1234):
    if dists is None:
        dists = [1000 + 10 * i for i in range(12)]
    body = bytearray()
    body.append(HEADER)
    body.append(VERLEN)
    body += struct.pack("<H", int(speed_dps))
    body += struct.pack("<H", int(fsa_deg * 100))
    for d in dists:
        body += struct.pack("<H", int(d))
        body.append(200)  # Intensität
    body += struct.pack("<H", int(lsa_deg * 100))
    body += struct.pack("<H", int(timestamp))
    body.append(crc8(bytes(body)))  # CRC über Byte [0:46]
    assert len(body) == 47, len(body)
    return bytes(body)


class MockSerial:
    """Liefert wiederholt eine Bytefolge, byteweise/blockweise wie pyserial.

    packet_delay drosselt auf eine realistische Paketrate (STL27L ~1800 Pkt/s)."""

    def __init__(self, stream: bytes, packet_delay: float = 0.0):
        self.stream = stream
        self.pos = 0
        self.packet_delay = packet_delay

    def read(self, n=1):
        out = bytearray()
        for _ in range(n):
            out.append(self.stream[self.pos % len(self.stream)])
            self.pos += 1
        if self.packet_delay and n > 2:
            time.sleep(self.packet_delay)
        return bytes(out)

    def close(self):
        pass

    @property
    def is_open(self):
        return True


def test_crc_roundtrip():
    pkt = build_packet()
    assert crc8(pkt[:46]) == pkt[46]
    print("OK  CRC8 roundtrip")


def test_decode():
    pkt = build_packet(fsa_deg=10.0, lsa_deg=32.0, dists=[2000] * 12)
    frames = []
    reader = LidarReader("mock", on_frame=frames.append, angle_offset=-1.05,
                         serial_factory=lambda: MockSerial(pkt))
    reader.start()
    time.sleep(0.2)
    reader.stop()
    assert frames, "keine Frames dekodiert"
    fr = frames[0]
    assert all(d == 2000 for d in fr.distances)
    # Roh-Winkel (ohne angle_offset, der wird erst in der Transformation angewandt)
    assert abs(fr.angles[0] - 10.0) < 0.01, fr.angles[0]
    assert reader.stats.packets_ok > 0
    assert reader.stats.crc_error_rate == 0.0
    print(f"OK  Decode: {len(frames)} Frames, speed={fr.speed_dps} dps, "
          f"angle0={fr.angles[0]:.2f}")


def test_pointcloud_geometry():
    from backend.pointcloud import build_pointcloud
    # Ein Punkt bei Winkel 0°, Distanz 1000 mm, z_angle 90° -> erwartete Lage prüfen
    frames = [{"angles": [0.0], "distances": [1000], "intensities": [100], "z_angle": 90.0}]
    pts, _ = build_pointcloud(frames, angle_offset=0.0, position_offset=(0, 0, 0),
                              dist_min_mm=30, dist_max_mm=25000)
    assert pts.shape == (1, 3)
    # Punkt in X-Z-Ebene (1000,0,0), Revolve -90° um Z -> (0,-1000,0)... Vorzeichen je Konvention
    r = np.linalg.norm(pts[0])
    assert abs(r - 1000) < 1e-6, r
    print(f"OK  Geometrie: Punkt={pts[0].round(1)}, |r|={r:.1f} mm")


def test_mock_scan():
    import os
    os.environ["PILIDAR_MOCK"] = "1"
    from backend.config import Config
    from backend.controller import ScanController

    cfg = Config()
    cfg._data["STORAGE"]["BASE_DIR"] = str(Path(__file__).resolve().parent / "_scans_tmp")
    # kleinen Scanwinkel für schnellen Test
    cfg._data["MODE_B_CONTINUOUS"]["SCAN_ANGLE"] = 20.0
    cfg._data["MODE_B_CONTINUOUS"]["SPEED_DPS"] = 40.0

    ctrl = ScanController(cfg, force_mock=True)
    # LiDAR mit gedrosselter Mock-Serial bestücken (~realistische Paketrate)
    pkt = build_packet()
    ctrl.lidar._serial_factory = lambda: MockSerial(pkt, packet_delay=0.0006)

    scan_id = ctrl.start_scan(mode="B")
    for _ in range(400):
        if ctrl.state == "idle":
            break
        time.sleep(0.05)
    ctrl.close()
    d = Path(cfg.STORAGE["BASE_DIR"]) / scan_id
    assert (d / "meta.json").exists(), "meta.json fehlt"
    assert (d / "raw" / "frames.jsonl").exists(), "Rohdaten fehlen"
    print(f"OK  Mock-Scan: {scan_id}, Ordner+meta+raw vorhanden, "
          f"finaler Winkel~={ctrl.stepper.get_current_angle():.1f}")


if __name__ == "__main__":
    test_crc_roundtrip()
    test_decode()
    test_pointcloud_geometry()
    test_mock_scan()
    print("\nAlle Trockentests bestanden.")
