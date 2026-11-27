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
        self.written = bytearray()

    def write(self, data):
        self.written += data
        return len(data)

    def flush(self):
        pass

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


def test_motor_commands():
    pkt = build_packet()
    # Standard: keine Befehle (RXD0 des ESP8266 auf dem Board kurzgeschlossen)
    ser = MockSerial(pkt)
    reader = LidarReader("mock", serial_factory=lambda: ser)
    reader.start(); time.sleep(0.05); reader.stop()
    reader.start(); time.sleep(0.05); reader.stop()
    assert ser.written == b"", ser.written
    # Mit Motorsteuerung: "0" beim Stopp, "1" beim Wiederanlauf
    ser = MockSerial(pkt)
    reader = LidarReader("mock", serial_factory=lambda: ser, motor_control=True)
    reader.start(); time.sleep(0.05); reader.stop()
    reader.start(); time.sleep(0.05); reader.stop()
    assert ser.written == b"010", ser.written
    print("OK  Motorbefehle: aus = nichts gesendet, an = b'010'")


def test_vectorized_equals_reference():
    from backend.pointcloud import build_pointcloud, polar_to_plane, transform_frame
    rng = np.random.default_rng(3)
    frames = [{"z_angle": float(rng.uniform(0, 184)), "angles": list(rng.uniform(0, 360, 12)),
               "distances": list(rng.integers(0, 5000, 12)), "intensities": list(rng.integers(0, 255, 12))}
              for _ in range(300)]
    pos = (0.0, -37.5, -41.9)
    pts, _ = build_pointcloud(frames, 178.95, pos, 30, 25000)
    ref = []
    for fr in frames:
        # Winkel wie im Speicherformat als float32 (LiDAR-Auflösung 0,01°)
        a = np.asarray(fr["angles"], dtype=np.float32).astype(float)
        d = np.asarray(fr["distances"], float)
        m = (d >= 30) & (d <= 25000)
        ref.append(transform_frame(polar_to_plane(a[m], d[m]), fr["z_angle"], 178.95, pos))
    ref = np.vstack(ref)
    assert pts.shape == ref.shape and np.abs(pts - ref).max() < 1e-6
    print(f"OK  vektorisierte Punktwolke = Referenz ({len(pts)} Punkte)")


def test_recorder():
    from backend.recorder import FrameRecorder
    import backend.recorder as rmod
    old = rmod._CHUNK; rmod._CHUNK = 7          # kleine Blöcke -> Blockwechsel testen
    try:
        r = FrameRecorder()
        for k in range(20):
            r.append(k * 0.5, [k] * 12, [100 + k] * 12, [k % 256] * 12, 3600.0, k, 1000.0 + k)
        a = r.arrays()
        assert len(r) == 20 and a["z_angle"].shape == (20,) and a["angles"].shape == (20, 12)
        assert a["z_angle"][19] == 9.5 and a["distances"][19, 0] == 119 and a["t"][0] == 1000.0
        r.clear(); assert len(r) == 0 and len(r.arrays()["z_angle"]) == 0
    finally:
        rmod._CHUNK = old
    print("OK  FrameRecorder (Blöcke, Felder, Leeren)")


def test_qa_gaps():
    from backend.qa import compute_qa
    n = 3000
    t = np.arange(n) / 1800.0
    t[1500:] += 0.3                                  # 300-ms-Lücke in der Mitte
    fr = {"z_angle": np.linspace(0, 184, n), "angles": np.zeros((n, 12)), "distances": np.zeros((n, 12)),
          "intensities": np.zeros((n, 12)), "speed_dps": np.full(n, 3600.0), "t": t}
    qa = compute_qa(fr, {"crc_error_rate": 0, "packet_rate": 1800}, {}, {}, 184.0, 4.0)
    assert len(qa["data_gaps"]) == 1 and qa["data_gaps"][0]["ms"] == 301, qa["data_gaps"]
    assert qa["status"] == "error" and "Datenlücke" in qa["messages"][0]
    print("OK  QA erkennt Datenlücken")


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
    # simulierter ESP32 meldet beim Öffnen einen Neustart -> Position unbekannt
    if getattr(ctrl.stepper, "position_lost", False):
        try:
            ctrl.start_scan(mode="B")
        except RuntimeError:
            pass
        else:
            raise AssertionError("Scan trotz unbekannter Position gestartet")
        ctrl.confirm_home()
    # LiDAR mit gedrosselter Mock-Serial bestücken (~realistische Paketrate)
    pkt = build_packet()
    ctrl.lidar._serial_factory = lambda: MockSerial(pkt, packet_delay=0.0006)

    scan_id = ctrl.start_scan(mode="B")
    for _ in range(400):
        if ctrl.state == "idle":
            break
        time.sleep(0.05)
    st = ctrl.selftest()
    assert st["status"] in ("ok", "warn", "error") and len(st["checks"]) >= 5, st
    assert any("size_mb" in m for m in ctrl.store.list_scans())
    ser = getattr(ctrl.stepper, "_ser", None)
    if ser is not None and hasattr(ser, "enabled"):   # ESP32-Backend (Mock)
        assert not ser.enabled, "Treiber muss nach dem Scan abgeschaltet sein"
    ctrl.close()
    d = Path(cfg.STORAGE["BASE_DIR"]) / scan_id
    assert (d / "meta.json").exists(), "meta.json fehlt"
    assert (d / "raw" / "lidar_raw.npz").exists(), "Rohdaten fehlen"
    assert not (d / "raw" / "frames.jsonl").exists(), "frames.jsonl sollte nicht mehr erzeugt werden"
    raw = ctrl.store.load_raw_arrays(scan_id)
    assert {"z_angle", "angles", "distances", "intensities", "t", "speed_dps"} <= set(raw), raw.keys()
    buf = ctrl.store.load_raw_packed(scan_id)
    n = struct.unpack("<I", buf[:4])[0]
    assert n == len(raw["z_angle"]) and len(buf) == 4 + n * (4 + 48 + 24 + 12)
    print(f"OK  Mock-Scan: {scan_id}, Ordner+meta+raw vorhanden, "
          f"finaler Winkel~={ctrl.stepper.get_current_angle():.1f}")


if __name__ == "__main__":
    test_crc_roundtrip()
    test_decode()
    test_motor_commands()
    test_vectorized_equals_reference()
    test_recorder()
    test_qa_gaps()
    test_pointcloud_geometry()
    test_mock_scan()
    print("\nAlle Trockentests bestanden.")
