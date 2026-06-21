"""Synthetischer LiDAR (für Demo/Test ohne Hardware).

Erzeugt gültige 47-Byte-Pakete eines STL27L, der einen rechteckigen Raum
abtastet. Damit ist die komplette Web-Oberfläche ohne Sensor demonstrierbar
(PILIDAR_MOCK=1). Drosselt auf ~1800 Pakete/s.
"""

from __future__ import annotations

import math
import struct
import time

from .lidar import HEADER, VERLEN, crc8

ROOM_W = 4000.0   # mm
ROOM_H = 3000.0   # mm


def _ray_to_box(angle_deg: float) -> int:
    """Distanz vom Zentrum zur Raumwand in Richtung angle (vertikale Ebene)."""
    a = math.radians(angle_deg % 360.0)
    dx, dy = math.cos(a), math.sin(a)
    t = float("inf")
    if abs(dx) > 1e-6:
        t = min(t, abs((ROOM_W / 2) / dx))
    if abs(dy) > 1e-6:
        t = min(t, abs((ROOM_H / 2) / dy))
    # leichtes Rauschen + ein "Möbelstück"
    d = t
    if 30 < (angle_deg % 360) < 70:
        d = min(d, 900.0)
    return int(max(30, min(25000, d + (hash(round(angle_deg, 1)) % 11 - 5))))


class MockLidarSerial:
    def __init__(self, throttle: bool = True):
        self.fsa = 0.0
        self.span = 22.0  # ° pro Paket (12 Punkte)
        self.timestamp = 0
        self._buf = bytearray()
        self.throttle = throttle

    def _make_packet(self) -> bytes:
        body = bytearray()
        body.append(HEADER)
        body.append(VERLEN)
        body += struct.pack("<H", 3600)                    # 10 Hz -> 3600 °/s
        body += struct.pack("<H", int(self.fsa * 100) % 36000)
        for i in range(12):
            ang = self.fsa + self.span / 11.0 * i
            d = _ray_to_box(ang)
            body += struct.pack("<H", d)
            body.append(180)
        lsa = (self.fsa + self.span) % 360.0
        body += struct.pack("<H", int(lsa * 100))
        self.timestamp = (self.timestamp + 1) % 30000
        body += struct.pack("<H", self.timestamp)
        body.append(crc8(bytes(body)))
        self.fsa = lsa
        return bytes(body)

    def read(self, n: int = 1) -> bytes:
        while len(self._buf) < n:
            self._buf += self._make_packet()
            if self.throttle:
                time.sleep(0.00055)  # ~1800 Pakete/s
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def write(self, data: bytes) -> int:
        # Motor-Steuerbefehle (b"0"/b"1") werden im Mock ignoriert.
        return len(data)

    def flush(self) -> None:
        pass

    def close(self):
        pass

    @property
    def is_open(self):
        return True
