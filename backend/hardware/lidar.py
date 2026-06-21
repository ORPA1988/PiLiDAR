"""LiDAR-Treiber für LDROBOT LD06 / LD19 / STL27L (identisches 47-Byte-Protokoll).

Der Treiber liest die serielle Schnittstelle (USB-CDC, /dev/ttyUSB0) in einem
eigenen Thread, validiert jedes Paket per CRC8 und ruft für jedes gültige Paket
einen Callback mit den dekodierten Rohwerten auf. Es findet bewusst KEINE
Punktwolken-Berechnung statt – der Pi liefert nur Rohdaten, die Mathematik läuft
im Client (siehe frontend/scan.worker.js). Eine identische Python-Referenz steht
in backend/pointcloud.py für den optionalen Server-Export.

Paketaufbau (47 Byte, Little-Endian Felder):
    [0]      0x54            Header
    [1]      0x2C            VerLen (12 Messpunkte)
    [2:4]    speed           Drehzahl des Lidar-Spiegels [°/s]
    [4:6]    FSA             Startwinkel [0.01°]
    [6:42]   12 x (dist u16 [mm] + intensity u8)
    [42:44]  LSA             Endwinkel [0.01°]
    [44:46]  timestamp       [ms], wrap bei 30000
    [46]     CRC8            über Byte [0:46]
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

try:  # pyserial ist auf dem Pi vorhanden, im Test optional
    import serial
except Exception:  # pragma: no cover
    serial = None


# --- CRC8 (LDROBOT, Polynom 0x4D, MSB-first, init 0) ---------------------
def _gen_crc_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        table.append(crc)
    return table


_CRC_TABLE = _gen_crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


HEADER = 0x54
VERLEN = 0x2C
PACKET_LEN = 47
POINTS_PER_PACKET = 12


@dataclass
class Frame:
    """Ein dekodiertes Paket. Winkel in Grad, Distanz in mm."""

    angles: list[float]          # 12 Punktwinkel (inkl. angle_offset) [°]
    distances: list[int]         # 12 Distanzen [mm]
    intensities: list[int]       # 12 Intensitäten [0..255]
    speed_dps: float             # Lidar-Drehzahl [°/s]
    timestamp_ms: int            # Lidar-interner Zeitstempel [ms]
    recv_time: float             # monotone Empfangszeit [s]
    z_angle: float = 0.0         # Plattform-/Drehwinkel zum Empfangszeitpunkt [°]


@dataclass
class LidarStats:
    packets_ok: int = 0
    packets_crc_fail: int = 0
    bytes_read: int = 0
    last_speed_dps: float = 0.0
    started_at: float = field(default_factory=time.monotonic)
    _final_rate: float | None = field(default=None, init=False, repr=False)

    def freeze(self) -> None:
        """Rate beim Stopp einfrieren, damit sie danach nicht weiter sinkt."""
        dt = max(1e-6, time.monotonic() - self.started_at)
        self._final_rate = self.packets_ok / dt

    @property
    def crc_error_rate(self) -> float:
        total = self.packets_ok + self.packets_crc_fail
        return (self.packets_crc_fail / total) if total else 0.0

    @property
    def packet_rate(self) -> float:
        if self._final_rate is not None:
            return self._final_rate
        dt = max(1e-6, time.monotonic() - self.started_at)
        return self.packets_ok / dt


class LidarReader:
    """Liest Pakete in einem Thread und ruft on_frame(frame) je gültigem Paket."""

    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        angle_offset: float = 0.0,
        on_frame: Optional[Callable[[Frame], None]] = None,
        angle_provider: Optional[Callable[[], float]] = None,
        serial_factory: Optional[Callable[[], "serial.Serial"]] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.angle_offset = angle_offset
        self.on_frame = on_frame
        # liefert den aktuellen Plattformwinkel (vom Stepper) zum Stempeln der Frames
        self.angle_provider = angle_provider or (lambda: 0.0)
        self._serial_factory = serial_factory

        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        # Der STL27L-Spiegel dreht ab Werk, sobald er mit Strom versorgt wird
        # (laut Waveshare-Doku stoppt erst ein "0"). Daher gilt er initial als
        # "läuft": beim normalen Start wird KEIN "1" gesendet, der Spiegel läuft
        # einfach durch. Ein "0" geht nur beim expliziten Stopp raus, ein "1" nur,
        # um nach einem solchen Stopp wieder anzufahren.
        self._motor_on = True
        self.stats = LidarStats()

    # ------------------------------------------------------------------
    def open(self) -> None:
        if self._serial_factory is not None:
            self._ser = self._serial_factory()
        else:
            if serial is None:
                raise RuntimeError("pyserial nicht installiert")
            self._ser = serial.Serial(self.port, self.baudrate, timeout=1)

    def close(self) -> None:
        self.stop()
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    # ------------------------------------------------------------------
    def _set_motor(self, on: bool) -> None:
        """Spiegelmotor schalten (Waveshare STL27L): b'1' = an, b'0' = aus.

        Idempotent: sendet den Befehl nur bei tatsächlichem Zustandswechsel, damit
        der Spiegel nicht versehentlich mehrfach hoch-/runtergefahren wird. Die
        ASCII-Zeichen "1"/"0" gehen über denselben Serial-Port (921600 Baud). Im
        Mock-Modus oder bei Serial-Fehlern wird still ignoriert.
        """
        if on == self._motor_on:
            return
        ser = self._ser
        if ser is not None:
            try:
                if getattr(ser, "is_open", True):
                    ser.write(b"1" if on else b"0")
                    ser.flush()
            except Exception:
                pass  # Serial-Schreibfehler nicht propagieren
        self._motor_on = on

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._running.is_set():
            return  # bereits aktiv – kein zweiter Reader-Thread, kein erneuter Motorbefehl
        if self._ser is None:
            self.open()
        # No-op falls der Spiegel ohnehin läuft; sendet "1" nur nach einem Stopp.
        self._set_motor(True)
        self.stats = LidarStats()
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="LidarReader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._running.is_set():
            self.stats.freeze()
            self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._set_motor(False)  # Spiegelmotor stoppen

    @property
    def running(self) -> bool:
        return self._running.is_set()

    # ------------------------------------------------------------------
    def _read_exact(self, n: int) -> bytes:
        buf = self._ser.read(n)
        return buf

    def _loop(self) -> None:
        ser = self._ser
        while self._running.is_set():
            # Auf Header synchronisieren
            b = ser.read(1)
            if not b:
                continue
            self.stats.bytes_read += 1
            if b[0] != HEADER:
                continue
            b2 = ser.read(1)
            if not b2 or b2[0] != VERLEN:
                continue
            rest = self._read_exact(PACKET_LEN - 2)
            if len(rest) != PACKET_LEN - 2:
                continue
            packet = bytes((HEADER, VERLEN)) + rest
            self.stats.bytes_read += len(rest)

            if crc8(packet[:PACKET_LEN - 1]) != packet[PACKET_LEN - 1]:
                self.stats.packets_crc_fail += 1
                continue

            frame = self._decode(packet)
            self.stats.packets_ok += 1
            self.stats.last_speed_dps = frame.speed_dps
            if self.on_frame is not None:
                self.on_frame(frame)

    # ------------------------------------------------------------------
    def _decode(self, p: bytes) -> Frame:
        speed = int.from_bytes(p[2:4], "little")              # °/s
        fsa = int.from_bytes(p[4:6], "little") / 100.0        # °
        lsa = int.from_bytes(p[42:44], "little") / 100.0      # °
        timestamp = int.from_bytes(p[44:46], "little")        # ms

        span = (lsa - fsa) if (lsa - fsa) > 0 else (lsa + 360.0 - fsa)
        step = span / (POINTS_PER_PACKET - 1)

        distances: list[int] = []
        intensities: list[int] = []
        angles: list[float] = []
        for i in range(POINTS_PER_PACKET):
            off = 6 + i * 3
            dist = int.from_bytes(p[off:off + 2], "little")
            inten = p[off + 2]
            # ROH-Winkel ohne angle_offset – die Offset-Korrektur erfolgt einmalig
            # als Y-Rotation in pointcloud.transform_frame (Client identisch).
            angle = (fsa + step * i) % 360.0
            distances.append(dist)
            intensities.append(inten)
            angles.append(angle)

        return Frame(
            angles=angles,
            distances=distances,
            intensities=intensities,
            speed_dps=float(speed),
            timestamp_ms=timestamp,
            recv_time=time.monotonic(),
            z_angle=float(self.angle_provider()),
        )
