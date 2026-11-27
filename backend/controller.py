"""Scan-Statemachine: verbindet LiDAR-Reader und Stepper, sammelt Frames,
broadcastet sie an WebSocket-Clients und speichert je Scan einen Ordner.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections import deque
from datetime import datetime

from .calibration import calibrate_offset, calibrate_rotation
from .pointcloud import frames_to_arrays
from .recorder import FrameRecorder
from .config import Config
from .hardware.lidar import Frame, LidarReader
from .hardware.stepper import Stepper
from .hardware.stepper_esp32 import Esp32Stepper
from .pointcloud import build_pointcloud
from .qa import compute_qa
from .storage import ScanStore

log = logging.getLogger(__name__)

STATE_IDLE = "idle"
STATE_LIDAR_ONLY = "lidar_only"
STATE_SCANNING = "scanning"
STATE_CALIBRATING = "calibrating"
STATE_PROCESSING = "processing"
STATE_STOPPING = "stopping"


def _offer(q: asyncio.Queue, msg: dict) -> None:
    """In die Client-Queue legen; ist sie voll (Client zu langsam), das älteste
    Element verwerfen. Läuft im Event-Loop – darf keine Ausnahme werfen, sonst
    protokolliert asyncio bei jedem Paket die komplette Queue."""
    try:
        q.put_nowait(msg)
    except asyncio.QueueFull:
        try:
            q.get_nowait()
        except asyncio.QueueEmpty:
            pass
        try:
            q.put_nowait(msg)
        except asyncio.QueueFull:
            pass


def _frame_to_msg(fr: Frame) -> dict:
    return {
        "t": round(fr.recv_time, 4),
        "z": round(fr.z_angle, 3),
        "a": fr.angles,          # wird binär (float32) gesendet, kein Runden nötig
        "d": fr.distances,
        "i": fr.intensities,
        "s": fr.speed_dps,
    }


class ScanController:
    def __init__(self, config: Config, force_mock: bool = False):
        self.cfg = config
        self.stepper = self._make_stepper(config, force_mock)
        l = config.LIDAR
        self.lidar = LidarReader(
            port=l["PORT"], baudrate=l["BAUDRATE"], angle_offset=l["ANGLE_OFFSET"],
            on_frame=self._on_frame, angle_provider=self.stepper.get_current_angle,
            motor_control=bool(l.get("MOTOR_CONTROL", False)),
        )
        self.store = ScanStore(config.STORAGE["BASE_DIR"])

        self.state = STATE_IDLE
        self.current_scan_id: str | None = None
        self._rec = FrameRecorder()             # Frames des aktuellen Laufs (kompakt, numpy)
        self._recording = False
        self._lock = threading.Lock()

        # WebSocket-Broadcast
        self._loop: asyncio.AbstractEventLoop | None = None
        self._subscribers: set[asyncio.Queue] = set()
        self._last_frame: dict | None = None
        self._recent = deque(maxlen=50)

        self._worker: threading.Thread | None = None

    @staticmethod
    def _make_stepper(config: Config, force_mock: bool):
        """STEPPER.BACKEND: "gpio" (STEP/DIR am Pi) oder "esp32" (USB-Seriell)."""
        s = config.STEPPER
        backend = str(s.get("BACKEND", "gpio")).lower()
        if backend == "esp32":
            factory = None
            if force_mock:
                from .hardware.mock_esp32 import MockEsp32Serial
                factory = MockEsp32Serial
            return Esp32Stepper(
                port=s.get("PORT", "auto"), step_angle=s["STEP_ANGLE"],
                microsteps=s["MICROSTEPS"], gear_ratio=s["GEAR_RATIO"],
                baudrate=s.get("BAUDRATE", 115200),
                move_speed_dps=s.get("MOVE_SPEED_DPS", 20.0),
                move_accel_dps2=s.get("MOVE_ACCEL_DPS2", 40.0),
                dir_invert=bool(s.get("DIR_INVERT", False)),
                report_ms=s.get("REPORT_MS", 20),
                exclude_ports=(config.LIDAR["PORT"],),
                open_mode=s.get("OPEN_MODE", "reset"),
                serial_factory=factory,
            )
        if backend != "gpio":
            raise ValueError(f"STEPPER.BACKEND unbekannt: {backend!r}")
        return Stepper(
            dir_pin=s["DIR_PIN"], step_pin=s["STEP_PIN"], ms_pins=s["MS_PINS"],
            step_angle=s["STEP_ANGLE"], microsteps=s["MICROSTEPS"],
            gear_ratio=s["GEAR_RATIO"], step_delay=s["STEP_DELAY"],
            pwm_channel=config.MODE_B_CONTINUOUS["PWM_CHANNEL"], force_mock=force_mock,
        )

    # ------------------------------------------------------------------
    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=2000)
        self._subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._subscribers.discard(q)

    # ------------------------------------------------------------------
    def _on_frame(self, fr: Frame) -> None:
        msg = _frame_to_msg(fr)
        self._last_frame = msg
        self._recent.append(msg)
        if self._recording:
            self._rec.append(fr.z_angle, fr.angles, fr.distances, fr.intensities,
                             fr.speed_dps, fr.timestamp_ms, fr.recv_time)
        if self._loop is not None:
            for q in list(self._subscribers):
                try:
                    self._loop.call_soon_threadsafe(_offer, q, msg)
                except Exception:
                    pass

    # ------------------------------------------------------------------
    def status(self) -> dict:
        return {
            "state": self.state,
            "scan_id": self.current_scan_id,
            "angle": round(self.stepper.get_current_angle(), 2),
            "lidar_running": self.lidar.running,
            "stepper_backend": "esp32" if isinstance(self.stepper, Esp32Stepper) else "gpio",
            "stepper_connected": bool(getattr(self.stepper, "connected", True)),
            "position_lost": bool(getattr(self.stepper, "position_lost", False)),
            "frames": len(self._rec),
            "stats": {
                "packets_ok": self.lidar.stats.packets_ok,
                "crc_error_rate": round(self.lidar.stats.crc_error_rate, 4),
                "packet_rate": round(self.lidar.stats.packet_rate, 1),
                "last_speed_dps": self.lidar.stats.last_speed_dps,
            },
        }

    # --- LiDAR-Only (2D-Live, Motor aus) ------------------------------
    def start_lidar_only(self) -> None:
        if self.state != STATE_IDLE:
            raise RuntimeError(f"Beschäftigt: {self.state}")
        self._recording = False
        self.lidar.start()
        self.state = STATE_LIDAR_ONLY

    def stop_lidar_only(self) -> None:
        if self.state == STATE_LIDAR_ONLY:
            self.lidar.stop()
            self.state = STATE_IDLE

    # --- Scan ----------------------------------------------------------
    def _check_position(self) -> None:
        if getattr(self.stepper, "position_lost", False):
            raise RuntimeError("Position des Drehkopfs unbekannt (ESP32-Neustart). Kopf von Hand "
                               "in die Ausgangslage drehen und „Ausgangslage bestätigen“ klicken.")

    def confirm_home(self) -> None:
        if self.state not in (STATE_IDLE, STATE_LIDAR_ONLY):
            raise RuntimeError(f"Beschäftigt: {self.state}")
        self.stepper.confirm_home()

    def start_scan(self, mode: str = "B", name: str = "") -> str:
        if self.state not in (STATE_IDLE, STATE_LIDAR_ONLY):
            raise RuntimeError(f"Beschäftigt: {self.state}")
        self._check_position()
        # Den LiDAR-Spiegel beim Übergang aus 2D-Live NICHT aus- und wieder
        # einschalten – er soll durchgehend laufen. lidar.start() in _run_scan
        # ist idempotent (No-op, falls bereits aktiv).
        scan_id = datetime.now().strftime("%y%m%d-%H%M%S")
        if name:
            scan_id += "_" + "".join(c for c in name if c.isalnum() or c in "-_")
        self.current_scan_id = scan_id
        self.store.create(scan_id)
        self._rec.clear()
        self._worker = threading.Thread(target=self._run_scan, args=(mode, scan_id),
                                        daemon=True, name="ScanRun")
        self.state = STATE_SCANNING
        self._worker.start()
        return scan_id

    def _run_scan(self, mode: str, scan_id: str) -> None:
        t0 = time.time()
        self.stepper.home()  # sicher in Ausgangsposition starten
        self.lidar.start()
        time.sleep(0.3)       # LiDAR-Drehzahl stabilisieren
        self._recording = True
        try:
            if mode.upper() == "A":
                self._run_mode_a()
                scan_angle = self.cfg.MODE_A_STEPWISE["SCAN_ANGLE"]
                overlap = 0.0
            else:
                scan_angle = self.cfg.MODE_B_CONTINUOUS["SCAN_ANGLE"]
                overlap = self.cfg.MODE_B_CONTINUOUS["OVERLAP_DEG"]
                self._run_mode_b()
        finally:
            self._recording = False
            time.sleep(0.1)
            self.lidar.stop()

        self.state = STATE_PROCESSING
        self._finalize(scan_id, mode, scan_angle, overlap, t0)
        self.stepper.home()
        self._release_stepper()
        self.current_scan_id = None
        self.state = STATE_IDLE

    def _release_stepper(self) -> None:
        """Nach Scan/Kalibrierung den Treiber abschalten (STEPPER.RELEASE_AFTER_SCAN,
        Standard an): sonst hält der A4988 im Stillstand Strom, der Motor fiept
        und wird warm."""
        if not bool(self.cfg.STEPPER.get("RELEASE_AFTER_SCAN", True)):
            return
        try:
            self.stepper.release()
        except Exception as exc:  # Abschalten darf den Ablauf nicht abbrechen
            log.warning("Treiber abschalten fehlgeschlagen: %s", exc)

    def _run_mode_a(self) -> None:
        cfg = self.cfg.MODE_A_STEPWISE
        scan_angle = cfg["SCAN_ANGLE"]
        spm = max(1, int(cfg["STEPS_PER_MEASURE"]))
        delay = float(cfg["SCAN_DELAY"])
        max_pkgs = int(cfg["MAX_PACKAGES"])
        timeout = delay * 5  # maximale Wartezeit pro Schritt

        while self.stepper.get_current_angle() < scan_angle and self.state == STATE_SCANNING:
            self.stepper.move_steps(spm)
            pkgs_before = self.lidar.stats.packets_ok
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and self.state == STATE_SCANNING:
                if self.lidar.stats.packets_ok - pkgs_before >= max_pkgs:
                    break
                time.sleep(0.005)

    def _run_mode_b(self) -> None:
        cfg = self.cfg.MODE_B_CONTINUOUS
        scan_angle = cfg["SCAN_ANGLE"]
        self.stepper.start_continuous(cfg["SPEED_DPS"], cfg["ACCEL_DPS2"], direction=1)
        while self.stepper.get_current_angle() < scan_angle and self.state == STATE_SCANNING:
            time.sleep(0.02)
        self.stepper.stop_continuous(ramp_down=True)

    def _finalize(self, scan_id, mode, scan_angle, overlap, t0) -> None:
        with self._lock:
            frames = self._rec.arrays()
        geom = self.cfg.GEOMETRY
        lcfg = self.cfg.LIDAR
        pos = (0.0, geom["MODEL_Y_OFFSET"], geom["MODEL_Z_OFFSET"])
        # Server-Referenz-Punktwolke (Client rechnet ebenfalls; hier für Export/QA)
        pts, inten = build_pointcloud(frames, lcfg["ANGLE_OFFSET"], pos,
                                      lcfg["DISTANCE_MIN_MM"], lcfg["DISTANCE_MAX_MM"])
        self.store.save_raw_frames(scan_id, frames, self.cfg.STORAGE.get("RAW_FORMATS", ["npy"]))
        self.store.save_pointcloud(scan_id, pts, inten, self.cfg.STORAGE.get("POINTCLOUD_FORMATS", ["ply"]))
        stats = {
            "crc_error_rate": self.lidar.stats.crc_error_rate,
            "packet_rate": self.lidar.stats.packet_rate,
        }
        qa = compute_qa(frames, stats, geom, lcfg, scan_angle, overlap)
        meta = {
            "id": scan_id,
            "mode": mode,
            "created": datetime.now().isoformat(timespec="seconds"),
            "duration_s": round(time.time() - t0, 1),
            "scan_angle": scan_angle,
            "overlap_deg": overlap,
            "lidar": lcfg.as_dict() if hasattr(lcfg, "as_dict") else dict(lcfg),
            "geometry": dict(geom),
            "n_frames": int(len(frames["z_angle"])),
            "n_points": int(len(pts)),
            "qa": qa,
        }
        self.store.write_meta(scan_id, meta)
        self.store.append_log(scan_id, f"Scan {scan_id} fertig: {len(pts)} Punkte, QA={qa['status']}")

    def stop(self) -> None:
        """Abbrechen. Läuft ein Scan oder eine Kalibrierung, beendet deren Thread
        den Ablauf selbst (Kopf zurück, Treiber aus) und setzt danach "idle" –
        bis dahin "stopping", damit kein neuer Lauf dazwischen startet."""
        if self.state in (STATE_SCANNING, STATE_CALIBRATING):
            self.state = STATE_STOPPING
        elif self.state != STATE_PROCESSING:
            self.state = STATE_IDLE
        self._recording = False
        try:
            self.stepper.stop_continuous(ramp_down=False)
        except Exception:
            pass
        self.lidar.stop()

    # --- Kalibrierung --------------------------------------------------
    def run_rotation_calibration(self) -> dict:
        """Vollständiger 360°-Lauf, dann Getriebeverhältnis schätzen."""
        if self.state != STATE_IDLE:
            raise RuntimeError(f"Beschäftigt: {self.state}")
        self._check_position()
        self.state = STATE_CALIBRATING
        try:
            self.stepper.home()
            self.lidar.start()
            time.sleep(0.3)
            self._rec.clear()
            self._recording = True
            cfg = self.cfg.MODE_B_CONTINUOUS
            self.stepper.start_continuous(cfg["SPEED_DPS"], cfg["ACCEL_DPS2"], direction=1)
            while self.stepper.get_current_angle() < 365 and self.state == STATE_CALIBRATING:
                time.sleep(0.02)
            self.stepper.stop_continuous(ramp_down=True)
            self._recording = False
            self.lidar.stop()
            frames = self._rec.arrays()
            result = calibrate_rotation(frames, self.cfg.STEPPER["GEAR_RATIO"])
        finally:
            self.stepper.home()
            self._release_stepper()
            self.state = STATE_IDLE
        return result

    def run_offset_calibration(self) -> dict:
        """Scan >180° (inkl. Δ), dann Achsversatz + angle_offset optimieren."""
        if self.state != STATE_IDLE:
            raise RuntimeError(f"Beschäftigt: {self.state}")
        self._check_position()
        self.state = STATE_CALIBRATING
        try:
            scan_angle = self.cfg.MODE_B_CONTINUOUS["SCAN_ANGLE"]
            overlap = self.cfg.MODE_B_CONTINUOUS["OVERLAP_DEG"]
            self.stepper.home()
            self.lidar.start()
            time.sleep(0.3)
            self._rec.clear()
            self._recording = True
            cfg = self.cfg.MODE_B_CONTINUOUS
            self.stepper.start_continuous(cfg["SPEED_DPS"], cfg["ACCEL_DPS2"], direction=1)
            while self.stepper.get_current_angle() < scan_angle and self.state == STATE_CALIBRATING:
                time.sleep(0.02)
            self.stepper.stop_continuous(ramp_down=True)
            self._recording = False
            self.lidar.stop()
            frames = self._rec.arrays()
            geom = self.cfg.GEOMETRY
            lcfg = self.cfg.LIDAR
            pos = (0.0, geom["MODEL_Y_OFFSET"], geom["MODEL_Z_OFFSET"])
            if self.state != STATE_CALIBRATING:
                return {"ok": False, "reason": "abgebrochen"}
            result = calibrate_offset(frames, lcfg["ANGLE_OFFSET"], pos,
                                      lcfg["DISTANCE_MIN_MM"], lcfg["DISTANCE_MAX_MM"],
                                      scan_angle, overlap,
                                      should_abort=lambda: self.state != STATE_CALIBRATING)
        finally:
            self.stepper.home()
            self._release_stepper()
            self.state = STATE_IDLE
        return result

    def selftest(self) -> dict:
        """Kurzer Selbsttest ohne Bewegung: LiDAR-Port, ESP32, Speicher, Konfiguration."""
        import os
        import shutil
        checks = []

        def add(name, ok, detail, level="error"):
            checks.append({"name": name, "ok": bool(ok), "level": "ok" if ok else level, "detail": detail})

        port = self.cfg.LIDAR["PORT"]
        add("LiDAR-Port", os.path.exists(port) or str(port).startswith("mock"),
            port if os.path.exists(port) else f"nicht gefunden: {port}")
        connected = bool(getattr(self.stepper, "connected", True))
        add("Schrittmotor-Steuerung", connected,
            "verbunden" if connected else "ESP32 nicht verbunden (USB?)")
        lost = bool(getattr(self.stepper, "position_lost", False))
        add("Position des Drehkopfs", not lost,
            "bekannt" if not lost else "unbekannt – Ausgangslage bestätigen", level="warn")
        try:
            du = shutil.disk_usage(str(self.store.base))
            free_gb = du.free / 1e9
            add("Speicherplatz", free_gb > 2, f"{free_gb:.1f} GB frei", level="warn" if free_gb > 0.5 else "error")
        except OSError as exc:
            add("Speicherplatz", False, str(exc))
        lc = self.cfg.LIDAR
        add("LiDAR-Motorbefehle", True, "aus (Board-RX defekt)" if not lc.get("MOTOR_CONTROL") else "an")
        add("Treiber nach Scan aus", bool(self.cfg.STEPPER.get("RELEASE_AFTER_SCAN", True)),
            "ja" if self.cfg.STEPPER.get("RELEASE_AFTER_SCAN", True) else "nein (Motor bleibt bestromt)", level="warn")
        worst = "ok"
        for c in checks:
            if c["level"] == "error":
                worst = "error"
            elif c["level"] == "warn" and worst == "ok":
                worst = "warn"
        return {"status": worst, "checks": checks}

    def check_offsets(self, scan_id: str, angle_offset: float, y_mm: float, z_mm: float = 0.0) -> dict:
        """Überlappung (Abstand Anfang/Ende des Scans, mm) für frei gewählte Offsets –
        zum Prüfen von Hand eingegebener Werte, ohne Motor/LiDAR, ohne zu speichern."""
        from .qa import loop_closure_residual
        arr = self.store.load_raw_arrays(scan_id)
        if arr is None:
            raise RuntimeError(f"Keine Rohdaten für Scan {scan_id}")
        meta = next((m for m in self.store.list_scans() if m.get("id") == scan_id), {})
        scan_angle = float(meta.get("scan_angle") or self.cfg.MODE_B_CONTINUOUS["SCAN_ANGLE"])
        overlap = float(meta.get("overlap_deg") or self.cfg.MODE_B_CONTINUOUS["OVERLAP_DEG"])
        if scan_angle < 180.0 + overlap - 1e-6 or overlap <= 0:
            raise RuntimeError("Scan hat keine Überlappung (Modus B mit > 180° nötig)")
        lc = self.cfg.LIDAR
        geom = self.cfg.GEOMETRY

        def res(a, y, z):
            v = loop_closure_residual(arr, a, (0.0, y, z), lc["DISTANCE_MIN_MM"], lc["DISTANCE_MAX_MM"],
                                      scan_angle, overlap, max_pts=20000)
            return None if v is None else round(float(v), 1)
        return {"ok": True, "scan_id": scan_id,
                "test": {"angle_offset": angle_offset, "model_y_offset": y_mm, "model_z_offset": z_mm,
                         "loop_closure_mm": res(angle_offset, y_mm, z_mm)},
                "current": {"angle_offset": lc["ANGLE_OFFSET"], "model_y_offset": geom["MODEL_Y_OFFSET"],
                            "model_z_offset": geom["MODEL_Z_OFFSET"],
                            "loop_closure_mm": res(lc["ANGLE_OFFSET"], geom["MODEL_Y_OFFSET"], geom["MODEL_Z_OFFSET"])}}

    def offset_from_scan(self, scan_id: str) -> dict:
        """Offset-Kalibrierung aus den Rohdaten eines gespeicherten Scans – ohne Motor
        und LiDAR (Modus-B-Scan mit Überlappung nötig)."""
        arr = self.store.load_raw_arrays(scan_id)
        if arr is None:
            raise RuntimeError(f"Keine Rohdaten für Scan {scan_id}")
        meta = next((m for m in self.store.list_scans() if m.get("id") == scan_id), {})
        scan_angle = float(meta.get("scan_angle") or self.cfg.MODE_B_CONTINUOUS["SCAN_ANGLE"])
        overlap = float(meta.get("overlap_deg") or self.cfg.MODE_B_CONTINUOUS["OVERLAP_DEG"])
        if scan_angle < 180.0 + overlap - 1e-6 or overlap <= 0:
            raise RuntimeError("Scan hat keine Überlappung (Modus B mit > 180° nötig)")
        geom, lcfg = self.cfg.GEOMETRY, self.cfg.LIDAR
        pos = (0.0, geom["MODEL_Y_OFFSET"], geom["MODEL_Z_OFFSET"])
        res = calibrate_offset(arr, lcfg["ANGLE_OFFSET"], pos, lcfg["DISTANCE_MIN_MM"],
                               lcfg["DISTANCE_MAX_MM"], scan_angle, overlap)
        res["scan_id"] = scan_id
        # Höhe (Z) ist über die Überlappung nicht bestimmbar -> nicht vorschlagen
        res["model_z_offset"] = float(geom["MODEL_Z_OFFSET"])
        return res

    def close(self) -> None:
        try:
            self.lidar.close()
        finally:
            self.stepper.close()
