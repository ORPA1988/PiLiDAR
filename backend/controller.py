"""Scan-Statemachine: verbindet LiDAR-Reader und Stepper, sammelt Frames,
broadcastet sie an WebSocket-Clients und speichert je Scan einen Ordner.
"""

from __future__ import annotations

import asyncio
import threading
import time
from collections import deque
from datetime import datetime

from .calibration import calibrate_offset, calibrate_rotation
from .config import Config
from .hardware.lidar import Frame, LidarReader
from .hardware.stepper import Stepper
from .pointcloud import build_pointcloud
from .qa import compute_qa
from .storage import ScanStore

STATE_IDLE = "idle"
STATE_LIDAR_ONLY = "lidar_only"
STATE_SCANNING = "scanning"
STATE_CALIBRATING = "calibrating"
STATE_PROCESSING = "processing"


def _frame_to_msg(fr: Frame) -> dict:
    return {
        "t": round(fr.recv_time, 4),
        "z": round(fr.z_angle, 3),
        "a": [round(x, 2) for x in fr.angles],
        "d": fr.distances,
        "i": fr.intensities,
        "s": fr.speed_dps,
    }


class ScanController:
    def __init__(self, config: Config, force_mock: bool = False):
        self.cfg = config
        s = config.STEPPER
        self.stepper = Stepper(
            dir_pin=s["DIR_PIN"], step_pin=s["STEP_PIN"], ms_pins=s["MS_PINS"],
            step_angle=s["STEP_ANGLE"], microsteps=s["MICROSTEPS"],
            gear_ratio=s["GEAR_RATIO"], step_delay=s["STEP_DELAY"],
            pwm_channel=config.MODE_B_CONTINUOUS["PWM_CHANNEL"], force_mock=force_mock,
        )
        l = config.LIDAR
        self.lidar = LidarReader(
            port=l["PORT"], baudrate=l["BAUDRATE"], angle_offset=l["ANGLE_OFFSET"],
            on_frame=self._on_frame, angle_provider=self.stepper.get_current_angle,
        )
        self.store = ScanStore(config.STORAGE["BASE_DIR"])

        self.state = STATE_IDLE
        self.current_scan_id: str | None = None
        self._frames: list[dict] = []          # gesammelte Frames des aktuellen Laufs
        self._recording = False
        self._lock = threading.Lock()

        # WebSocket-Broadcast
        self._loop: asyncio.AbstractEventLoop | None = None
        self._subscribers: set[asyncio.Queue] = set()
        self._last_frame: dict | None = None
        self._recent = deque(maxlen=50)

        self._worker: threading.Thread | None = None

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
            with self._lock:
                self._frames.append({
                    "z_angle": fr.z_angle,
                    "angles": fr.angles,
                    "distances": fr.distances,
                    "intensities": fr.intensities,
                    "speed_dps": fr.speed_dps,
                    "timestamp_ms": fr.timestamp_ms,
                })
        if self._loop is not None:
            for q in list(self._subscribers):
                try:
                    self._loop.call_soon_threadsafe(q.put_nowait, msg)
                except Exception:
                    pass

    # ------------------------------------------------------------------
    def status(self) -> dict:
        return {
            "state": self.state,
            "scan_id": self.current_scan_id,
            "angle": round(self.stepper.get_current_angle(), 2),
            "lidar_running": self.lidar.running,
            "frames": len(self._frames),
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
    def start_scan(self, mode: str = "B", name: str = "") -> str:
        if self.state not in (STATE_IDLE, STATE_LIDAR_ONLY):
            raise RuntimeError(f"Beschäftigt: {self.state}")
        if self.state == STATE_LIDAR_ONLY:
            self.lidar.stop()
        scan_id = datetime.now().strftime("%y%m%d-%H%M%S")
        if name:
            scan_id += "_" + "".join(c for c in name if c.isalnum() or c in "-_")
        self.current_scan_id = scan_id
        self.store.create(scan_id)
        with self._lock:
            self._frames = []
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
        self.current_scan_id = None
        self.state = STATE_IDLE

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
            frames = list(self._frames)
        geom = self.cfg.GEOMETRY
        lcfg = self.cfg.LIDAR
        pos = (0.0, geom["MODEL_Y_OFFSET"], geom["MODEL_Z_OFFSET"])
        # Server-Referenz-Punktwolke (Client rechnet ebenfalls; hier für Export/QA)
        pts, inten = build_pointcloud(frames, lcfg["ANGLE_OFFSET"], pos,
                                      lcfg["DISTANCE_MIN_MM"], lcfg["DISTANCE_MAX_MM"])
        self.store.save_raw_frames(scan_id, frames, self.cfg.STORAGE["RAW_FORMATS"])
        self.store.save_pointcloud(scan_id, pts, inten, self.cfg.STORAGE["POINTCLOUD_FORMATS"])
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
            "n_frames": len(frames),
            "n_points": int(len(pts)),
            "qa": qa,
        }
        self.store.write_meta(scan_id, meta)
        self.store.append_log(scan_id, f"Scan {scan_id} fertig: {len(pts)} Punkte, QA={qa['status']}")

    def stop(self) -> None:
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
        self.state = STATE_CALIBRATING
        try:
            self.stepper.home()
            self.lidar.start()
            time.sleep(0.3)
            with self._lock:
                self._frames = []
            self._recording = True
            cfg = self.cfg.MODE_B_CONTINUOUS
            self.stepper.start_continuous(cfg["SPEED_DPS"], cfg["ACCEL_DPS2"], direction=1)
            while self.stepper.get_current_angle() < 365 and self.state == STATE_CALIBRATING:
                time.sleep(0.02)
            self.stepper.stop_continuous(ramp_down=True)
            self._recording = False
            self.lidar.stop()
            with self._lock:
                frames = list(self._frames)
            result = calibrate_rotation(frames, self.cfg.STEPPER["GEAR_RATIO"])
        finally:
            self.stepper.home()
            self.state = STATE_IDLE
        return result

    def run_offset_calibration(self) -> dict:
        """Scan >180° (inkl. Δ), dann Achsversatz + angle_offset optimieren."""
        if self.state != STATE_IDLE:
            raise RuntimeError(f"Beschäftigt: {self.state}")
        self.state = STATE_CALIBRATING
        try:
            scan_angle = self.cfg.MODE_B_CONTINUOUS["SCAN_ANGLE"]
            overlap = self.cfg.MODE_B_CONTINUOUS["OVERLAP_DEG"]
            self.stepper.home()
            self.lidar.start()
            time.sleep(0.3)
            with self._lock:
                self._frames = []
            self._recording = True
            cfg = self.cfg.MODE_B_CONTINUOUS
            self.stepper.start_continuous(cfg["SPEED_DPS"], cfg["ACCEL_DPS2"], direction=1)
            while self.stepper.get_current_angle() < scan_angle and self.state == STATE_CALIBRATING:
                time.sleep(0.02)
            self.stepper.stop_continuous(ramp_down=True)
            self._recording = False
            self.lidar.stop()
            with self._lock:
                frames = list(self._frames)
            geom = self.cfg.GEOMETRY
            lcfg = self.cfg.LIDAR
            pos = (0.0, geom["MODEL_Y_OFFSET"], geom["MODEL_Z_OFFSET"])
            result = calibrate_offset(frames, lcfg["ANGLE_OFFSET"], pos,
                                      lcfg["DISTANCE_MIN_MM"], lcfg["DISTANCE_MAX_MM"],
                                      scan_angle, overlap)
        finally:
            self.stepper.home()
            self.state = STATE_IDLE
        return result

    def close(self) -> None:
        try:
            self.lidar.close()
        finally:
            self.stepper.close()
