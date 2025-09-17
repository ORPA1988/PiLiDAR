"""High level orchestration for a complete PiLiDAR scan."""

from __future__ import annotations

import os
import json
import threading
import time
from statistics import mean
from typing import Callable, Dict, List, Optional

from lib.pointcloud import save_summary


class ScanController:
    """Manage the LiDAR, stepper motor and camera pipeline."""

    def __init__(
        self,
        config,
        stepper_factory: Optional[Callable] = None,
        lidar_factory: Optional[Callable] = None,
    ) -> None:
        self.config = config
        self._status_callback: Optional[Callable[[str], None]] = None
        self._completion_callback: Optional[Callable[[bool, Optional[Exception]], None]] = None

        self._stop_event = threading.Event()
        self._scan_thread: Optional[threading.Thread] = None
        self.scan_result: Dict[str, Optional[object]] = {}
        self.scan_metadata: Dict[str, object] = {}

        self.stepper = None
        self.lidar = None
        self.custom_scan_id: Optional[str] = None

        self.stepper_factory = stepper_factory or self._default_stepper_factory
        self.lidar_factory = lidar_factory or self._default_lidar_factory

    # ------------------------------------------------------------------
    # factory helpers
    # ------------------------------------------------------------------
    def _default_stepper_factory(self, config):  # pragma: no cover - hardware specific
        from lib.a4988_driver import A4988

        pins = config.get("STEPPER", "pins", "MS_PINS")
        return A4988(
            config.get("STEPPER", "pins", "DIR_PIN"),
            config.get("STEPPER", "pins", "STEP_PIN"),
            pins,
            delay=config.get("STEPPER", "STEP_DELAY"),
            step_angle=config.get("STEPPER", "STEP_ANGLE"),
            microsteps=config.get("STEPPER", "MICROSTEPS"),
            gear_ratio=config.get("STEPPER", "GEAR_RATIO"),
        )

    def _default_lidar_factory(self, config):  # pragma: no cover - hardware specific
        from lib.lidar_driver import Lidar

        return Lidar(config, visualization=None)

    # ------------------------------------------------------------------
    # callback registration
    # ------------------------------------------------------------------
    def register_status_callback(self, callback: Callable[[str], None]) -> None:
        self._status_callback = callback

    def register_completion_callback(self, callback: Callable[[bool, Optional[Exception]], None]) -> None:
        self._completion_callback = callback

    def set_scan_id(self, scan_id: Optional[str]) -> None:
        self.custom_scan_id = scan_id or None

    def _notify(self, message: str) -> None:
        print(message)
        if self._status_callback:
            self._status_callback(message)

    def _notify_completion(self, success: bool, error: Optional[Exception]) -> None:
        if self._completion_callback:
            self._completion_callback(success, error)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def start_scan_async(self) -> None:
        if self._scan_thread and self._scan_thread.is_alive():
            self._notify("Ein Scan läuft bereits.")
            return
        self._stop_event.clear()
        self._scan_thread = threading.Thread(target=self._run_with_callbacks, daemon=True)
        self._scan_thread.start()

    def run_scan(self) -> Dict[str, Optional[object]]:
        self._stop_event.clear()
        result = self._execute_scan()
        self.scan_result = result
        return result

    def _run_with_callbacks(self) -> None:
        error = None
        try:
            self.scan_result = self._execute_scan()
            self._notify_completion(True, None)
        except Exception as exc:  # pragma: no cover - debugging aid
            error = exc
            self._notify(f"Fehler: {exc}")
            self._notify_completion(False, exc)
        finally:
            if error:
                self.scan_result = {}

    def request_stop(self) -> None:
        self._notify("Stopp angefordert.")
        self._stop_event.set()
        if self.lidar is not None:
            self.lidar.request_stop()

    def is_running(self) -> bool:
        return self._scan_thread is not None and self._scan_thread.is_alive()

    # ------------------------------------------------------------------
    # scan pipeline
    # ------------------------------------------------------------------
    def _execute_scan(self) -> Dict[str, Optional[object]]:
        self._notify("Bereite neuen Scan vor...")
        self.config.init(scan_id=self.custom_scan_id)
        self.config.relay_on()

        self.scan_metadata = {
            "start_time": time.time(),
            "step_events": [],
            "requested_steps": 0,
            "completed_steps": 0,
            "stop_requested": False,
        }

        self.stepper = self.stepper_factory(self.config) if self.config.get("ENABLE_LIDAR") else None
        self.lidar = self.lidar_factory(self.config) if self.config.get("ENABLE_LIDAR") else None
        if self.lidar is not None:
            self.lidar.set_status_callback(self._notify)

        try:
            if self._stop_event.is_set():
                return {}

            if self.config.get("ENABLE_LIDAR") and self.lidar is not None:
                self._notify("LiDAR wird hochgefahren...")
                self.lidar.power_on()
                self.lidar.start_motor()
                time.sleep(2)

            if self.config.get("ENABLE_CAM"):
                self._capture_panorama()

            raw_scan = None
            if self.config.get("ENABLE_LIDAR") and self.lidar is not None:
                raw_scan = self._capture_lidar()

            pano_path = None
            if self.config.get("ENABLE_CAM") and self.config.get("ENABLE_PANO"):
                pano_path = self._build_panorama()

            pointclouds = {}
            if self.config.get("ENABLE_3D") and (raw_scan or os.path.exists(self.config.raw_path)):
                pointclouds = self._process_pointcloud()

            result = {
                "raw_scan": raw_scan,
                "panorama": pano_path,
                "pointclouds": pointclouds,
                "scan_dir": self.config.scan_dir,
            }
            self._notify("Scan erfolgreich abgeschlossen.")
            return result
        finally:
            self._cleanup()

    # ------------------------------------------------------------------
    # individual pipeline stages
    # ------------------------------------------------------------------
    def _capture_panorama(self) -> None:
        from lib.rpicam_utils import estimate_camera_parameters, take_HDR_photo

        self._notify("Kalibriere Kamera...")
        exposure_time, gain, awbgains = estimate_camera_parameters(self.config)

        IMGCOUNT = self.config.get("PANO", "IMGCOUNT")
        for i in range(IMGCOUNT):
            if self._stop_event.is_set():
                self._notify("Panorama-Aufnahme abgebrochen.")
                return

            current_angle = 0 if self.stepper is None else self.stepper.get_current_angle()
            formatted_angle = format(current_angle, f".{self.config.get('ANGULAR_DIGITS')}f")
            self._notify(f"Foto {i + 1}/{IMGCOUNT} bei {formatted_angle}°")

            imgname = f"image_{formatted_angle}.jpg"
            imgpath = os.path.join(self.config.img_dir, imgname)

            imgpaths = take_HDR_photo(
                AEB=self.config.get("CAM", "AEB"),
                AEB_stops=self.config.get("CAM", "AEB_STOPS"),
                path=imgpath,
                exposure_time=exposure_time,
                gain=gain,
                awbgains=awbgains,
                denoise=self.config.get("CAM", "denoise"),
                sharpness=self.config.get("CAM", "sharpness"),
                saturation=self.config.get("CAM", "saturation"),
                save_raw=self.config.get("CAM", "raw"),
                blocking=True,
            )

            self.config.imglist.extend(imgpaths)

            if self.stepper is not None:
                self.stepper.move_to_angle((360 / IMGCOUNT) * (i + 1))
                time.sleep(0.5)

        if self.stepper is not None:
            self.stepper.move_to_angle(0)
            time.sleep(0.5)

    def _capture_lidar(self):
        scan_delay = self.config.get("STEPPER", "SCAN_DELAY")

        def move_steps_callback():
            if self._stop_event.is_set():
                self.lidar.request_stop()
                self.scan_metadata["stop_requested"] = True
                return
            if self.stepper is None:
                return
            requested = self.config.steps if self.config.SCAN_ANGLE > 0 else -self.config.steps
            moved = self.stepper.move_steps(requested)
            self.scan_metadata["requested_steps"] += abs(requested)
            self.scan_metadata["completed_steps"] += abs(moved)
            event = {
                "timestamp": time.time(),
                "requested": requested,
                "completed": moved,
                "z_angle": self.stepper.get_current_angle(),
            }
            self.scan_metadata["step_events"].append(event)
            self.lidar.z_angle = event["z_angle"]
            time.sleep(scan_delay)

        self._notify("Starte LiDAR-Aufnahme...")
        self.lidar.read_loop(callback=move_steps_callback, max_packages=self.config.max_packages)

        if self.stepper is not None:
            self.stepper.move_to_angle(0)

        metadata = self._finalize_metadata(len(self.lidar.package_history))
        raw_scan = self.lidar.create_raw_scan(metadata=metadata)
        self._notify(f"Rohdaten gespeichert unter {self.config.raw_path}")
        return raw_scan

    def _build_panorama(self):
        from lib.pano_utils import hugin_stitch

        self._notify("Berechne Panorama...")
        pano_path = hugin_stitch(self.config)
        return pano_path

    def _process_pointcloud(self):
        from lib.pointcloud import process_raw

        self._notify("Erzeuge Punktwolken...")
        clouds = process_raw(self.config, save=True)
        return {
            "intensity": clouds.intensity,
            "color": clouds.color,
            "filtered": clouds.filtered,
        }

    def _finalize_metadata(self, samples: int) -> Dict[str, object]:
        end_time = time.time()
        start_time = self.scan_metadata.get("start_time")
        duration = end_time - start_time if start_time else None

        expected = int(self.scan_metadata.get("requested_steps", 0))
        completed = int(self.scan_metadata.get("completed_steps", 0))
        tolerance = max(1, int(expected * 0.01)) if expected else 0
        difference = expected - completed

        plausibility = {
            "expected_steps": expected,
            "completed_steps": completed,
            "difference": difference,
            "tolerance": tolerance,
            "within_tolerance": abs(difference) <= tolerance,
        }

        history_deviation = self._update_history(duration, completed, samples)
        if history_deviation is not None:
            plausibility["historical_deviation"] = history_deviation

        summary = {
            "start_time": start_time,
            "end_time": end_time,
            "duration": duration,
            "samples": samples,
            "step_events": self.scan_metadata.get("step_events", []),
            "plausibility": plausibility,
            "stop_requested": self.scan_metadata.get("stop_requested", False),
        }

        summary_path = os.path.join(self.config.logs_dir, "scan_summary.json")
        save_summary(summary_path, summary)
        return summary

    def _update_history(self, duration: Optional[float], completed_steps: int, samples: int) -> Optional[float]:
        history_path = os.path.join(self.config.scans_root, "scan_history.json")
        history: List[Dict[str, object]]
        if os.path.exists(history_path):
            with open(history_path, "r", encoding="utf-8") as handle:
                history = json.load(handle)
        else:
            history = []

        entry = {
            "scan_id": getattr(self.config, "scan_id", "unknown"),
            "completed_steps": completed_steps,
            "samples": samples,
            "duration": duration,
        }
        history.append(entry)

        with open(history_path, "w", encoding="utf-8") as handle:
            json.dump(history, handle, indent=2)

        if len(history) <= 1:
            return None

        previous = history[:-1]
        valid_steps = [item["completed_steps"] for item in previous if item["completed_steps"]]
        if not valid_steps:
            return None

        avg_steps = mean(valid_steps)
        if avg_steps == 0:
            return None

        deviation = abs(completed_steps - avg_steps) / avg_steps
        return deviation

    # ------------------------------------------------------------------
    # cleanup
    # ------------------------------------------------------------------
    def _cleanup(self):
        if self.lidar is not None:
            self.lidar.close()
        if self.stepper is not None:
            self.stepper.close()
        self.config.relay_off()
