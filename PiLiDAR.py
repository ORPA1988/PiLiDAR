"""Main entry point for a complete PiLiDAR capture run.

The goal of this script is to orchestrate every single step of the workflow:

1.  Position the hardware using the stepper motor.
2.  Capture HDR photographs with the Raspberry Pi camera.
3.  Record the raw lidar measurements and convert them into a point cloud.
4.  Fuse the optical data with the geometry and export easy to understand
    visualisations.

To make the project approachable for absolute beginners we accompany every
section with detailed comments.  The new simulation mode mirrors the behaviour
of the hardware which means that the full pipeline can be executed on any
computer without sensors attached – perfect for learning and testing.
"""

from __future__ import annotations

import os
from time import sleep
from typing import Dict, Optional

from lib.lidar_driver import Lidar
from lib.a4988_driver import A4988
from lib.config import Config, format_value
from lib.pointcloud import (
    create_fusion_visualizations,
    get_scan_dict,
    process_raw,
    save_raw_scan,
)
from lib.rpicam_utils import take_HDR_photo, estimate_camera_parameters
from lib.pano_utils import hugin_stitch
from lib.simulation import run_simulation_capture


ASCII_ART = r"""
 ____  _ _     _ ____    _    ____
|  _ \(_) |   (_)  _ \  / \  |  _ \
| |_) | | |   | | | | |/ _ \ | |_) |
|  __/| | |___| | |_| / ___ \|  _ <
|_|   |_|_____|_|____/_/   \_\_| \_\
"""


def print_intro(config: Config) -> None:
    """Display a friendly welcome message and show where the data will live."""

    print(ASCII_ART)
    print("\nStarte PiLiDAR Scan...\n")
    print("Alle Ergebnisse werden im folgenden Ordner gespeichert:")
    print(f" -> {config.scan_dir}\n")


def initialize_stepper(config: Config) -> Optional[A4988]:
    """Create the stepper driver or ``None`` when something goes wrong."""

    try:
        return A4988(
            config.get("STEPPER", "pins", "DIR_PIN"),
            config.get("STEPPER", "pins", "STEP_PIN"),
            config.get("STEPPER", "pins", "MS_PINS"),
            delay=config.get("STEPPER", "STEP_DELAY"),
            step_angle=config.get("STEPPER", "STEP_ANGLE"),
            microsteps=config.get("STEPPER", "MICROSTEPS"),
            gear_ratio=config.get("STEPPER", "GEAR_RATIO"),
        )
    except Exception as error:  # pragma: no cover - hardware specific
        print("\n[WARNUNG] Der Schrittmotor konnte nicht initialisiert werden:")
        print(f"{error}\n")
        return None


def initialize_lidar(config: Config, enable_lidar: bool) -> Optional[Lidar]:
    """Create the lidar driver only when explicitly enabled."""

    if not enable_lidar:
        return None

    try:
        return Lidar(config, visualization=None)
    except Exception as error:  # pragma: no cover - hardware specific
        print("\n[WARNUNG] Der LiDAR-Sensor konnte nicht gestartet werden:")
        print(f"{error}\n")
        return None


def capture_photos(config: Config, stepper: Optional[A4988]) -> bool:
    """Capture HDR photos around 360°.

    Returns ``True`` when the capture succeeded and at least one image was
    written to disk.  When ``False`` is returned the calling code should fall
    back to simulation data.
    """

    if stepper is None:
        return False

    try:
        print("Kalibriere Kamera...")
        exposure_time, gain, awb_gains = estimate_camera_parameters(config)
    except Exception as error:  # pragma: no cover - hardware specific
        print("\n[WARNUNG] Kamera konnte nicht kalibriert werden:")
        print(f"{error}\n")
        return False

    img_count = config.get("PANO", "IMGCOUNT")

    for index in range(img_count):
        current_angle = stepper.get_current_angle()
        formatted_angle = format_value(current_angle, config.get("ANGULAR_DIGITS"))

        print(f"\nFoto {index + 1}/{img_count} bei Winkel {formatted_angle}°")
        img_name = f"image_{formatted_angle}.jpg"
        img_path = os.path.join(config.img_dir, img_name)

        try:
            image_paths = take_HDR_photo(
                AEB=config.get("CAM", "AEB"),
                AEB_stops=config.get("CAM", "AEB_STOPS"),
                path=img_path,
                exposure_time=exposure_time,
                gain=gain,
                awbgains=awb_gains,
                denoise=config.get("CAM", "denoise"),
                sharpness=config.get("CAM", "sharpness"),
                saturation=config.get("CAM", "saturation"),
                save_raw=config.get("CAM", "raw"),
                blocking=True,
            )
        except Exception as error:  # pragma: no cover - hardware specific
            print("\n[WARNUNG] Fotoaufnahme fehlgeschlagen:")
            print(f"{error}\n")
            return False

        config.imglist.extend(image_paths)

        stepper.move_to_angle((360 / img_count) * (index + 1))
        sleep(0.5)

    stepper.move_to_angle(0)
    sleep(0.5)
    return len(config.imglist) > 0


def capture_lidar_scan(config: Config, stepper: Optional[A4988], lidar: Optional[Lidar]) -> bool:
    """Perform the lidar scan and store the raw data."""

    if stepper is None or lidar is None:
        return False

    def move_steps_callback() -> None:
        stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)
        lidar.z_angle = stepper.get_current_angle()

    if not config.get("ENABLE_CAM"):
        sleep(2)  # allow the lidar motor to stabilise

    try:
        print("\nStarte LiDAR-Scan...\n")
        lidar.read_loop(callback=move_steps_callback, max_packages=config.max_packages)
    except Exception as error:  # pragma: no cover - hardware specific
        print("\n[WARNUNG] LiDAR-Scan fehlgeschlagen:")
        print(f"{error}\n")
        return False

    stepper.move_to_angle(0)

    raw_scan = get_scan_dict(lidar.z_angles, cartesian_list=lidar.cartesian_list)
    save_raw_scan(lidar.raw_path, raw_scan)
    return True


def finalize_hardware(config: Config, lidar: Optional[Lidar], stepper: Optional[A4988]) -> None:
    """Gracefully release hardware resources."""

    print("\nPiLiDAR gestoppt\n")
    if lidar is not None:
        lidar.close()
    if stepper is not None:
        stepper.close()
    config.relay_off()


def main() -> None:
    config = Config()
    config.init()
    config.relay_on()
    print_intro(config)

    enable_cam = config.get("ENABLE_CAM")
    enable_lidar = config.get("ENABLE_LIDAR")
    enable_pano = config.get("ENABLE_PANO")
    enable_3d = config.get("ENABLE_3D")

    stepper = initialize_stepper(config)
    lidar = initialize_lidar(config, enable_lidar and not config.simulation_mode)

    hardware_success = True
    panorama_path: Optional[str] = None
    simulation_outputs: Dict[str, object] = {}

    try:
        if not config.simulation_mode and enable_cam:
            hardware_success &= capture_photos(config, stepper)

        if not config.simulation_mode and enable_lidar:
            hardware_success &= capture_lidar_scan(config, stepper, lidar)

    finally:
        finalize_hardware(config, lidar, stepper)

    should_simulate = config.simulation_mode or not hardware_success

    if should_simulate:
        print("\nSimulation wird ausgeführt, um Beispiel-Daten zu erzeugen...\n")
        simulation_outputs = run_simulation_capture(config)
        panorama_path = simulation_outputs.get("panorama")
    elif enable_cam and enable_pano and config.imglist:
        try:
            print("\nBerechne Panorama mit Hugin...")
            hugin_stitch(config)
            panorama_path = config.pano_path if os.path.exists(config.pano_path) else None
        except Exception as error:  # pragma: no cover - external dependency
            print("\n[WARNUNG] Panorama konnte nicht berechnet werden:")
            print(f"{error}\n")
            should_simulate = True
            simulation_outputs = run_simulation_capture(config)
            panorama_path = simulation_outputs.get("panorama")

    # Ensure we always know which panorama image to use.
    if panorama_path is None and os.path.exists(config.pano_path):
        panorama_path = config.pano_path

    pointcloud = None
    if enable_3d or should_simulate:
        print("\nErzeuge Punktwolke...")
        pointcloud = process_raw(config, save=True)

    if pointcloud is not None:
        print("\nErstelle Visualisierungen...")
        fusion_paths = create_fusion_visualizations(config, pointcloud, pano_path=panorama_path)
    else:
        fusion_paths = {}

    print("\nZusammenfassung der wichtigsten Dateien:")
    print(f"  Rohdaten:          {config.raw_path if os.path.exists(config.raw_path) else 'nicht vorhanden'}")
    print(f"  Punktwolke:        {config.pcd_path if os.path.exists(config.pcd_path) else 'nicht vorhanden'}")
    print(f"  Fusionsbild:       {fusion_paths.get('fusion', 'nicht erstellt')}")
    print(f"  LiDAR-Projektion:  {fusion_paths.get('lidar_projection', 'nicht erstellt')}\n")


if __name__ == "__main__":
    main()

