"""Utility helpers that generate simulated lidar and camera data.

The real PiLiDAR hardware is great for field work, yet for learning the
workflow it is convenient to be able to run everything on a normal laptop.  The
functions in this module create synthetic images and lidar measurements that
roughly mimic a very small room.  The output is fully compatible with the rest
of the pipeline which means the main script does not need to treat simulation
as a special case.
"""

from __future__ import annotations

import os
from typing import Dict, Iterable, List

import cv2
import numpy as np

from .config import format_value
from .pointcloud import get_scan_dict, save_raw_scan


def _simulation_value(settings: Dict[str, int], key: str, default: int) -> int:
    """Helper that returns a simulation setting with a sensible default."""

    return int(settings.get(key, default))


def generate_simulated_images(config) -> List[str]:
    """Create colourful demo images to mimic the panorama capture.

    The images intentionally contain gradients and big annotations so that
    beginners can immediately recognise in which order they were captured when
    looking at the panorama result later on.
    """

    img_width, img_height = config.simulation.get(
        "IMG_DIMS", config.get("CAM", "preview_dims")
    )
    img_width = int(img_width)
    img_height = int(img_height)

    num_images = config.get("PANO", "IMGCOUNT")
    angle_step = 360 / num_images

    image_paths: List[str] = []
    for index in range(num_images):
        angle = index * angle_step

        # Create a smooth gradient background that changes with the angle.
        x_gradient = np.linspace(0, 255, img_width, dtype=np.uint8)
        base = np.tile(x_gradient, (img_height, 1))
        image = np.dstack(
            [
                base,
                np.roll(base, shift=index * 10, axis=1),
                cv2.flip(base, 1),
            ]
        )

        # Overlay simple geometry to make the images visually distinct.
        centre = (img_width // 2, img_height // 2)
        colour = (
            int(80 + 175 * np.sin(np.deg2rad(angle))),
            int(80 + 175 * np.sin(np.deg2rad(angle + 120))),
            int(80 + 175 * np.sin(np.deg2rad(angle + 240))),
        )
        cv2.circle(image, centre, min(centre) - 10, colour, thickness=30)

        # Add a big annotation that explains what the user is seeing.
        title = f"Simulation {index + 1}/{num_images}"
        angle_text = f"Winkel: {angle:.1f}°"
        cv2.putText(
            image,
            title,
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            image,
            angle_text,
            (20, img_height - 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 0),
            2,
            lineType=cv2.LINE_AA,
        )

        formatted_angle = format_value(angle, config.get("ANGULAR_DIGITS"))
        filename = f"image_{formatted_angle}.jpg"
        filepath = os.path.join(config.img_dir, filename)
        cv2.imwrite(filepath, image)
        image_paths.append(filepath)

    config.imglist = image_paths
    return image_paths


def generate_simulated_panorama(image_paths: Iterable[str], config) -> str:
    """Concatenate the simulated photos to a simple panorama."""

    images = [cv2.imread(path) for path in image_paths]
    images = [img for img in images if img is not None]

    if not images:
        raise RuntimeError("No simulated images available for panorama creation")

    panorama = np.concatenate(images, axis=1)
    pano_width = config.get("PANO", "PANO_WIDTH")
    pano_height = pano_width // 2  # equirectangular panoramas are 2:1
    panorama = cv2.resize(panorama, (pano_width, pano_height), interpolation=cv2.INTER_AREA)
    cv2.imwrite(config.pano_path, panorama)
    return config.pano_path


def generate_simulated_lidar(config) -> Dict[str, Iterable]:
    """Create a synthetic lidar scan with gentle waves and intensity values."""

    settings = config.simulation
    num_planes = _simulation_value(settings, "PLANES", 90)
    points_per_plane = _simulation_value(settings, "POINTS_PER_PLANE", 240)

    z_angles = np.linspace(0, config.SCAN_ANGLE, num_planes, endpoint=True)
    cartesian_list = []

    for angle in z_angles:
        x = np.linspace(-400, 400, points_per_plane, dtype=np.float32)
        ripple = np.sin(np.linspace(0, 4 * np.pi, points_per_plane, dtype=np.float32) + np.deg2rad(angle))
        height = 250 + 150 * ripple
        intensity = ((ripple + 1) / 2 * 255).astype(np.float32)

        points = np.column_stack((x, height, intensity)).astype(np.float32)
        cartesian_list.append(points)

    raw_scan = get_scan_dict(list(z_angles), cartesian_list=cartesian_list)
    save_raw_scan(config.raw_path, raw_scan)
    return raw_scan


def run_simulation_capture(config) -> Dict[str, object]:
    """Generate a complete mock data set for one scanning run."""

    image_paths = generate_simulated_images(config)
    panorama_path = generate_simulated_panorama(image_paths, config)
    raw_scan = generate_simulated_lidar(config)

    return {
        "images": image_paths,
        "panorama": panorama_path,
        "raw_scan": raw_scan,
    }


__all__ = [
    "generate_simulated_images",
    "generate_simulated_panorama",
    "generate_simulated_lidar",
    "run_simulation_capture",
]

