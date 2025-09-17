import numpy as np
from lib.config import Config
from lib.pointcloud import ProcessedPointClouds, get_scan_dict, process_raw, save_raw_scan


def test_process_raw_returns_intensity_pointcloud(tmp_path):
    config = Config(scans_root=str(tmp_path))
    config.init(scan_id="testscan")
    config.set(False, "ENABLE_VERTEXCOLOUR")
    config.set(False, "ENABLE_FILTERING")

    z_angles = [0.0]
    cartesian = [
        np.column_stack(
            (
                np.linspace(0, 1000, 12),
                np.linspace(0, 1000, 12),
                np.linspace(0, 255, 12),
            )
        )
    ]

    raw_scan = get_scan_dict(z_angles, cartesian_list=cartesian)
    save_raw_scan(config.raw_path, raw_scan)

    result = process_raw(config, save=False)

    assert isinstance(result, ProcessedPointClouds)
    assert result.intensity is not None
    assert len(result.intensity.points) > 0
    assert result.intensity.colors.shape[0] == len(result.intensity.points)
    assert result.color is None
