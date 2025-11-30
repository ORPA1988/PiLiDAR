import os
import glob
import numpy as np

from lib.config import Config
from lib.pointcloud_numpy import merge_2D_points
from lib.raw_utils import load_raw_scan


def latest_scan_dir(scans_root):
    dirs = sorted(
        [d for d in glob.glob(os.path.join(scans_root, '*')) if os.path.isdir(d)],
        reverse=True,
    )
    return dirs[0] if dirs else None


def stats(points_m):
    mins = np.min(points_m, axis=0)
    maxs = np.max(points_m, axis=0)
    centroid = np.mean(points_m, axis=0)
    return mins, maxs, centroid


def main():
    config = Config()
    config.init()

    scan_dir = latest_scan_dir(config.scans_root)
    if not scan_dir:
        print('No scans found.')
        return

    scan_id = os.path.basename(scan_dir)
    raw_path = os.path.join(scan_dir, f"{scan_id}{config.get('LIDAR','RAW_NAME')}")
    print(f"Using scan: {scan_id}")

    if not os.path.exists(raw_path):
        print(f"Raw scan not found: {raw_path}")
        return

    raw = load_raw_scan(raw_path)
    arr = merge_2D_points(
        raw,
        position_offset=(0, config.get('3D','Y_OFFSET'), 0),
        angle_offset=config.get('LIDAR','LIDAR_OFFSET_ANGLE'),
        up_vector=(0,0,1),
    )

    # Convert mm to m
    arr[:,2] += config.get('3D','Z_OFFSET')
    arr[:,:3] *= config.get('3D','SCALE')

    mins, maxs, centroid = stats(arr[:,:3])
    print(f"Bounds (m): X[{mins[0]:.3f},{maxs[0]:.3f}] Y[{mins[1]:.3f},{maxs[1]:.3f}] Z[{mins[2]:.3f},{maxs[2]:.3f}]")
    print(f"Centroid (m): [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")

    # Plausibility checks
    issues = []
    # Expect Z within reasonable room height (~ -1m to +2m after offset)
    if maxs[2] - mins[2] < 0.5:
        issues.append('Z range too small; check SCALE/units.')
    if abs(centroid[2]) > 2.0:
        issues.append('Z centroid far from 0; adjust Z_OFFSET.')
    # X/Y spread should be non-trivial
    if maxs[0] - mins[0] < 0.5 or maxs[1] - mins[1] < 0.5:
        issues.append('Horizontal spread too small; check rotation/angle offset.')

    if issues:
        print('Plausibility issues:')
        for i in issues:
            print('-', i)
    else:
        print('Plausibility: OK')

if __name__ == '__main__':
    main()
