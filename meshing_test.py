import os

import pytest
try:
    import open3d as o3d
except ModuleNotFoundError:
    pytest.skip("open3d not installed", allow_module_level=True)


from lib.visualization import visualize
from lib.mesh_utils import mesh_from_poisson
from lib.config import Config


config = Config()
scan_id = "240824-1230"
config.init(scan_id=scan_id)

pcd_path = config.pcd_path
if not os.path.exists(pcd_path):
    pytest.skip("sample data not available", allow_module_level=True)


pcd = o3d.io.read_point_cloud(config.pcd_path)

mesh = mesh_from_poisson(
    pcd,
    depth=config.get("MESH", "POISSON", "depth"),
    k=config.get("MESH", "POISSON", "k"),
    estimate_normals=config.get("MESH", "POISSON", "estimate_normals"),
    density_threshold=config.get("MESH", "POISSON", "density_threshold"),
)

visualize([pcd, mesh], unlit=True, point_size=2, point_colors="normal")
