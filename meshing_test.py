import open3d as o3d
from lib.visualization import visualize
from lib.mesh_utils import mesh_from_poisson 
from lib.config import Config


config = Config()
config.init(scan_id="240824-1230")


pcd = o3d.io.read_point_cloud(config.pcd_path)

mesh = mesh_from_poisson(pcd, 
                         depth             = config.get("MESH", "POISSON", "depth"), 
                         k                 = config.get("MESH", "POISSON", "k"), 
                         estimate_normals  = config.get("MESH", "POISSON", "estimate_normals"), 
                         density_threshold = config.get("MESH", "POISSON", "density_threshold"))

visualize([pcd, mesh], unlit=True, point_size=2, point_colors="normal")
