import open3d as o3d
from lib.mesh_utils import mesh_from_poisson


def test_mesh_from_poisson_returns_mesh():
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    pcd = sphere.sample_points_uniformly(number_of_points=500)
    mesh = mesh_from_poisson(pcd, depth=4, k=20)
    assert isinstance(mesh, o3d.geometry.TriangleMesh)
    assert len(mesh.vertices) > 0
