sudo apt-get install meshlab  # installs MeshLab 2020.09
sudo apt-get install mesa-utils libgl1-mesa-glx libgl1-mesa-dri

################################################################
# REQUIRES OPENGL - NOT WORKING ON RASPERRY PI IN CONSOLE MODE #
################################################################

# MESHING USING POISSON RECONSTRUCTION
meshlabserver -l meshlab/log_mesh.txt -i scans/241007-2101/241007-2101.ply -o scans/241007-2101/meshed.ply -l x -s meshlab/meshing.mlx

# MAP UV TEXTURE
# export to PLY with vertex normals and wedge texture coordinates
meshlabserver -l meshlab/log_map.txt -p box.mlp -i scans/241007-2101/meshed.ply -o scans/241007-2101/textured.ply -m vn wt -s mapping.mlx
