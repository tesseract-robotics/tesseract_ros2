# Define geometries

uint8 SPHERE=1
uint8 CYLINDER=2
uint8 CONE=3
uint8 BOX=4
uint8 PLANE=5
uint8 MESH=6
uint8 CONVEX_MESH=7
uint8 SDF_MESH=8
uint8 OCTREE=9
uint8 CAPSULE=10
uint8 POLYGON_MESH=11
uint8 COMPOUND_MESH=12

# The type of the geometry
uint8 type

# The UUID of the geometry
string uuid

# Sphere
float64 sphere_radius

# CYLINDER
float64[2] cylinder_dimensions

# CAPSULE
float64[2] capsule_dimensions

# CONE (radius, length)
float64[2] cone_dimensions

# BOX (x, y, z) lengths
float64[3] box_dimensions

# PLANE
float64[4] plane_coeff

# MESH, CONVEX_MESH, SDF_MESH, POLYGON_MESH
tesseract_msgs/Mesh mesh

# OCTREE
octomap_msgs/Octomap octomap
tesseract_msgs/OctreeSubType octomap_sub_type

# COMPOUND_MESH (MESH, CONVEX_MESH, SDF_MESH, POLYGON_MESH)
tesseract_msgs/Mesh[] compound_mesh
uint8 compound_mesh_type #(MESH, CONVEX_MESH, SDF_MESH, POLYGON_MESH)
