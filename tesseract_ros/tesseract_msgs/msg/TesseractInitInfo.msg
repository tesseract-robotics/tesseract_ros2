# TesseractInitType
uint8 SCENE_GRAPH=0
uint8 SCENE_GRAPH_SRDF_MODEL=1
uint8 URDF_STRING=2
uint8 URDF_STRING_SRDF_STRING=3
uint8 URDF_PATH=4
uint8 URDF_PATH_SRDF_PATH=5

# Specifies the type of constructor to use. Indicates which of the fields below have valid data
uint8 type


# SCENE_GRAPH
tesseract_msgs/SceneGraph scene_graph

# URDF_STRING or URDF_STRING_SRDF_STRING
string urdf_string
# URDF_STRING_SRDF_STRING
string srdf_string

# URDF_PATH or URDF_PATH_SRDF_PATH
string urdf_path
# URDF_PATH_SRDF_PATH
string srdf_path
