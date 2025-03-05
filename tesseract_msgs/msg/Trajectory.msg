# Header
std_msgs/Header header

# The namespace the trajectory is associated with
# This is mostly used by visualization tools
string ns

# (Optional) Override the existing Tesseract Environment
Environment environment

# (Optional, ignored if environment is provided) Additional Commands to be applied to environment prior to trajectory visualization
tesseract_msgs/EnvironmentCommand[] commands

# (Required if no environment provided, otherwise ignored) Initial Environment Joint State
tesseract_msgs/StringDoublePair[] initial_state

############################################
# One of the following options is required #
############################################

# Tesseract Command Language Instructions
string instructions

# (Ignored if instructions are provided) Joint Trajectory
string joint_trajectories_uuid
string joint_trajectories_description
tesseract_msgs/JointTrajectory[] joint_trajectories
