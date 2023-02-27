cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(interface_pkg_deps
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  octomap_msgs
)

foreach(pkg ${interface_pkg_deps})
  find_package(${pkg} REQUIRED)
endforeach()

set(message_files
  "msg/AllowedCollisionEntry.msg"
  "msg/ChainGroup.msg"
  "msg/CollisionGeometry.msg"
  "msg/CollisionMarginData.msg"
  "msg/CollisionMarginOverrideType.msg"
  "msg/ContactManagersPluginInfo.msg"
  "msg/ContactMarginPair.msg"
  "msg/ContactResult.msg"
  "msg/ContactResultVector.msg"
  "msg/Environment.msg"
  "msg/EnvironmentCommand.msg"
  "msg/EnvironmentState.msg"
  "msg/Geometry.msg"
  "msg/GroupsJointState.msg"
  "msg/GroupsJointStates.msg"
  "msg/GroupsKinematicPlugins.msg"
  "msg/GroupsTCP.msg"
  "msg/GroupsTCPs.msg"
  "msg/Inertial.msg"
  "msg/Joint.msg"
  "msg/JointCalibration.msg"
  "msg/JointDynamics.msg"
  "msg/JointGroup.msg"
  "msg/JointLimits.msg"
  "msg/JointMimic.msg"
  "msg/JointSafety.msg"
  "msg/JointState.msg"
  "msg/JointTrajectory.msg"
  "msg/KinematicsInformation.msg"
  "msg/KinematicsPluginInfo.msg"
  "msg/Link.msg"
  "msg/LinkGroup.msg"
  "msg/Material.msg"
  "msg/Mesh.msg"
  "msg/ObjectColor.msg"
  "msg/OctreeSubType.msg"
  "msg/PlannerProfileRemapping.msg"
  "msg/PlanningRequest.msg"
  "msg/PlanningResponse.msg"
  "msg/PluginInfo.msg"
  "msg/PluginInfoContainer.msg"
  "msg/ProfileMap.msg"
  "msg/SceneGraph.msg"
  "msg/StringDoublePair.msg"
  "msg/StringLimitsPair.msg"
  "msg/StringPair.msg"
  "msg/StringPluginInfoPair.msg"
  "msg/TaskComposerNodeInfo.msg"
  "msg/Trajectory.msg"
  "msg/TransformMap.msg"
  "msg/VisualGeometry.msg"
)

set(service_files
  "srv/ComputeContactResultVector.srv"
  "srv/GetEnvironmentChanges.srv"
  "srv/GetEnvironmentInformation.srv"
  "srv/ModifyEnvironment.srv"
  "srv/SaveSceneGraph.srv"
)

set(action_files
  "action/GetMotionPlan.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  ${service_files}
  ${action_files}
  DEPENDENCIES ${interface_pkg_deps}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
