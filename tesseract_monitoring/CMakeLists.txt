cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)

find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_rosutils REQUIRED)

#generate_dynamic_reconfigure_options(
#cfg/EnvironmentMonitorDynamicReconfigure.cfg
#)

#catkin_package(
#INCLUDE_DIRS
#include
#LIBRARIES
#${PROJECT_NAME}_environment
#${PROJECT_NAME}_environment_interface
#${PROJECT_NAME}_contacts
#CATKIN_DEPENDS
#roscpp
#tesseract_msgs
#tesseract_rosutils
#pluginlib
#dynamic_reconfigure
#tf2_ros
#tf2_eigen
#visualization_msgs
#DEPENDS
#EIGEN3
#orocos_kdl
#tesseract_common
#tesseract_environment
#)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# Tesseract ROS Nodes
add_library(${PROJECT_NAME}_environment SHARED
  # src/environment_monitor.cpp
  src/current_state_monitor.cpp
)
target_link_libraries(${PROJECT_NAME}_environment PUBLIC tesseract::tesseract_environment ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_environment ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_environment PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_environment PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_environment SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_environment PUBLIC
  tesseract_msgs
  tesseract_rosutils
  pluginlib
)

# add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
# target_link_libraries(${PROJECT_NAME}_environment_node PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_environment_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_environment_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_environment_node PRIVATE VERSION 17)
# target_include_directories(${PROJECT_NAME}_environment_node PUBLIC
#   " $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")

# add_library(${PROJECT_NAME}_contacts src/contact_monitor.cpp)
# target_link_libraries(${PROJECT_NAME}_contacts PRIVATE tesseract::tesseract_environment ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_contacts PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_contacts ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_contacts PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_contacts PRIVATE
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# add_dependencies(${PROJECT_NAME}_contacts ${PROJECT_NAME}_gencfg)

# add_executable(${PROJECT_NAME}_contacts_node src/contact_monitor_node.cpp)
# target_link_libraries(${PROJECT_NAME}_contacts_node PRIVATE tesseract::tesseract_environment ${PROJECT_NAME}_contacts ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_contacts_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_contacts_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_contacts_node PRIVATE VERSION 17)
# target_include_directories(${PROJECT_NAME}_contacts_node PRIVATE
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_environment_interface SHARED src/environment_monitor_interface.cpp)
target_link_libraries(${PROJECT_NAME}_environment_interface PUBLIC
  tesseract::tesseract_environment)
target_compile_options(${PROJECT_NAME}_environment_interface PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_environment_interface ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_environment_interface PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_environment_interface PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME}_environment_interface PUBLIC
  tesseract_msgs
  tesseract_rosutils
)

# add_executable(demo_scene demos/demo_scene.cpp)
# target_link_libraries(demo_scene PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
# target_compile_options(demo_scene PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(demo_scene ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(demo_scene PRIVATE VERSION 17)
# target_include_directories(demo_scene PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")

# Mark libraries for installation
install(
  TARGETS
    # ${PROJECT_NAME}_environment
    ${PROJECT_NAME}_environment_interface
    # ${PROJECT_NAME}_contacts
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mark executables for installation
install(
  TARGETS
  # ${PROJECT_NAME}_contacts_node
  # ${PROJECT_NAME}_environment_node
  # demo_scene
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
