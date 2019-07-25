cmake_minimum_required(VERSION 3.5.0)
project(tesseract_monitoring VERSION 0.1.0 LANGUAGES CXX)

find_package(tesseract REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_17 CXX_FEATURE_FOUND)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Tesseract ROS Nodes
add_executable(${PROJECT_NAME}_contacts_node src/contact_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_contacts_node PRIVATE
    tesseract::tesseract
    tesseract::tesseract_rosutils
    ${rclcpp_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${Boost_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME}_contacts_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
set_target_properties(${PROJECT_NAME}_contacts_node PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_contacts_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_contacts_node SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_contacts_node)

add_library(${PROJECT_NAME}_environment SHARED
#    src/environment_monitor_node.cpp
    src/environment_monitor.cpp
    src/current_state_monitor.cpp
    )
target_link_libraries(${PROJECT_NAME}_environment PUBLIC
    ${rclcpp_LIBRARIES}
    tesseract::tesseract
    tesseract::tesseract_rosutils
    tesseract::tesseract_collision_core
    ${Boost_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${pluginlib_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME}_environment PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
set_target_properties(${PROJECT_NAME}_environment PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_environment PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_environment SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    )
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_environment)

# TODO Joe: Migrate demo
#add_executable(demo_scene demos/demo_scene.cpp)
#target_link_libraries(demo_scene PRIVATE
#    ${PROJECT_NAME}_environment
#    ${catkin_LIBRARIES})
#target_compile_options(demo_scene PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
#if(CXX_FEATURE_FOUND EQUAL "-1")
#    target_compile_options(demo_scene PRIVATE -std=c++11)
#else()
#    target_compile_features(demo_scene PRIVATE cxx_std_11)
#endif()
#target_include_directories(demo_scene PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
#target_include_directories(demo_scene SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

# TODO Joe: Migrate this node(?)
add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_environment_node PRIVATE
    ${PROJECT_NAME}_environment)
target_compile_options(${PROJECT_NAME}_environment_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
set_target_properties(${PROJECT_NAME}_environment_node PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_environment_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
#target_include_directories(${PROJECT_NAME}_environment_node SYSTEM PUBLIC ${rclcpp_INCLUDE_DIRS})
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_environment_node)

ament_export_dependencies(
    roscpp
    tesseract_msgs
    tesseract_rosutils
    pluginlib
    dynamic_reconfigure
    tf2_ros
    tf2_eigen
    EIGEN3
    tesseract
    orocos_kdl)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME}_environment)

ament_package()

install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark executables and/or libraries for installation
#install(TARGETS
#    ${PROJECT_NAME}_environment
#    ${PROJECT_NAME}_contacts_node
#    ${PROJECT_NAME}_environment_node
##    demo_scene
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

install(TARGETS
   ${PROJECT_NAME}_environment
   ${PROJECT_NAME}_environment_node
   DESTINATION lib/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY launch
  DESTINATION share
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}
#  DESTINATION include
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
# )

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
