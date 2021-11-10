cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rosutils VERSION 0.1.0 LANGUAGES CXX)

# ROS packages
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Non-ROS packages
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets
find_package(tesseract_collision REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} SHARED src/plotting.cpp src/conversions.cpp src/utils.cpp)
target_link_libraries(${PROJECT_NAME}
    tesseract::tesseract_environment
    tesseract::tesseract_scene_graph
    tesseract::tesseract_srdf
    tesseract::tesseract_geometry
    tesseract::tesseract_collision_core
    tesseract::tesseract_visualization
    tesseract::tesseract_common
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_process_managers
    tesseract::tesseract_command_language
    ${rclcpp_LIBRARIES}
    ${ament_index_cpp_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
    tesseract_msgs
    rclcpp
    tf2
    tf2_eigen
    tf2_ros
    geometry_msgs
    yaml_cpp_vendor
    std_msgs
)

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)
  find_package(tesseract_support REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_unit test/tesseract_rosutils_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit
    GTest::GTest
    GTest::Main
    tesseract::tesseract_support
    ${PROJECT_NAME}
  )
  target_clang_tidy(${PROJECT_NAME}_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_unit PUBLIC VERSION 17)
  target_include_directories(${PROJECT_NAME}_unit PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PROJECT_NAME}_unit SYSTEM PRIVATE
      ${EIGEN3_INCLUDE_DIRS})
endif()

ament_export_dependencies(
    tesseract_msgs
    rclcpp
    tesseract_environment
    tesseract_scene_graph
    tesseract_srdf
    tesseract_geometry
    tesseract_collision
    tesseract_visualization
    tesseract_common
    tesseract_motion_planners
    tesseract_process_managers
    tesseract_command_language
    yaml_cpp_vendor
)

ament_export_include_directories(
    include
)

ament_package()

