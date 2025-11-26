cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

# ROS
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# tesseract
find_package(tesseract_common REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_visualization REQUIRED)
# tesseract_ros2
find_package(tesseract_msgs REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()
add_library(${PROJECT_NAME} SHARED src/plotting.cpp src/conversions.cpp src/utils.cpp)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    ${trajectory_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tesseract_msgs_TARGETS}
    tesseract::tesseract_common
    tesseract::tesseract_task_composer
    tesseract::tesseract_visualization
    tesseract::tesseract_motion_planners_core
  PRIVATE
    ament_index_cpp::ament_index_cpp
    ${tf2_eigen_TARGETS}
    ${octomap_msgs_TARGETS}
)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  trajectory_msgs
  visualization_msgs
  tesseract_msgs
  tesseract_task_composer
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
