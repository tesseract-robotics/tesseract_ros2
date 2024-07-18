cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
# tesseract
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_task_composer REQUIRED)
# tesseract_ros2
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_rosutils REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} SHARED
  src/tesseract_planning_server.cpp)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    tf2_ros::tf2_ros

    tesseract::tesseract_command_language
    tesseract::tesseract_common
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_motion_planners_simple
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_trajopt_ifopt
    tesseract::tesseract_motion_planners_ompl
    tesseract::tesseract_motion_planners_descartes
    tesseract::tesseract_task_composer
    tesseract::tesseract_task_composer_planning_nodes

    ${tesseract_msgs_TARGETS}
    tesseract_monitoring::tesseract_monitoring_environment
    tesseract_rosutils::tesseract_rosutils
  PRIVATE
    ${tf2_eigen_TARGETS}
)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(${PROJECT_NAME}_node src/tesseract_planning_server_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

# add_executable(${PROJECT_NAME}_raster_example examples/server_raster_example.cpp)
# target_link_libraries(${PROJECT_NAME}_raster_example PRIVATE tesseract::tesseract_command_language tesseract::tesseract_task_composer ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_raster_example PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_raster_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_raster_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
# target_include_directories(${PROJECT_NAME}_raster_example PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_raster_example SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})

# Mark libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark executables for installation
install(
 TARGETS
   ${PROJECT_NAME}_node
  #  ${PROJECT_NAME}_raster_example
 RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  tf2_ros
  tesseract_msgs
)
ament_package()
