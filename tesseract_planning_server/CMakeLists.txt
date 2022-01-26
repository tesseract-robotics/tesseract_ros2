cmake_minimum_required(VERSION 3.12)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# find_package(Boost REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_managers REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} SHARED src/tesseract_planning_server.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
    rclcpp::rclcpp
    tesseract::tesseract_command_language
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_process_managers
    tesseract_monitoring::tesseract_monitoring_environment
    tesseract_rosutils::tesseract_rosutils
    tf2_ros::tf2_ros
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC tesseract_msgs tf2_ros tf2_eigen)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${tf2_eigen_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_archive_viewer_node src/archive_viewer_node.cpp)
# target_link_libraries(${PROJECT_NAME}_archive_viewer_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_archive_viewer_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_archive_viewer_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_archive_viewer_node PRIVATE VERSION 17)
# target_include_directories(${PROJECT_NAME}_archive_viewer_node PUBLIC
    # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    # "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_archive_viewer_node SYSTEM PUBLIC
    # ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_node src/tesseract_planning_server_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION 17)
# target_include_directories(${PROJECT_NAME}_node PUBLIC
    # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    # "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC
    # ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_raster_example examples/server_raster_example.cpp)
# target_link_libraries(${PROJECT_NAME}_raster_example PRIVATE tesseract::tesseract_command_language tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_raster_example PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_raster_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_raster_example PRIVATE VERSION 17)
# target_include_directories(${PROJECT_NAME}_raster_example PUBLIC
    # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    # "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_raster_example SYSTEM PUBLIC
    # ${catkin_INCLUDE_DIRS})

# Libraries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executables
#install(
#  TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_raster_example
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  tesseract_monitoring
  tesseract_msgs
  tesseract_rosutils
  tf2_ros
  tf2_eigen
  # Boost
  # Eigen3
  tesseract_common
  tesseract_command_language
  tesseract_motion_planners
  tesseract_process_managers
)
ament_package()
