cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rosutils VERSION 0.1.0 LANGUAGES CXX)

# ROS packages
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Non-ROS packages
find_package(Eigen3 REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_process_planners REQUIRED)
find_package(visualization_msgs REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_17 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
    tesseract::tesseract_scene_graph
    tesseract::tesseract_geometry
    tesseract::tesseract_collision_core
    tesseract::tesseract_visualization
    tesseract::tesseract_common
    tesseract::tesseract_process_planners
    Eigen3::Eigen
    ${rclcpp_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${ament_index_cpp_LIBRARIES}
    ${tf2_eigen_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} INTERFACE -std=c++17)
else()
    target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
endif()
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    )
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${rclcpp_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    ${ament_index_cpp_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}

    )

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

ament_export_dependencies(
    EIGEN3
    tesseract_msgs
    visualization_msgs
    rclcpp
    tesseract_scene_graph
    tesseract_geometry
    tesseract_visualization
    tesseract_collision
    tesseract_common
    tesseract_process_planners
    )

ament_export_include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    )

ament_package()

install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)
  find_package(tesseract_support REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_unit test/tesseract_rosutils_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main tesseract::tesseract_support ${PROJECT_NAME})
	if(CXX_FEATURE_FOUND EQUAL "-1")
            target_compile_options(${PROJECT_NAME}_unit PRIVATE -std=c++17)
	else()
            target_compile_features(${PROJECT_NAME}_unit PRIVATE cxx_std_17)
	endif()
  target_include_directories(${PROJECT_NAME}_unit PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
endif()
