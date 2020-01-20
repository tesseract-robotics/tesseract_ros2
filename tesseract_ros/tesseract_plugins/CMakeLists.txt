cmake_minimum_required(VERSION 3.5.0)
project(tesseract_plugins VERSION 0.1.0 LANGUAGES CXX)

find_package(tesseract_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

add_library(tesseract_collision_bullet_plugin src/plugins/bullet_plugin.cpp)
target_link_libraries(tesseract_collision_bullet_plugin PRIVATE
    tesseract::tesseract_collision_core
    tesseract::tesseract_collision_bullet
    Eigen3::Eigen
    ${rclcpp_LIBRARIES}
    ${tf2_eigen_LIBRARIES}
    ${pluginlib_LIBRARIES}
    )
target_compile_options(tesseract_collision_bullet_plugin PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(tesseract_collision_bullet_plugin PRIVATE -std=c++11)
else()
    target_compile_features(tesseract_collision_bullet_plugin PRIVATE cxx_std_11)
endif()
target_include_directories(tesseract_collision_bullet_plugin SYSTEM PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    )


add_library(tesseract_collision_fcl_plugin src/plugins/fcl_plugin.cpp)
target_link_libraries(tesseract_collision_fcl_plugin PRIVATE
    tesseract::tesseract_collision_core
    tesseract::tesseract_collision_fcl
    ${rclcpp_LIBRARIES}
    ${tf2_eigen_LIBRARIES}
    ${pluginlib_LIBRARIES}
    )
target_compile_options(tesseract_collision_fcl_plugin PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(tesseract_collision_fcl_plugin PRIVATE -std=c++11)
else()
    target_compile_features(tesseract_collision_fcl_plugin PRIVATE cxx_std_11)
endif()
target_include_directories(tesseract_collision_fcl_plugin SYSTEM PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    )

ament_export_dependencies(
    tesseract_msgs
    roscpp
    eigen_conversions
    pluginlib
    EIGEN3
    tesseract_collision
    )

ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})

ament_package()

# Mark executables and/or libraries for installation
install(TARGETS tesseract_collision_bullet_plugin tesseract_collision_fcl_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES tesseract_collision_bullet_plugin_description.xml tesseract_collision_fcl_plugin_description.xml
  DESTINATION share
)
