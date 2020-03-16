cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rviz VERSION 0.1.0)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(tesseract REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_rosutils REQUIRED)

find_package(Boost REQUIRED thread date_time system filesystem)
find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

# Ogre
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(OGRE OGRE)
#link_directories(${OGRE_LIBRARY_DIRS} )

# TODO: Find out if this still exists
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(rviz2_QT_VERSION 5)
message(WARNING QT5_Found: ${Qt5_FOUND})


# Qt Stuff
if(rviz2_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz2_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz2_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${QT_INCLUDE_DIR}
#  SYSTEM ${OGRE_INCLUDE_DIRS}
)

# Rviz Render Tools
qt_wrap_cpp(${PROJECT_NAME}_render_tools_cpp_MOCS
  include/tesseract_rviz/render_tools/trajectory_panel.h
  include/tesseract_rviz/render_tools/visualization_widget.h
  include/tesseract_rviz/render_tools/link_widget.h
  include/tesseract_rviz/render_tools/joint_widget.h
  include/tesseract_rviz/render_tools/joint_state_monitor_widget.h
  include/tesseract_rviz/render_tools/environment_widget.h
  include/tesseract_rviz/render_tools/trajectory_monitor_widget.h
#  include/tesseract_rviz/render_tools/manipulation_widget.h
  include/tesseract_rviz/interactive_marker/integer_action.h
#  include/tesseract_rviz/interactive_marker/interactive_marker.h
  include/tesseract_rviz/property/button_property.h
  )

add_library(${PROJECT_NAME}_render_tools SHARED
  src/render_tools/visualization_widget.cpp
  src/render_tools/joint_widget.cpp
  src/render_tools/link_widget.cpp
  src/render_tools/trajectory_panel.cpp
  src/render_tools/joint_state_monitor_widget.cpp
  src/render_tools/environment_widget.cpp
  src/render_tools/trajectory_monitor_widget.cpp
#  src/render_tools/manipulation_widget.cpp
  src/interactive_marker/integer_action.cpp
#  src/interactive_marker/interactive_marker.cpp
#  src/interactive_marker/interactive_marker_control.cpp
  src/markers/marker_base.cpp
  src/markers/marker_selection_handler.cpp
#  src/markers/utils.cpp
  src/markers/arrow_marker.cpp
  src/markers/sphere_marker.cpp
  src/markers/cube_marker.cpp
  src/markers/cylinder_marker.cpp
  src/markers/triangle_list_marker.cpp
  src/markers/text_view_facing_marker.cpp
  src/property/button_property.cpp
  ${${PROJECT_NAME}_render_tools_cpp_MOCS}
)
target_link_libraries(${PROJECT_NAME}_render_tools PUBLIC
  tesseract::tesseract
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering)
ament_target_dependencies(${PROJECT_NAME}_render_tools PUBLIC
  tesseract_msgs
  tesseract_rosutils
#  ${OGRE_LIBRARIES}
)

target_compile_options(${PROJECT_NAME}_render_tools PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_render_tools PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_render_tools PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_render_tools PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_render_tools SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
#    ${OGRE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

#add_library(${PROJECT_NAME}_scene_plugin_core
#  src/tesseract_scene_plugin/tesseract_scene_display.cpp
#  include/tesseract_rviz/tesseract_scene_plugin/tesseract_scene_display.h)
#ament_add_dependencies(${PROJECT_NAME}_scene_plugin_core ${PROJECT_NAME}_render_tools ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

#add_library(${PROJECT_NAME}_scene_plugin src/tesseract_scene_plugin/plugin_init.cpp)
#ament_add_dependencies(${PROJECT_NAME}_scene_plugin ${PROJECT_NAME}_scene_plugin_core ${Boost_LIBRARIES})

# Trajectory State Display
qt_wrap_cpp(${PROJECT_NAME}_state_plugin_core_cpp_MOCS include/tesseract_rviz/tesseract_state_plugin/tesseract_state_display.h)

add_library(${PROJECT_NAME}_state_plugin_core SHARED src/tesseract_state_plugin/tesseract_state_display.cpp ${${PROJECT_NAME}_state_plugin_core_cpp_MOCS})
target_link_libraries(${PROJECT_NAME}_state_plugin_core PUBLIC ${PROJECT_NAME}_render_tools)
target_compile_options(${PROJECT_NAME}_state_plugin_core PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_state_plugin_core PUBLIC -std=c++14)
else()
    target_compile_features(${PROJECT_NAME}_state_plugin_core PUBLIC cxx_std_14)
endif()
target_include_directories(${PROJECT_NAME}_state_plugin_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_state_plugin_core SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
#    ${OGRE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_state_plugin SHARED src/tesseract_state_plugin/plugin_init.cpp)
target_link_libraries(${PROJECT_NAME}_state_plugin PRIVATE ${PROJECT_NAME}_state_plugin_core)
target_compile_options(${PROJECT_NAME}_state_plugin PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_state_plugin PUBLIC -std=c++14)
else()
    target_compile_features(${PROJECT_NAME}_state_plugin PUBLIC cxx_std_14)
endif()
target_include_directories(${PROJECT_NAME}_state_plugin PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_state_plugin SYSTEM PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

## Trajectory Display
qt_wrap_cpp(${PROJECT_NAME}_trajectory_plugin_core_cpp_MOCS include/tesseract_rviz/tesseract_trajectory_plugin/tesseract_trajectory_display.h)

add_library(${PROJECT_NAME}_trajectory_plugin_core SHARED src/tesseract_trajectory_plugin/tesseract_trajectory_display.cpp ${${PROJECT_NAME}_trajectory_plugin_core_cpp_MOCS})
target_link_libraries(${PROJECT_NAME}_trajectory_plugin_core PUBLIC
  ${PROJECT_NAME}_render_tools
  ${PROJECT_NAME}_state_plugin_core
  tesseract::tesseract
#  ${OGRE_LIBRARIES}
  )
target_compile_options(${PROJECT_NAME}_trajectory_plugin_core PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_trajectory_plugin_core PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_trajectory_plugin_core PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_trajectory_plugin_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajectory_plugin_core SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
#    ${OGRE_INCLUDE_DIRS}
#    ${EIGEN3_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}_trajectory_plugin SHARED src/tesseract_trajectory_plugin/plugin_init.cpp)
target_link_libraries(${PROJECT_NAME}_trajectory_plugin PRIVATE
    ${PROJECT_NAME}_trajectory_plugin_core
    tesseract::tesseract
    )
target_compile_options(${PROJECT_NAME}_trajectory_plugin PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_trajectory_plugin PUBLIC -std=c++14)
else()
    target_compile_features(${PROJECT_NAME}_trajectory_plugin PUBLIC cxx_std_14)
endif()
target_include_directories(${PROJECT_NAME}_trajectory_plugin PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajectory_plugin SYSTEM PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
#    ${EIGEN3_INCLUDE_DIRS}
    )

### Manipulation Display
#qt_wrap_cpp(${PROJECT_NAME}_manipulation_plugin_core_cpp_MOCS include/tesseract_rviz/tesseract_manipulation_plugin/tesseract_manipulation_display.h)

#add_library(${PROJECT_NAME}_manipulation_plugin_core SHARED src/tesseract_manipulation_plugin/tesseract_manipulation_display.cpp ${${PROJECT_NAME}_manipulation_plugin_core_cpp_MOCS})
#ament_add_dependencies(${PROJECT_NAME}_manipulation_plugin_core PUBLIC
#    ${PROJECT_NAME}_render_tools
#    tesseract::tesseract
#    ${OGRE_LIBRARIES}
#    ${QT_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${EIGEN3_INCLUDE_DIRS})
#target_compile_options(${PROJECT_NAME}_manipulation_plugin_core PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
#if(CXX_FEATURE_FOUND EQUAL "-1")
#    target_compile_options(${PROJECT_NAME}_manipulation_plugin_core PUBLIC -std=c++11)
#else()
#    target_compile_features(${PROJECT_NAME}_manipulation_plugin_core PUBLIC cxx_std_11)
#endif()
#target_include_directories(${PROJECT_NAME}_manipulation_plugin_core PUBLIC
#    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#    "$<INSTALL_INTERFACE:include>")
#target_include_directories(${PROJECT_NAME}_manipulation_plugin_core SYSTEM PUBLIC
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#  "$<INSTALL_INTERFACE:include>"
#    ${Boost_INCLUDE_DIRS}
#    ${QT_INCLUDE_DIR}
#    ${OGRE_INCLUDE_DIRS})

#add_library(${PROJECT_NAME}_manipulation_plugin SHARED src/tesseract_manipulation_plugin/plugin_init.cpp)
#ament_add_dependencies(${PROJECT_NAME}_manipulation_plugin PRIVATE
#    ${PROJECT_NAME}_manipulation_plugin_core
#    tesseract::tesseract
#    ${Boost_LIBRARIES})
#target_compile_options(${PROJECT_NAME}_manipulation_plugin PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
#if(CXX_FEATURE_FOUND EQUAL "-1")
#    target_compile_options(${PROJECT_NAME}_manipulation_plugin PUBLIC -std=c++11)
#else()
#    target_compile_features(${PROJECT_NAME}_manipulation_plugin PUBLIC cxx_std_11)
#endif()
#target_include_directories(${PROJECT_NAME}_manipulation_plugin PRIVATE
#    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#    "$<INSTALL_INTERFACE:include>")
#target_include_directories(${PROJECT_NAME}_manipulation_plugin SYSTEM PRIVATE
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#  "$<INSTALL_INTERFACE:include>"
#    ${Boost_INCLUDE_DIRS}
#    ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_test src/test.cpp)
target_link_libraries(${PROJECT_NAME}_test PRIVATE ${PROJECT_NAME}_trajectory_plugin_core)
target_compile_options(${PROJECT_NAME}_test PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_test PUBLIC -std=c++14)
else()
    target_compile_features(${PROJECT_NAME}_test PUBLIC cxx_std_14)
endif()
target_include_directories(${PROJECT_NAME}_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_test SYSTEM PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")


# Mark executables and/or libraries for installation
install(TARGETS
    ${PROJECT_NAME}_render_tools
    ${PROJECT_NAME}_state_plugin_core
    ${PROJECT_NAME}_state_plugin
    ${PROJECT_NAME}_trajectory_plugin_core
    ${PROJECT_NAME}_trajectory_plugin
#    ${PROJECT_NAME}_manipulation_plugin_core
#    ${PROJECT_NAME}_manipulation_plugin
    EXPORT ${PACKAGE_NAME}-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark plugin XML files for installation
install(FILES tesseract_rviz_state_plugin_description.xml tesseract_rviz_trajectory_plugin_description.xml #tesseract_rviz_manipulation_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(rviz_common tesseract_rviz_state_plugin_description.xml)
pluginlib_export_plugin_description_file(rviz_common tesseract_rviz_trajectory_plugin_description.xml)
#pluginlib_export_plugin_description_file(rviz_common tesseract_rviz_manipulation_plugin_description.xml)

ament_export_dependencies(
    ament_cmake
    interactive_markers
    pluginlib
    rclcpp
    rviz_common
    rviz_rendering
    rviz_ogre_vendor
    tesseract
    tesseract_msgs
    tesseract_rosutils
    Boost
    console_bridge
    Eigen3)

ament_export_interfaces(${PACKAGE_NAME}-targets)
ament_package()
