cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

# ROS
find_package(resource_retriever REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
# tesseract
find_package(tesseract_qt REQUIRED)
# tesseract_ros2
find_package(tesseract_monitoring REQUIRED)

# Ogre
# find_package(OGRE REQUIRED)
link_directories(${OGRE_LIBRARY_DIRS})

# Qt Stuff
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()
macro(qt_wrap_cpp)
  qt5_wrap_cpp(${ARGN})
endmacro()

#add_definitions(-DQT_NO_KEYWORDS)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME}_markers SHARED
  src/markers/marker_base.cpp
  src/markers/marker_selection_handler.cpp
  src/markers/utils.cpp
  src/markers/arrow_marker.cpp
  src/markers/sphere_marker.cpp
  src/markers/cube_marker.cpp
  src/markers/cylinder_marker.cpp
  src/markers/triangle_list_marker.cpp
  src/markers/text_view_facing_marker.cpp
)
target_link_libraries(${PROJECT_NAME}_markers PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
)
target_compile_options(${PROJECT_NAME}_markers PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(${PROJECT_NAME}_markers PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_markers PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_compile_definitions(${PROJECT_NAME}_markers PUBLIC $ENV{ROS_DISTRO}_BUILD)

qt_wrap_cpp(${PROJECT_NAME}_interactive_marker_cpp_MOCS
  include/tesseract_rviz/interactive_marker/integer_action.h
  include/tesseract_rviz/interactive_marker/interactive_marker.h
)
add_library(${PROJECT_NAME}_interactive_marker SHARED
  src/interactive_marker/integer_action.cpp
  src/interactive_marker/interactive_marker.cpp
  src/interactive_marker/interactive_marker_control.cpp
  ${${PROJECT_NAME}_interactive_marker_cpp_MOCS}
)
target_link_libraries(${PROJECT_NAME}_interactive_marker PUBLIC
  ${PROJECT_NAME}_markers
  tesseract::tesseract_qt_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  ${QT_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_interactive_marker PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(${PROJECT_NAME}_interactive_marker PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_interactive_marker PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_interactive_marker SYSTEM PUBLIC
  ${QT_INCLUDE_DIR}
)
target_compile_definitions(${PROJECT_NAME}_interactive_marker PUBLIC $ENV{ROS_DISTRO}_BUILD)

#######################################
## New Panels using tesseract_widget ##
#######################################

# Environment Panel
qt_wrap_cpp(${PROJECT_NAME}_cpp_MOCS
  include/tesseract_rviz/rostopic_combo_box.h
  include/tesseract_rviz/environment_display.h
  include/tesseract_rviz/environment_monitor_properties.h
  include/tesseract_rviz/joint_trajectory_monitor_properties.h
  include/tesseract_rviz/workbench_display.h
  include/tesseract_rviz/set_theme_tool.h
)
add_library(${PROJECT_NAME} SHARED
  src/rostopic_combo_box.cpp
  src/environment_display.cpp
  src/environment_monitor_properties.cpp
  src/joint_trajectory_monitor_properties.cpp
  src/workbench_display.cpp
  src/conversions.cpp
  src/ros_contact_results_render_manager.cpp
  src/ros_scene_graph_render_manager.cpp
  src/set_theme_tool.cpp
  ${${PROJECT_NAME}_cpp_MOCS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  resource_retriever::resource_retriever
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering

  tesseract::tesseract_qt_common
  tesseract::tesseract_qt_environment_widgets
  tesseract::tesseract_qt_manipulation_widgets
  tesseract::tesseract_qt_workbench_widgets

  tesseract_monitoring::tesseract_monitoring_environment
  ${PROJECT_NAME}_markers
  ${PROJECT_NAME}_interactive_marker
  ${QT_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${QT_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_environment_plugin SHARED src/environment_plugin_init.cpp)
target_link_libraries(${PROJECT_NAME}_environment_plugin PRIVATE
  ${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME}_environment_plugin PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(${PROJECT_NAME}_environment_plugin PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_environment_plugin PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_workbench_plugin SHARED src/workbench_plugin_init.cpp)
target_link_libraries(${PROJECT_NAME}_workbench_plugin PRIVATE
  ${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME}_workbench_plugin PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(${PROJECT_NAME}_workbench_plugin PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_workbench_plugin PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Files generated by MOC, RCC, and UIC may produce clang-tidy warnings.
# We generate a dummy .clang-tidy file in the binary directory that disables
# all clang-tidy checks except one that will never match.  This one check is
# necessary; clang-tidy reports an error when no checks are enabled.
# Since the Qt code generators will generate source files in the binary tree,
# clang-tidy will load the configuration from this dummy file when the sources
# are built.
#file(WRITE "${tesseract_rviz_BINARY_DIR}/.clang-tidy" "
#---
#Checks: '-*,llvm-twine-local'
#...
#")

ament_export_targets(${PACKAGE_NAME}-targets)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Mark executables and/or libraries for installation
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_markers
    ${PROJECT_NAME}_interactive_marker
    ${PROJECT_NAME}_environment_plugin
    ${PROJECT_NAME}_workbench_plugin
  EXPORT ${PACKAGE_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Resource files for visualizations
install(DIRECTORY ogre_media DESTINATION share/${PROJECT_NAME})

# Mark plugin XML files for installation
install(
  FILES
    tesseract_rviz_environment_plugin_description.xml
    tesseract_rviz_workbench_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(rviz_common tesseract_rviz_environment_plugin_description.xml)
pluginlib_export_plugin_description_file(rviz_common tesseract_rviz_workbench_plugin_description.xml)

ament_package()
