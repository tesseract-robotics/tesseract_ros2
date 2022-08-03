cmake_minimum_required(VERSION 3.5)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
set(ament_pkg_deps
  rclcpp
  ament_index_cpp
  tesseract_monitoring
  tesseract_rosutils
#  tesseract_planning_server
  # octomap_ros
  # pcl_conversions
)

foreach(ament_pkg IN LISTS ament_pkg_deps)
    find_package(${ament_pkg} REQUIRED)
endforeach()

find_package(tesseract_environment REQUIRED)
#find_package(tesseract_command_language REQUIRED)
#find_package(tesseract_motion_planners REQUIRED)
#find_package(tesseract_process_managers REQUIRED)
#find_package(tesseract_common REQUIRED)
find_package(tesseract_examples REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
# find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

macro(add_example test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(${test_name}
    tesseract::tesseract_examples
    tesseract::tesseract_environment
    tesseract_monitoring::tesseract_monitoring_environment)
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${test_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${test_name} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
endmacro()

add_example(${PROJECT_NAME}_puzzle_piece_example_node src/puzzle_piece_example_node.cpp)

# add_library(basic_cartesian_example src/basic_cartesian_example.cpp# )
# target_link_libraries(basic_cartesian_example PUBLIC
#   tesseract::tesseract_environment
#   tesseract::tesseract_process_managers
#   ${PCL_LIBRARIES})
# target_compile_options(basic_cartesian_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(basic_cartesian_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(basic_cartesian_example PUBLIC VERSION 17)
# foreach(DEF ${PCL_DEFINITIONS})
#   string(STRIP ${DEF} DEF)
#   if (NOT "${DEF}" STREQUAL "")
#     string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
#     if ("${DEF_PREFIX}" STREQUAL "-m")
#       string(REPLACE " " ";" DEF ${DEF})
#       foreach(OPTION_DEF ${DEF})
#         target_compile_options(basic_cartesian_example PUBLIC ${OPTION_DEF})
#       endforeach()
#     else()
#       target_compile_definitions(basic_cartesian_example PUBLIC ${DEF})
#     endif()
#   endif()
# endforeach()
# target_include_directories(basic_cartesian_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(basic_cartesian_example SYSTEM PUBLIC
#     ${PCL_INCLUDE_DIRS})
# ament_target_dependencies(basic_cartesian_example PUBLIC ${ament_pkg_deps})

# add_executable(basic_cartesian_example_node src/basic_cartesian_example_node.cpp)
# target_link_libraries(basic_cartesian_example_node PRIVATE basic_cartesian_example)
# target_compile_options(basic_cartesian_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(basic_cartesian_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(basic_cartesian_example_node PUBLIC VERSION 17)
# target_include_directories(basic_cartesian_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_library(freespace_ompl_example src/freespace_ompl_example.cpp)
# target_link_libraries(freespace_ompl_example PUBLIC
#   tesseract::tesseract_environment
#   tesseract::tesseract_process_managers)
# target_compile_options(freespace_ompl_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(freespace_ompl_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(freespace_ompl_example PUBLIC VERSION 17)
# target_include_directories(freespace_ompl_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# ament_target_dependencies(freespace_ompl_example PUBLIC ${ament_pkg_deps})

# add_executable(freespace_ompl_example_node src/freespace_ompl_example_node.cpp)
# target_link_libraries(freespace_ompl_example_node PRIVATE freespace_ompl_example)
# target_compile_options(freespace_ompl_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(freespace_ompl_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(freespace_ompl_example_node PRIVATE VERSION 17)
# target_include_directories(freespace_ompl_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_library(freespace_hybrid_example src/freespace_hybrid_example.cpp)
# target_link_libraries(freespace_hybrid_example PUBLIC
#    tesseract::tesseract_environment
#    tesseract::tesseract_process_managers)
# target_compile_options(freespace_hybrid_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(freespace_hybrid_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(freespace_hybrid_example PUBLIC VERSION 17)
# target_include_directories(freespace_hybrid_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# ament_target_dependencies(freespace_hybrid_example PUBLIC ${ament_pkg_deps})

# add_executable(freespace_hybrid_example_node src/freespace_hybrid_example_node.cpp)
# target_link_libraries(freespace_hybrid_example_node PRIVATE freespace_hybrid_example)
# target_compile_options(freespace_hybrid_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(freespace_hybrid_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(freespace_hybrid_example_node PRIVATE VERSION 17)
# target_include_directories(freespace_hybrid_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_library(car_seat_example src/car_seat_example.cpp)
# target_link_libraries(car_seat_example PUBLIC
#     tesseract::tesseract_environment
#     tesseract::tesseract_process_managers)
# target_compile_options(car_seat_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(car_seat_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(car_seat_example PUBLIC VERSION 17)
# target_include_directories(car_seat_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_executable(car_seat_example_node src/car_seat_example_node.cpp)
# target_link_libraries(car_seat_example_node PRIVATE car_seat_example)
# target_compile_options(car_seat_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(car_seat_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(car_seat_example_node PRIVATE VERSION 17)
# target_include_directories(car_seat_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_library(glass_upright_example src/glass_upright_example.cpp)
# target_link_libraries(glass_upright_example PUBLIC
#     tesseract::tesseract_environment
#     tesseract::tesseract_process_managers)
# target_compile_options(glass_upright_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(glass_upright_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(glass_upright_example PUBLIC VERSION 17)
# target_include_directories(glass_upright_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_executable(glass_upright_example_node src/glass_upright_example_node.cpp)
# target_link_libraries(glass_upright_example_node PRIVATE glass_upright_example)
# target_compile_options(glass_upright_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(glass_upright_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(glass_upright_example_node PRIVATE VERSION 17)
# target_include_directories(glass_upright_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  add_library(${PROJECT_NAME}_glass_upright_ompl_example src/glass_upright_ompl_example.cpp)
#  target_link_libraries(${PROJECT_NAME}_glass_upright_ompl_example tesseract::tesseract_environment trajopt::trajopt tesseract::tesseract_motion_planners_ompl tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${catkin_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
#  target_clang_tidy(${PROJECT_NAME}_glass_upright_ompl_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#  target_cxx_version(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC VERSION 17)
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example SYSTEM PUBLIC
#      ${catkin_INCLUDE_DIRS})

#  add_executable(${PROJECT_NAME}_glass_upright_ompl_example_node src/glass_upright_ompl_example_node.cpp)
#  target_link_libraries(${PROJECT_NAME}_glass_upright_ompl_example_node ${PROJECT_NAME}_glass_upright_ompl_example ${catkin_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_glass_upright_ompl_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#  target_clang_tidy(${PROJECT_NAME}_glass_upright_ompl_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#  target_cxx_version(${PROJECT_NAME}_glass_upright_ompl_example_node PUBLIC VERSION 17)
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example_node PRIVATE
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example_node SYSTEM PRIVATE
#      ${catkin_INCLUDE_DIRS})
#endif()

# add_library(online_planning_example src/online_planning_example.cpp)
# target_link_libraries(online_planning_example PUBLIC
#     tesseract::tesseract_environment
#     trajopt::trajopt_ifopt
#     trajopt::trajopt_sqp)
# target_compile_options(online_planning_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(online_planning_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(online_planning_example PUBLIC VERSION 17)
# target_include_directories(online_planning_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_executable(online_planning_example_node src/online_planning_example_node.cpp)
# target_link_libraries(online_planning_example_node PRIVATE online_planning_example)
# target_compile_options(online_planning_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(online_planning_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(online_planning_example_node PRIVATE VERSION 17)
# target_include_directories(online_planning_example_node PRIVATE
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")

# add_library(pick_and_place_example src/pick_and_place_example.cpp)
# target_link_libraries(pick_and_place_example PUBLIC
#     tesseract::tesseract_environment
#     tesseract::tesseract_process_managers)
# target_compile_options(pick_and_place_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(pick_and_place_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(pick_and_place_example PUBLIC VERSION 17)
# target_include_directories(pick_and_place_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_executable(pick_and_place_example_node src/pick_and_place_example_node.cpp)
# target_link_libraries(pick_and_place_example_node PRIVATE pick_and_place_example)
# target_compile_options(pick_and_place_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(pick_and_place_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(pick_and_place_example_node PRIVATE VERSION 17)
# target_include_directories(pick_and_place_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

#add_library(puzzle_piece_example src/puzzle_piece_example.cpp)
#target_link_libraries(puzzle_piece_example PUBLIC
#  ament_index_cpp::ament_index_cpp
#  tesseract::tesseract_environment
#  tesseract::tesseract_process_managers
#  tesseract_monitoring::tesseract_monitoring_environment
#  tesseract_planning_server::tesseract_planning_server
#  tesseract::tesseract_examples
#)
#target_compile_options(puzzle_piece_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
#target_clang_tidy(puzzle_piece_example
#  ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#target_cxx_version(puzzle_piece_example PUBLIC VERSION 17)
#target_include_directories(puzzle_piece_example PUBLIC
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#  "$<INSTALL_INTERFACE:include>")

#add_executable(puzzle_piece_example_node src/puzzle_piece_example_node.cpp)
#target_link_libraries(puzzle_piece_example_node PUBLIC puzzle_piece_example)
#target_compile_options(puzzle_piece_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#target_clang_tidy(puzzle_piece_example_node
#  ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#target_cxx_version(puzzle_piece_example_node PRIVATE VERSION 17)
#target_include_directories(puzzle_piece_example_node PRIVATE
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#  "$<INSTALL_INTERFACE:include>")

# add_library(puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example.cpp)
# target_link_libraries(puzzle_piece_auxillary_axes_example PUBLIC
#     tesseract::tesseract_environment
#     tesseract::tesseract_motion_planners_trajopt
#     tesseract::tesseract_process_managers)
# target_compile_options(puzzle_piece_auxillary_axes_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(puzzle_piece_auxillary_axes_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(puzzle_piece_auxillary_axes_example PUBLIC VERSION 17)
# target_include_directories(puzzle_piece_auxillary_axes_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_executable(puzzle_piece_auxillary_axes_example_node src/puzzle_piece_auxillary_axes_example_node.cpp)
# target_link_libraries(puzzle_piece_auxillary_axes_example_node PUBLIC puzzle_piece_auxillary_axes_example)
# target_compile_options(puzzle_piece_auxillary_axes_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(puzzle_piece_auxillary_axes_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(puzzle_piece_auxillary_axes_example_node PRIVATE VERSION 17)
# target_include_directories(puzzle_piece_auxillary_axes_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_library(scene_graph_example src/scene_graph_example.cpp)
# target_link_libraries(scene_graph_example PUBLIC
#     tesseract::tesseract_environment
#     tesseract::tesseract_motion_planners_trajopt
#     jsoncpp_lib)
# target_compile_options(scene_graph_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(scene_graph_example
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(scene_graph_example PUBLIC VERSION 17)
# target_include_directories(scene_graph_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# add_executable(scene_graph_example_node src/scene_graph_example_node.cpp)
# target_link_libraries(scene_graph_example_node PUBLIC scene_graph_example)
# target_compile_options(scene_graph_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(scene_graph_example_node
#   ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
#   ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(scene_graph_example_node PRIVATE VERSION 17)
# target_include_directories(scene_graph_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
 )

## Mark libraries for installation
#install(
#  TARGETS
#    # basic_cartesian_example
#    # car_seat_example
#    # freespace_hybrid_example
#    # freespace_ompl_example
#    # glass_upright_example
#    # online_planning_example
#    # pick_and_place_example
#    ${PROJECT_NAME}_puzzle_piece_example_node
#    # puzzle_piece_auxillary_axes_example
#    # scene_graph_example
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)
# Mark executables for installation
install(
  TARGETS
    # basic_cartesian_example_node
    # car_seat_example_node
    # freespace_hybrid_example_node
    # freespace_ompl_example_node
    # glass_upright_example_node
    # online_planning_example_node
    # pick_and_place_example_node
    ${PROJECT_NAME}_puzzle_piece_example_node
    # puzzle_piece_auxillary_axes_example_node
    # scene_graph_example_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  install(
#    TARGETS
#      ${PROJECT_NAME}_glass_upright_ompl_example
#      ${PROJECT_NAME}_glass_upright_ompl_example_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.test"
)

ament_package()

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(GTest REQUIRED)

    add_rostest_gtest(${PROJECT_NAME}_basic_cartesian_example_unit test/basic_cartesian_example_unit.test test/basic_cartesian_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_basic_cartesian_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_car_seat_example_unit test/car_seat_example_unit.test test/car_seat_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_car_seat_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_car_seat_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_car_seat_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_car_seat_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_glass_upright_example_unit test/glass_upright_example_unit.test test/glass_upright_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_glass_upright_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_glass_upright_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_glass_upright_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_glass_upright_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_glass_upright_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_glass_upright_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_glass_upright_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_example_unit test/puzzle_piece_example_unit.test test/puzzle_piece_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit test/puzzle_piece_auxillary_axes_example_unit.test test/puzzle_piece_auxillary_axes_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_pick_and_place_example_unit test/pick_and_place_example_unit.test test/pick_and_place_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_pick_and_place_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_pick_and_place_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_pick_and_place_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_scene_graph_example_unit test/scene_graph_example_unit.test test/scene_graph_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_scene_graph_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_scene_graph_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_scene_graph_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_scene_graph_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_scene_graph_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_scene_graph_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_scene_graph_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})
endif()

