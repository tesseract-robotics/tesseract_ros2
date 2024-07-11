cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

find_package(std_srvs REQUIRED)
find_package(tesseract_examples REQUIRED)
find_package(tesseract_monitoring REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

macro(add_example test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(${test_name}
    ${std_srvs_TARGETS}
    tesseract::tesseract_examples
    tesseract_monitoring::tesseract_monitoring_environment)
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${test_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${test_name} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
endmacro()

add_example(${PROJECT_NAME}_basic_cartesian_example_node src/basic_cartesian_example_node.cpp)
add_example(${PROJECT_NAME}_car_seat_example_node src/car_seat_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_hybrid_example_node src/freespace_hybrid_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_ompl_example_node src/freespace_ompl_example_node.cpp)
add_example(${PROJECT_NAME}_glass_upright_example_node src/glass_upright_example_node.cpp)
#TODO: Enable in tesseract_planning first
#add_example(${PROJECT_NAME}_glass_upright_ompl_example_node src/glass_upright_ompl_example_node.cpp)
add_example(${PROJECT_NAME}_online_planning_example_node src/online_planning_example_node.cpp)
add_example(${PROJECT_NAME}_pick_and_place_example_node src/pick_and_place_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node src/puzzle_piece_auxillary_axes_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_example_node src/puzzle_piece_example_node.cpp)
add_example(${PROJECT_NAME}_scene_graph_example_node src/scene_graph_example_node.cpp)

# Mark executables for installation
install(
  TARGETS
    ${PROJECT_NAME}_basic_cartesian_example_node
    ${PROJECT_NAME}_car_seat_example_node
    ${PROJECT_NAME}_freespace_hybrid_example_node
    ${PROJECT_NAME}_freespace_ompl_example_node
    ${PROJECT_NAME}_glass_upright_example_node
#    ${PROJECT_NAME}_glass_upright_ompl_example_node
    ${PROJECT_NAME}_online_planning_example_node
    ${PROJECT_NAME}_pick_and_place_example_node
    ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node
    ${PROJECT_NAME}_puzzle_piece_example_node
    ${PROJECT_NAME}_scene_graph_example_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
