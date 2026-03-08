find_gtest()

add_executable(${PROJECT_NAME}_conversions_unit tesseract_rviz_conversions_unit.cpp)
target_link_libraries(${PROJECT_NAME}_conversions_unit 
  GTest::GTest 
  GTest::Main 
  Eigen3::Eigen 
  tesseract::tesseract_geometry 
  tesseract::tesseract_scene_graph
  tesseract::tesseract_qt_common
  rviz_rendering::rviz_rendering
  ${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME}_conversions_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_cxx_version(${PROJECT_NAME}_conversions_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_conversions_unit)
