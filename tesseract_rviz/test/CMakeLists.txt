find_gtest()
find_package(rviz_ogre_vendor REQUIRED)

add_executable(${PROJECT_NAME}_conversions_unit tesseract_rviz_conversions_unit.cpp)
target_link_libraries(${PROJECT_NAME}_conversions_unit
  GTest::GTest
  GTest::Main
  Eigen3::Eigen
  tesseract::common
  tesseract::geometry
  tesseract::scene_graph
  tesseract::tesseract_qt_common
  rviz_rendering::rviz_rendering
  ${PROJECT_NAME}
)
# Ogre resolves a bare plugin name through dlopen, which never searches the vendored plugin
# directory, so the test loads the render system by full path.
target_compile_definitions(${PROJECT_NAME}_conversions_unit PRIVATE TESSERACT_RVIZ_OGRE_PLUGIN_DIR="${OGRE_PLUGIN_DIR}")
target_compile_options(${PROJECT_NAME}_conversions_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_conversions_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_conversions_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_conversions_unit)
