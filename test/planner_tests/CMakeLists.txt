set(TEST_NAME planner_tests)
add_compile_definitions(TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
# Executables
add_executable(
  ${TEST_NAME}
  src/TrapezoidalMotionPlannerTest.cpp
  src/MockSystems.cpp
)
set_target_properties(
  ${TEST_NAME}
  PROPERTIES
  CXX_STANDARD ${CMAKE_CXX_STANDARD}
  CXX_STANDARD_REQUIRED ${CMAKE_CXX_STANDARD_REQUIRED}
  CXX_EXTENSIONS NO
)
target_include_directories(
  ${TEST_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
  ${TEST_NAME} PRIVATE
  trapezoidal_motion_planner::trapezoidal_motion_planner
  GTest::gtest_main
  GTest::gmock_main
)
install(
  DIRECTORY include/
  DESTINATION include
)
target_compile_options(${TEST_NAME} PRIVATE -Wall -Wextra -g -Wunused-result -pthread)
include(GoogleTest)
gtest_discover_tests(${TEST_NAME})