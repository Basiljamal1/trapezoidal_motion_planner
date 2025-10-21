cmake_minimum_required(VERSION 3.10)
set(CMAKE_PROJECT_VERSION 0.0.1)
project(trapezoidal_motion_planner
  VERSION ${CMAKE_PROJECT_VERSION})

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options("$<$<CONFIG:RELEASE>:-O3>"
    "$<$<CONFIG:RELEASE>:-fpic>"
    "$<$<CONFIG:RELEASE>:-ftree-vectorize>"
    "$<$<CONFIG:DEBUG>:-O0;-g;-fpic>")
  add_compile_definitions("$<$<CONFIG:DEBUG>:DEBUG>" "$<$<CONFIG:RELEASE>:NDEBUG>")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PACKAGE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_library(${PROJECT_NAME}
  SHARED
  src/TrapezoidalMotionPlanner.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  "FMT_HEADER_ONLY"
)

# INSTALL
include(GNUInstallDirs)
set(INSTALL_CMAKE_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)
install(EXPORT ${PROJECT_NAME}Targets
  FILE
  ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

if (NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING ON)
endif()

# # Testing
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
