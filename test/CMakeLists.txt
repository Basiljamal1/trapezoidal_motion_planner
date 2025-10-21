set(GTEST_MIN_VERSION 1.12.1)

find_package(GTest ${GTEST_MIN_VERSION})

if(NOT GTEST_FOUND OR GTest_VERSION VERSION_LESS ${GTEST_MIN_VERSION})
  message(WARNING "GTest not found " ${GTEST_MIN_VERSION} ". Fetching GTest from source.")
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/58d77fa8070e8cec2dc1ed015d66b454c8d78850.zip # v1.12.1
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

find_package(fmt REQUIRED)
find_package(GTest REQUIRED)

add_subdirectory(planner_tests)
