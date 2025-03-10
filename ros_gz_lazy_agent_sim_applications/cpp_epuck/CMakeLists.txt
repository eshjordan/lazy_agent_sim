cmake_minimum_required(VERSION 3.22.0)
project(cpp_epuck)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME} src/cpp_epuck.cpp)

# Configure any tests
if(BUILD_TESTING)
  ## Testing
  ament_auto_find_test_dependencies()

  # Create the test executable
  ament_auto_add_gmock(${PROJECT_NAME}_unit_test
    test/test_BaseRobotCommsModel.cpp
  )
endif()

ament_auto_package()
