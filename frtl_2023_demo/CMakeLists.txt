cmake_minimum_required(VERSION 3.8)
project(frtl_2023_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)
find_package(frtl_2023_drone REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(takeoff_and_land
  src/examples/takeoff_and_land.cpp
)

#target_include_directories(takeoff_and_land PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

ament_target_dependencies(takeoff_and_land
  rclcpp
  fsm
  frtl_2023_drone
  Eigen3
)

add_executable(circle
  src/examples/circle.cpp
)

ament_target_dependencies(circle
  rclcpp
  fsm
  frtl_2023_drone
  Eigen3
)


install(
  TARGETS
    takeoff_and_land
    circle
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
