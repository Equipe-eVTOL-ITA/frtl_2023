cmake_minimum_required(VERSION 3.8)
project(frtl_2023_fase1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)
find_package(frtl_2023_drone REQUIRED)
#find_package(frtl_2023_cv_utils REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)
#find_package(nlohmann_json CONFIG REQUIRED)

include_directories(include)

#target_include_directories(drone PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

#add_executable(fase1
#src/fase1.cpp
#)

#ament_target_dependencies(fase1
#  rclcpp
#  fsm
#  frtl_2023_drone
#  frtl_2023_cv_utils
#  Eigen3
#)

add_executable(fase1_safe
  src/fase1_safe.cpp
)

ament_target_dependencies(fase1_safe
  rclcpp
  fsm
  frtl_2023_drone
  #frtl_2023_cv_utils
  Eigen3
)

install(
  TARGETS
    

    fase1_safe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
