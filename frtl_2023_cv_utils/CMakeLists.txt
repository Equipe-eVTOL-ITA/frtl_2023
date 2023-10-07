cmake_minimum_required(VERSION 3.8)
project(frtl_2023_cv_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# define a library target called my_value_converter_library
add_library(frtl_2023_cv_utils_lib SHARED
  src/ColorBoxFilter.cpp
  src/ContourExtractor.cpp
  src/EllipseFitter.cpp
  src/ImageToWorldConverter.cpp
  src/PointCloudTracker.cpp
  src/PolygonIdentifier.cpp
  src/qrcode_reader.cpp
)
ament_target_dependencies(frtl_2023_cv_utils_lib OpenCV Eigen3)
target_link_libraries(frtl_2023_cv_utils_lib nlohmann_json)
ament_export_targets(frtl_2023_cv_utils_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV Eigen3 nlohmann_json)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/cv_utils
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
 
install(
  TARGETS frtl_2023_cv_utils_lib
  EXPORT frtl_2023_cv_utils_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
