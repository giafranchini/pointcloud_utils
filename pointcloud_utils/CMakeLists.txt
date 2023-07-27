cmake_minimum_required(VERSION 3.8)
project(pointcloud_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(elevator_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

set(dependencies
  rclcpp
  elevator_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
  tf2
  pcl_ros
)

include_directories(include
  ${EIGEN3_INCLUDE_DIR})

add_library(tf_utils SHARED
  src/tf_utils.cpp
)
ament_target_dependencies(tf_utils ${dependencies})

add_library(point_types_custom SHARED
  src/point_types_custom.cpp
)
ament_target_dependencies(point_types_custom ${dependencies})

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

install(TARGETS
  tf_utils point_types_custom
  EXPORT pointcloud_utilsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}" ${EIGEN3_INCLUDE_DIR})
ament_export_targets(pointcloud_utilsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} PCL)
ament_package()
