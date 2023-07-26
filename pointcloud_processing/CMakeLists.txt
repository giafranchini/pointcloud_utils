cmake_minimum_required(VERSION 3.15)
project(pointcloud_processing)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(PCL 1.12.1 REQUIRED QUIET COMPONENTS core features filters io segmentation surface)
find_package(rclcpp REQUIRED)
find_package(pointcloud_utils REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenMP QUIET)
find_package(native_adapters REQUIRED)

if(OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DGRID_MAP_PCL_OPENMP_FOUND=${OpenMP_FOUND})
endif()

set(dependencies 
  rclcpp
  pointcloud_utils
  common_interfaces
  ament_cmake
  pcl_ros
  pcl_conversions
  native_adapters
  Boost
  geometry2
  geometry_msgs
  tf2_eigen
  tf2_ros
  tf2
  tf2_geometry_msgs
  sensor_msgs
  rclcpp_components
  )

include_directories(
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## COMPILE

add_library(pointcloud_merge SHARED
  src/pointcloud_merge.cpp
)

add_library(pointcloud_transform SHARED
  src/pointcloud_transform.cpp
)

# DEPENDENCIES

ament_target_dependencies(
  pointcloud_transform
  ${dependencies}
)

ament_target_dependencies(
  pointcloud_merge
  ${dependencies}
)

# LINK ADDITIONAL LIBRARIES

target_link_libraries(pointcloud_transform ${PCL_LIBRARIES})
target_link_libraries(pointcloud_merge ${PCL_LIBRARIES})

# REGISTER COMPONENTS

rclcpp_components_register_node(pointcloud_transform PLUGIN "pointcloud_processing::PointCloudTransform" EXECUTABLE pointcloud_transform_node)
rclcpp_components_register_node(pointcloud_merge PLUGIN "pointcloud_processing::PointCloudMerge" EXECUTABLE pointcloud_merge_node)

set(target_libs
  pointcloud_transform
  pointcloud_merge
  )
set(target_execs
  pointcloud_transform_node
  pointcloud_merge_node)  

# INSTALL
install(
  TARGETS ${target_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${target_execs}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


# EXPORTS
ament_export_include_directories("include${PROJECT_NAME}")
ament_export_libraries(${target_libs})
ament_export_dependencies(${dependencies})

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

ament_package()
