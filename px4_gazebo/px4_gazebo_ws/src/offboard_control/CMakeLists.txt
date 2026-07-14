cmake_minimum_required(VERSION 3.8)
project(offboard_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED) 
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(traj_utils REQUIRED)

add_executable(go src/go.cpp)
add_executable(circle src/circle.cpp)
add_executable(odom_world src/odom_world.cpp)
add_executable(cloud_converter_node src/cloud_converter.cpp)
add_executable(bspline_px4_bridge src/bspline_px4_bridge.cpp)
add_executable(fastlio_cloud_converter src/fastlio_cloud_converter.cpp)


ament_target_dependencies(fastlio_cloud_converter  rclcpp sensor_msgs)
ament_target_dependencies(go rclcpp px4_msgs)
ament_target_dependencies(circle rclcpp px4_msgs)
ament_target_dependencies(odom_world  rclcpp  px4_msgs  nav_msgs  tf2_ros  geometry_msgs  sensor_msgs  tf2)
ament_target_dependencies(bspline_px4_bridge  rclcpp  px4_msgs  nav_msgs  traj_utils  geometry_msgs)

include_directories(${PCL_INCLUDE_DIRS})

ament_target_dependencies(
  cloud_converter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
)


target_link_libraries(cloud_converter_node
  ${PCL_LIBRARIES}
)


install(TARGETS go circle odom_world cloud_converter_node bspline_px4_bridge fastlio_cloud_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
