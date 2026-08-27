cmake_minimum_required(VERSION 3.5)
project(g1_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


link_directories(src)

set (
  DEPENDENCY_LIST
  rclcpp
  std_msgs
  rosbag2_cpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  robot_localization
  sensor_msgs
  geographic_msgs
  tf2_geometry_msgs
  tf2
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PkgConfig REQUIRED)
#find_package(nav2_gps_waypoint_follower_demo REQUIRED)
#find_package(septentrio_gnss_driver REQUIRED)


include_directories(
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
)

pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

add_executable(ros2serial src/ros2serial.cpp)
ament_target_dependencies(ros2serial ${DEPENDENCY_LIST})

add_executable(gps_waypoint_follower src/gps_waypoint_follower.cpp)
ament_target_dependencies(gps_waypoint_follower ${DEPENDENCY_LIST})

add_executable(gps_waypoint_follower_recovery src/gps_waypoint_follower_recovery.cpp)
ament_target_dependencies(gps_waypoint_follower_recovery ${DEPENDENCY_LIST})


add_executable(set_datum src/set_datum.cpp)
ament_target_dependencies(set_datum ${DEPENDENCY_LIST})

add_executable(gps_odom_filter src/gps_odom_filter.cpp)
ament_target_dependencies(gps_odom_filter ${DEPENDENCY_LIST})

add_executable(lidar_transformer src/lidar_transformer.cpp)
ament_target_dependencies(lidar_transformer ${DEPENDENCY_LIST})

target_link_libraries(gps_waypoint_follower
  yaml-cpp)

target_link_libraries(gps_waypoint_follower_recovery
  yaml-cpp
   ${GEOGRAPHICLIB_LIBRARIES})


install(TARGETS 
        ros2serial
        gps_waypoint_follower
        gps_waypoint_follower_recovery
        set_datum
        gps_odom_filter
        lidar_transformer
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    src/bash_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()