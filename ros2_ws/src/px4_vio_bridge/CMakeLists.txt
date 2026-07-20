cmake_minimum_required(VERSION 3.8)
project(px4_vio_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/basalt_odometry_test
  scripts/basalt_rtabmap_slam_ros2
  scripts/offboard_hold_yaw
  scripts/offboard_hover
  scripts/px4_local_position_to_ros
  scripts/vio_to_px4_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
