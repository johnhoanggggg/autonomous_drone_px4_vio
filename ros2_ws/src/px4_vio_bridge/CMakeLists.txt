cmake_minimum_required(VERSION 3.8)
project(px4_vio_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/basalt_odometry_test
  scripts/basalt_rtabmap_slam_ros2
  scripts/battery_to_ros
  scripts/map_correction
  scripts/offboard_hold_yaw
  scripts/offboard_hover
  scripts/offboard_square
  scripts/offboard_vfh
  scripts/offboard_waypoint
  scripts/px4_local_position_to_ros
  scripts/vfh_monitor
  scripts/vio_to_px4_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_battery_to_ros test/test_battery_to_ros.py)
  ament_add_pytest_test(test_map_correction test/test_map_correction.py)
  ament_add_pytest_test(test_offboard_hover test/test_offboard_hover.py)
  ament_add_pytest_test(test_offboard_square test/test_offboard_square.py)
  ament_add_pytest_test(test_offboard_vfh test/test_offboard_vfh.py)
  ament_add_pytest_test(test_offboard_waypoint test/test_offboard_waypoint.py)
  ament_add_pytest_test(test_vfh2d test/test_vfh2d.py)
  ament_add_pytest_test(test_vfh_telemetry test/test_vfh_telemetry.py)
  ament_add_pytest_test(test_vio_to_px4_odometry test/test_vio_to_px4_odometry.py)
endif()

ament_package()
