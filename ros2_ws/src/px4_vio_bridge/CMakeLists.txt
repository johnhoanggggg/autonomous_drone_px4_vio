cmake_minimum_required(VERSION 3.8)
project(px4_vio_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/basalt_odometry_test
  scripts/basalt_rtabmap_slam_ros2
  scripts/battery_to_ros
  scripts/global_planner_monitor
  scripts/global_planner_sim
  scripts/route_follower_monitor
  scripts/offboard_hold_yaw
  scripts/offboard_global_planner
  scripts/offboard_hover
  scripts/offboard_square
  scripts/offboard_vfh
  scripts/offboard_waypoint
  scripts/process_monitor
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
  ament_add_pytest_test(test_grid_planner test/test_grid_planner.py)
  ament_add_pytest_test(test_rtabmap_grid test/test_rtabmap_grid.py)
  ament_add_pytest_test(test_path_follower test/test_path_follower.py)
  ament_add_pytest_test(test_offboard_hover test/test_offboard_hover.py)
  ament_add_pytest_test(test_planner_flight test/test_planner_flight.py)
  ament_add_pytest_test(test_process_monitor test/test_process_monitor.py)
  ament_add_pytest_test(test_process_singleton test/test_process_singleton.py)
  ament_add_pytest_test(test_offboard_square test/test_offboard_square.py)
  ament_add_pytest_test(test_offboard_vfh test/test_offboard_vfh.py)
  ament_add_pytest_test(test_offboard_waypoint test/test_offboard_waypoint.py)
  ament_add_pytest_test(test_vfh2d test/test_vfh2d.py)
  ament_add_pytest_test(test_vfh_telemetry test/test_vfh_telemetry.py)
  ament_add_pytest_test(test_vio_to_px4_odometry test/test_vio_to_px4_odometry.py)
endif()

ament_package()
