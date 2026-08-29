cmake_minimum_required(VERSION 3.8)
project(px4_vio_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# ROS-free flight geometry: the exact counterpart of grid_planner.py's
# clearance test, path_follower.py's polyline and planner_flight.py's command
# limiters. Kept separate from the node so the parity tests can link it alone.
add_library(planner_flight_cpp
  src/grid_clearance.cpp
  src/path_geometry.cpp
  src/route_follower.cpp
  src/command_limiter.cpp
  src/grid_planner.cpp
)
target_compile_features(planner_flight_cpp PUBLIC cxx_std_17)
target_include_directories(planner_flight_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cpp_clearance_shadow
  src/cpp_clearance_shadow.cpp
)
target_compile_features(cpp_clearance_shadow PUBLIC cxx_std_17)
target_link_libraries(cpp_clearance_shadow planner_flight_cpp)
target_link_libraries(cpp_clearance_shadow nlohmann_json::nlohmann_json)
ament_target_dependencies(cpp_clearance_shadow
  geometry_msgs nav_msgs px4_msgs rclcpp std_msgs
)

# Replay harnesses driven by the cross-language parity tests.
add_executable(planner_flight_replay src/planner_flight_replay.cpp)
target_compile_features(planner_flight_replay PUBLIC cxx_std_17)
target_link_libraries(planner_flight_replay planner_flight_cpp nlohmann_json::nlohmann_json)

add_executable(grid_planner_replay src/grid_planner_replay.cpp)
target_compile_features(grid_planner_replay PUBLIC cxx_std_17)
target_link_libraries(grid_planner_replay planner_flight_cpp nlohmann_json::nlohmann_json)

add_executable(route_follower_replay src/route_follower_replay.cpp)
target_compile_features(route_follower_replay PUBLIC cxx_std_17)
target_link_libraries(route_follower_replay planner_flight_cpp nlohmann_json::nlohmann_json)

# The observation-only C++ A* planner: the counterpart of
# global_planner_monitor.py, sharing its singleton lock so only one can run.
add_executable(cpp_astar_planner src/global_planner_node.cpp)
target_compile_features(cpp_astar_planner PUBLIC cxx_std_17)
target_link_libraries(cpp_astar_planner planner_flight_cpp nlohmann_json::nlohmann_json)
ament_target_dependencies(cpp_astar_planner
  geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs
)

# Observation-only route follower, selected by cpp_nodes / cpp_follower.
add_executable(cpp_route_follower src/route_follower_node.cpp)
target_compile_features(cpp_route_follower PUBLIC cxx_std_17)
target_link_libraries(cpp_route_follower planner_flight_cpp nlohmann_json::nlohmann_json)
ament_target_dependencies(cpp_route_follower
  geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs
)

# The commanding C++ flight adapter: the full offboard sequencer.
add_executable(cpp_flight_adapter
  src/offboard_hover_node.cpp
  src/offboard_global_planner_node.cpp
)
target_compile_features(cpp_flight_adapter PUBLIC cxx_std_17)
target_include_directories(cpp_flight_adapter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(cpp_flight_adapter planner_flight_cpp)
target_link_libraries(cpp_flight_adapter nlohmann_json::nlohmann_json)
ament_target_dependencies(cpp_flight_adapter
  geometry_msgs nav_msgs px4_msgs rclcpp std_msgs
)

install(TARGETS
  planner_flight_cpp
  cpp_astar_planner
  cpp_clearance_shadow
  cpp_flight_adapter
  cpp_route_follower
  grid_planner_replay
  planner_flight_replay
  route_follower_replay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_grid_clearance_cpp test/test_grid_clearance_cpp.cpp)
  target_link_libraries(test_grid_clearance_cpp planner_flight_cpp)
  ament_add_gtest(test_planner_flight_cpp test/test_planner_flight_cpp.cpp)
  target_link_libraries(test_planner_flight_cpp planner_flight_cpp)
  ament_add_gtest(test_route_follower_cpp test/test_route_follower_cpp.cpp)
  target_link_libraries(test_route_follower_cpp planner_flight_cpp)
  ament_add_gtest(test_grid_planner_cpp test/test_grid_planner_cpp.cpp)
  target_link_libraries(test_grid_planner_cpp planner_flight_cpp)
  ament_add_pytest_test(test_planner_flight_parity test/test_planner_flight_parity.py)
  ament_add_pytest_test(test_route_follower_parity test/test_route_follower_parity.py)
  ament_add_pytest_test(test_grid_planner_parity test/test_grid_planner_parity.py)
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
