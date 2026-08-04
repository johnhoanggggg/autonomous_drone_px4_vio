"""Launch the observation-only global A* monitor, optionally with a simulator."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def recorder_exited(event, _context):
    # ros2 bag refuses to overwrite an existing directory. Do not let the
    # planner continue and give the operator an apparently successful but
    # unrecorded run when the recorder failed at startup.
    if event.returncode in (0, -2, -15):
        return []
    return [
        LogInfo(msg=f"ERROR: bag recorder exited with code {event.returncode}; stopping this run"),
        EmitEvent(event=Shutdown(reason="bag recorder failed")),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("simulate", default_value="false"),
        DeclareLaunchArgument("dynamic_obstacle", default_value="true"),
        DeclareLaunchArgument("rate_hz", default_value="2.0"),
        DeclareLaunchArgument("map_timeout", default_value="3.0"),
        DeclareLaunchArgument("pose_timeout", default_value="1.0"),
        DeclareLaunchArgument("occupied_threshold", default_value="65"),
        DeclareLaunchArgument("robot_radius", default_value="0.30"),
        DeclareLaunchArgument("safety_margin", default_value="0.10"),
        DeclareLaunchArgument("inflation_extra", default_value="0.20"),
        DeclareLaunchArgument("heuristic_weight", default_value="1.0"),
        DeclareLaunchArgument("planning_timeout_ms", default_value="100.0"),
        DeclareLaunchArgument("route_follower", default_value="true"),
        DeclareLaunchArgument("follower_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("lookahead", default_value="0.60"),
        DeclareLaunchArgument("max_carrot_speed", default_value="0.25"),
        DeclareLaunchArgument("max_carrot_acceleration", default_value="0.50"),
        DeclareLaunchArgument("max_cross_track", default_value="0.60"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument("bag_output", default_value="flight_logs/global_planner_monitor"),
    ]
    simulator = Node(
        package="px4_vio_bridge",
        executable="global_planner_sim",
        name="global_planner_sim",
        output="screen",
        condition=IfCondition(LaunchConfiguration("simulate")),
        parameters=[{"dynamic_obstacle": typed("dynamic_obstacle", bool)}],
    )
    monitor = Node(
        package="px4_vio_bridge",
        executable="global_planner_monitor",
        name="global_planner_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "rate_hz": typed("rate_hz", float),
            "map_timeout": typed("map_timeout", float),
            "pose_timeout": typed("pose_timeout", float),
            "occupied_threshold": typed("occupied_threshold", int),
            "robot_radius": typed("robot_radius", float),
            "safety_margin": typed("safety_margin", float),
            "inflation_extra": typed("inflation_extra", float),
            "heuristic_weight": typed("heuristic_weight", float),
            "planning_timeout_ms": typed("planning_timeout_ms", float),
        }],
    )
    follower = Node(
        package="px4_vio_bridge",
        executable="route_follower_monitor",
        name="route_follower_monitor",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("route_follower")),
        parameters=[{
            "rate_hz": typed("follower_rate_hz", float),
            "lookahead": typed("lookahead", float),
            "max_carrot_speed": typed("max_carrot_speed", float),
            "max_carrot_acceleration": typed("max_carrot_acceleration", float),
            "max_cross_track": typed("max_cross_track", float),
        }],
    )
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--storage", "mcap",
            "--storage-preset-profile", "fastwrite",
            "--disable-keyboard-controls",
            "--topics",
            "/rtabmap/grid", "/rtabmap/pose", "/rtabmap/vio_pose",
            "/vio/map_correction", "/vio/map_correction_target", "/waypoint/clicked",
            "/planner/path", "/planner/candidate_path", "/planner/inflated_map",
            "/planner/markers", "/planner/status", "/planner/planning_ms",
            "/planner/path_length", "/planner/expanded_cells",
            "/planner/follower/carrot", "/planner/follower/lookahead",
            "/planner/follower/displacement", "/planner/follower/status",
            "/planner/follower/progress", "/planner/follower/remaining",
            "/planner/follower/path_progress",
            "/planner/follower/cross_track", "/planner/follower/path_generation",
            "/planner/follower/markers",
            "--output", LaunchConfiguration("bag_output"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    recorder_watchdog = RegisterEventHandler(
        OnProcessExit(target_action=recorder, on_exit=recorder_exited)
    )
    return LaunchDescription(args + [
        LogInfo(msg="global planner monitor is observation only; it cannot command PX4"),
        recorder_watchdog, recorder, simulator, monitor, follower,
    ])
