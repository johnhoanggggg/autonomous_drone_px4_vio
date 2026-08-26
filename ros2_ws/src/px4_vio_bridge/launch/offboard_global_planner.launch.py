"""Launch the reviewed position-only global-planner PX4 adapter and flight bag.

Start the SLAM/PX4 stack and global_planner_monitor.launch.py first. Select a
short valid route and confirm `/planner/follower/valid: true` before launching
this file. `auto_arm` defaults false and must remain false for the props-off run.

Yaw follows the commanded path heading by default: the vehicle turns onto each
leg before translating along it, so the route is flown forward instead of
sideways. Set `yaw_follows_heading:=false` to restore the fixed takeoff yaw.
"""

from datetime import datetime, timezone
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from px4_vio_bridge.log_paths import timestamped_bag


STORAGE_PRESET = "fastwrite"


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def write_run_marker(context, *args, **kwargs):
    bag_output = LaunchConfiguration("bag_output").perform(context)
    marker = Path(bag_output + ".launchinfo")
    try:
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.write_text(
            f"launch_file: {Path(__file__).resolve()}\n"
            f"storage_preset: {STORAGE_PRESET}\n"
            f"bag_output: {bag_output}\n"
            f"started_utc: {datetime.now(timezone.utc).isoformat()}\n"
        )
    except OSError as exc:
        return [LogInfo(msg=f"could not write run marker: {exc}")]
    return [LogInfo(msg=f"Run marker: {marker}")]


def generate_launch_description():
    default_bag = timestamped_bag("offboard_global")
    arguments = [
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("hover_height", default_value="0.40"),
        DeclareLaunchArgument("climb_timeout", default_value="15.0"),
        DeclareLaunchArgument("max_flight_time", default_value="45.0"),
        DeclareLaunchArgument("command_speed", default_value="0.10"),
        DeclareLaunchArgument("command_acceleration", default_value="0.30"),
        DeclareLaunchArgument(
            "path_command_projection_tolerance", default_value="0.05"
        ),
        DeclareLaunchArgument(
            "path_command_entry_tolerance", default_value="0.30"
        ),
        DeclareLaunchArgument(
            "path_command_connector_tolerance", default_value="0.20"
        ),
        DeclareLaunchArgument(
            "path_command_suffix_tolerance", default_value="0.01"
        ),
        DeclareLaunchArgument("path_corner_tolerance", default_value="0.05"),
        DeclareLaunchArgument("route_command_grace", default_value="2.0"),
        DeclareLaunchArgument("replan_during_yaw_align", default_value="false"),
        DeclareLaunchArgument("geofence_radius", default_value="1.0"),
        DeclareLaunchArgument("planner_fault_land_time", default_value="3.0"),
        DeclareLaunchArgument("goal_hold_time", default_value="3.0"),
        DeclareLaunchArgument("max_correction_m", default_value="0.25"),
        DeclareLaunchArgument("max_correction_yaw_deg", default_value="5.0"),
        DeclareLaunchArgument("min_vio_features", default_value="80"),
        DeclareLaunchArgument("vio_feature_loss_time", default_value="0.25"),
        DeclareLaunchArgument("max_horizontal_error", default_value="0.35"),
        DeclareLaunchArgument("transit_horizontal_error", default_value="0.60"),
        DeclareLaunchArgument("pre_route_max_horizontal_error", default_value="0.15"),
        # Yaw follows the path heading. yaw_rate_deg is the slew of the published
        # yaw setpoint (the node's own watchdog lands above max_yaw_rate_deg=60);
        # 20 deg/s turns 90 deg in ~4.5s without stepping the mixer.
        DeclareLaunchArgument("yaw_follows_heading", default_value="true"),
        DeclareLaunchArgument("yaw_rate_deg", default_value="20.0"),
        DeclareLaunchArgument("yaw_track_min_displacement", default_value="0.15"),
        DeclareLaunchArgument("yaw_track_deadband_deg", default_value="15.0"),
        DeclareLaunchArgument("yaw_align_error_deg", default_value="40.0"),
        DeclareLaunchArgument("yaw_resume_error_deg", default_value="15.0"),
        DeclareLaunchArgument("tracking_loss_land", default_value="true"),
        DeclareLaunchArgument("keyboard_kill", default_value="true"),
        DeclareLaunchArgument("keyboard_land", default_value="true"),
        DeclareLaunchArgument("foxglove_teleop", default_value="true"),
        DeclareLaunchArgument(
            "foxglove_teleop_topic", default_value="/planner/flight/teleop"
        ),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("bag_output", default_value=default_bag),
    ]
    node = Node(
        package="px4_vio_bridge",
        executable="offboard_global_planner",
        name="offboard_global_planner",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "auto_arm": typed("auto_arm", bool),
            "hover_height": typed("hover_height", float),
            "climb_timeout": typed("climb_timeout", float),
            "max_flight_time": typed("max_flight_time", float),
            "command_speed": typed("command_speed", float),
            "command_acceleration": typed("command_acceleration", float),
            "path_command_projection_tolerance": typed(
                "path_command_projection_tolerance", float
            ),
            "path_command_entry_tolerance": typed(
                "path_command_entry_tolerance", float
            ),
            "path_command_connector_tolerance": typed(
                "path_command_connector_tolerance", float
            ),
            "path_command_suffix_tolerance": typed(
                "path_command_suffix_tolerance", float
            ),
            "path_corner_tolerance": typed("path_corner_tolerance", float),
            "route_command_grace": typed("route_command_grace", float),
            "replan_during_yaw_align": typed("replan_during_yaw_align", bool),
            "geofence_radius": typed("geofence_radius", float),
            "planner_fault_land_time": typed("planner_fault_land_time", float),
            "goal_hold_time": typed("goal_hold_time", float),
            "max_correction_m": typed("max_correction_m", float),
            "max_correction_yaw_deg": typed("max_correction_yaw_deg", float),
            "min_vio_features": typed("min_vio_features", int),
            "vio_feature_loss_time": typed("vio_feature_loss_time", float),
            "max_horizontal_error": typed("max_horizontal_error", float),
            "transit_horizontal_error": typed("transit_horizontal_error", float),
            "pre_route_max_horizontal_error": typed(
                "pre_route_max_horizontal_error", float
            ),
            "yaw_follows_heading": typed("yaw_follows_heading", bool),
            "yaw_rate_deg": typed("yaw_rate_deg", float),
            "yaw_track_min_displacement": typed("yaw_track_min_displacement", float),
            "yaw_track_deadband_deg": typed("yaw_track_deadband_deg", float),
            "yaw_align_error_deg": typed("yaw_align_error_deg", float),
            "yaw_resume_error_deg": typed("yaw_resume_error_deg", float),
            "tracking_loss_land": typed("tracking_loss_land", bool),
            "keyboard_kill": typed("keyboard_kill", bool),
            "keyboard_land": typed("keyboard_land", bool),
            "foxglove_teleop": typed("foxglove_teleop", bool),
            "foxglove_teleop_topic": LaunchConfiguration("foxglove_teleop_topic"),
        }],
    )
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--all-topics", "--storage", "mcap",
            "--storage-preset-profile", STORAGE_PRESET,
            "--disable-keyboard-controls", "--polling-interval", "100",
            "--exclude-regex",
            (
                r"^/(parameter_events|tf|tf_static|"
                r"rtabmap/(image.*|depth|camera_info|obstacle_cloud|ground_cloud|path)|"
                r"px4/local_position/path|vio/yaw_offset/path)$"
            ),
            "--output", LaunchConfiguration("bag_output"),
        ],
        name="global_planner_flight_recorder",
        output="screen",
        sigterm_timeout="20",
        sigkill_timeout="20",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    start = TimerAction(period=3.0, actions=[node])
    stop_after_node = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(
                reason="global planner flight node finished; finalizing bag"
            ))],
        )
    )
    stop_if_recorder_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(
                reason="flight recorder exited; stopping flight launch"
            ))],
        ),
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    return LaunchDescription(arguments + [
        LogInfo(msg="GLOBAL PLANNER FLIGHT: auto_arm defaults false; K=kill, L=land"),
        LogInfo(msg=["Flight bag output: ", LaunchConfiguration("bag_output")]),
        OpaqueFunction(function=write_run_marker),
        recorder,
        stop_after_node,
        stop_if_recorder_exits,
        start,
    ])
