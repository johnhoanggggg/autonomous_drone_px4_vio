"""Launch the observation-only OctoMap 3D planner.

This launch never starts a PX4 adapter and records only planner/perception
topics. It is the monitor/replay gate from HANDOFF_3D_NAVIGATION.md.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from px4_vio_bridge.log_paths import timestamped_bag


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def required_node_exited(event, context):
    if context.is_shutdown:
        return []
    return [
        LogInfo(msg=(
            f"ERROR: required 3D planner '{event.process_name}' exited "
            f"with code {event.returncode}"
        )),
        EmitEvent(event=Shutdown(reason="required 3D planner exited")),
    ]


def recorder_exited(event, _context):
    if event.returncode in (0, -2, -15):
        return []
    return [
        LogInfo(msg=f"ERROR: 3D planner recorder exited with code {event.returncode}"),
        EmitEvent(event=Shutdown(reason="3D planner recorder failed")),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("map_topic", default_value="/rtabmap/octomap"),
        DeclareLaunchArgument("map_data_topic", default_value="/rtabmap/mapData"),
        DeclareLaunchArgument(
            "map_metadata_topic", default_value="/rtabmap/octomap_metadata"
        ),
        DeclareLaunchArgument("octomap_producer", default_value="true"),
        DeclareLaunchArgument("route_follower", default_value="true"),
        DeclareLaunchArgument("simulate", default_value="false"),
        DeclareLaunchArgument("fixture_resolution", default_value="0.10"),
        DeclareLaunchArgument("fixture_loop_correction_after", default_value="5.0"),
        DeclareLaunchArgument("pose_topic", default_value="/rtabmap/pose"),
        DeclareLaunchArgument("goal_topic", default_value="/waypoint/clicked"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("robot_radius", default_value="0.25"),
        DeclareLaunchArgument("safety_margin", default_value="0.10"),
        DeclareLaunchArgument("max_cross_track", default_value="0.05"),
        DeclareLaunchArgument("inflation_extra", default_value="0.20"),
        DeclareLaunchArgument("planning_radius_xy", default_value="3.0"),
        DeclareLaunchArgument("min_z", default_value="0.20"),
        DeclareLaunchArgument("max_z", default_value="2.00"),
        DeclareLaunchArgument("planning_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("map_timeout", default_value="3.0"),
        DeclareLaunchArgument("pose_timeout", default_value="1.0"),
        DeclareLaunchArgument("start_recovery_radius", default_value="0.20"),
        DeclareLaunchArgument("heuristic_weight", default_value="1.0"),
        DeclareLaunchArgument("cost_weight", default_value="2.0"),
        DeclareLaunchArgument("inflation_cost_scaling", default_value="3.0"),
        DeclareLaunchArgument("planning_timeout_ms", default_value="150.0"),
        DeclareLaunchArgument("follower_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("lookahead", default_value="0.35"),
        DeclareLaunchArgument("max_horizontal_speed", default_value="0.10"),
        DeclareLaunchArgument("max_vertical_speed", default_value="0.05"),
        DeclareLaunchArgument("max_horizontal_acceleration", default_value="0.30"),
        DeclareLaunchArgument("max_vertical_acceleration", default_value="0.20"),
        DeclareLaunchArgument("max_vertical_track", default_value="0.05"),
        DeclareLaunchArgument("vio_timeout", default_value="0.5"),
        DeclareLaunchArgument("correction_timeout", default_value="1.0"),
        DeclareLaunchArgument("max_correction_m", default_value="0.50"),
        DeclareLaunchArgument("max_correction_yaw_deg", default_value="15.0"),
        DeclareLaunchArgument("max_correction_roll_pitch_deg", default_value="5.0"),
        DeclareLaunchArgument("max_marker_voxels", default_value="20000"),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument(
            "bag_output", default_value=timestamped_bag("global_planner_3d_monitor")
        ),
    ]
    parameters = [{
        "map_topic": LaunchConfiguration("map_topic"),
        "map_metadata_topic": LaunchConfiguration("map_metadata_topic"),
        "require_map_metadata": True,
        "pose_topic": LaunchConfiguration("pose_topic"),
        "goal_topic": LaunchConfiguration("goal_topic"),
        "frame_id": LaunchConfiguration("frame_id"),
        "voxel_size": typed("voxel_size", float),
        "robot_radius": typed("robot_radius", float),
        "safety_margin": typed("safety_margin", float),
        "max_cross_track": typed("max_cross_track", float),
        "inflation_extra": typed("inflation_extra", float),
        "planning_radius_xy": typed("planning_radius_xy", float),
        "min_z": typed("min_z", float),
        "max_z": typed("max_z", float),
        "planning_rate_hz": typed("planning_rate_hz", float),
        "map_timeout": typed("map_timeout", float),
        "pose_timeout": typed("pose_timeout", float),
        "start_recovery_radius": typed("start_recovery_radius", float),
        "heuristic_weight": typed("heuristic_weight", float),
        "cost_weight": typed("cost_weight", float),
        "inflation_cost_scaling": typed("inflation_cost_scaling", float),
        "planning_timeout_ms": typed("planning_timeout_ms", float),
        "max_marker_voxels": typed("max_marker_voxels", int),
    }]
    planner = Node(
        package="px4_vio_bridge",
        executable="cpp_astar_planner_3d",
        name="global_planner_3d_monitor",
        output="screen",
        emulate_tty=True,
        parameters=parameters,
    )
    producer = Node(
        package="px4_vio_bridge",
        executable="rtabmap_octomap_node",
        name="rtabmap_octomap",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("octomap_producer")),
        parameters=[{
            "map_data_topic": LaunchConfiguration("map_data_topic"),
            "frame_id": LaunchConfiguration("frame_id"),
            "max_marker_voxels": typed("max_marker_voxels", int),
        }],
    )
    follower = Node(
        package="px4_vio_bridge",
        executable="cpp_route_follower_3d",
        name="route_follower_3d_monitor",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("route_follower")),
        parameters=[{
            "frame_id": LaunchConfiguration("frame_id"),
            "map_topic": LaunchConfiguration("map_topic"),
            "map_metadata_topic": LaunchConfiguration("map_metadata_topic"),
            "pose_topic": LaunchConfiguration("pose_topic"),
            "rate_hz": typed("follower_rate_hz", float),
            "robot_radius": typed("robot_radius", float),
            "safety_margin": typed("safety_margin", float),
            "planning_radius_xy": typed("planning_radius_xy", float),
            "min_z": typed("min_z", float),
            "max_z": typed("max_z", float),
            "lookahead": typed("lookahead", float),
            "max_horizontal_speed": typed("max_horizontal_speed", float),
            "max_vertical_speed": typed("max_vertical_speed", float),
            "max_horizontal_acceleration": typed("max_horizontal_acceleration", float),
            "max_vertical_acceleration": typed("max_vertical_acceleration", float),
            "max_cross_track": typed("max_cross_track", float),
            "max_vertical_track": typed("max_vertical_track", float),
            "vio_timeout": typed("vio_timeout", float),
            "correction_timeout": typed("correction_timeout", float),
            "max_correction_m": typed("max_correction_m", float),
            "max_correction_yaw_deg": typed("max_correction_yaw_deg", float),
            "max_correction_roll_pitch_deg": typed(
                "max_correction_roll_pitch_deg", float
            ),
        }],
    )
    fixture = Node(
        package="px4_vio_bridge",
        executable="rtabmap_3d_fixture",
        name="rtabmap_3d_fixture",
        output="screen",
        condition=IfCondition(LaunchConfiguration("simulate")),
        parameters=[{
            "resolution": typed("fixture_resolution", float),
            "loop_correction_after": typed("fixture_loop_correction_after", float),
        }],
    )
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--storage", "mcap",
            "--storage-preset-profile", "fastwrite",
            "--disable-keyboard-controls", "--topics",
            "/rtabmap/mapData", "/rtabmap/octomap", "/rtabmap/octomap_metadata",
            "/rtabmap/octomap_markers",
            "/rtabmap/pose", "/rtabmap/vio_pose",
            "/rtabmap/odom_correction", "/waypoint/clicked",
            "/planner3d/path", "/planner3d/candidate_path",
            "/planner3d/status", "/planner3d/markers",
            "/planner3d/map_generation", "/planner3d/path_map_generation",
            "/planner3d/follower/displacement", "/planner3d/follower/velocity",
            "/planner3d/follower/acceleration", "/planner3d/follower/carrot",
            "/planner3d/follower/lookahead", "/planner3d/follower/valid",
            "/planner3d/follower/goal_reached", "/planner3d/follower/status",
            "--output", LaunchConfiguration("bag_output"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("foxglove")),
        launch_arguments={
            "port": LaunchConfiguration("foxglove_port"),
            "topic_whitelist": (
                "['^/tf(_static)?$', "
                "'^/rtabmap/(octomap_markers|octomap_metadata|pose|vio_pose|odom_correction)$', "
                "'^/planner3d/.*$', '^/waypoint/clicked$']"
            ),
            "service_whitelist": "['^$']",
            "param_whitelist": "['^$']",
            "client_topic_whitelist": "['^/waypoint/clicked$']",
            "capabilities": "[clientPublish,connectionGraph]",
            "min_qos_depth": "1",
            "max_qos_depth": "1",
        }.items(),
    )
    return LaunchDescription(arguments + [
        LogInfo(msg=(
            "3D planner monitor is observation only; it has no /fmu/in publishers. "
            "Goals must be PointStamped in frame 'world' and clicked Z is preserved."
        )),
        RegisterEventHandler(OnProcessExit(target_action=planner, on_exit=required_node_exited)),
        RegisterEventHandler(OnProcessExit(target_action=producer, on_exit=required_node_exited)),
        RegisterEventHandler(OnProcessExit(target_action=follower, on_exit=required_node_exited)),
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=recorder_exited)),
        recorder,
        foxglove,
        producer,
        fixture,
        planner,
        TimerAction(period=1.0, actions=[follower]),
    ])
