"""ROS-native OctoMap navigation observation mode; no PX4 publishers.

Attach to an existing ROS mapper by default. start_backend starts the pinned
ROS camera/odometry/mapper profile directly, without the PX4 parent launch.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, EmitEvent, LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def required_exited(event, context):
    if context.is_shutdown:
        return []
    return [LogInfo(msg=f"Required ROS navigation process exited: {event.process_name}"),
            EmitEvent(event=Shutdown(reason="ROS navigation input exited"))]


def validate_test_override(context):
    radius = float(LaunchConfiguration("test_start_clear_radius").perform(context))
    backend = LaunchConfiguration("start_backend").perform(context).lower() == "true"
    if not 0.0 <= radius <= 0.5:
        raise RuntimeError("test_start_clear_radius must be between 0 and 0.5 metres")
    if radius > 0.0 and not backend:
        raise RuntimeError("test_start_clear_radius requires start_backend:=true here; "
                           "for an existing mapper, configure its OctoMap producer directly")
    return []


def generate_launch_description():
    share = FindPackageShare("px4_vio_bridge")
    defaults = {
        "start_backend": "false", "foxglove": "false", "record_bag": "false",
        "use_sim_time": "false", "min_z": "0.20", "max_z": "2.00",
        "test_start_clear_radius": "0.0",
        "process_monitor": "true",
        "planning_radius_xy": "3.0", "robot_radius": "0.25", "safety_margin": "0.10",
        "voxel_size": "0.05", "max_marker_voxels": "5000",
        "camera_position_frd_x": "0.100", "camera_position_frd_y": "-0.036",
        "camera_position_frd_z": "0.056",
    }
    adapter = Node(
        package="px4_vio_bridge", executable="cpp_ros_planner_pose", output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            **{key: ParameterValue(LaunchConfiguration(key), value_type=float)
               for key in defaults if key.startswith("camera_position_frd_")},
        }],
    )
    backend = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_backend")), output="screen",
        cmd=[PathJoinSubstitution([FindPackagePrefix("px4_vio_bridge"), "lib",
                                  "px4_vio_bridge", "ros_rtabmap_backend"]),
             "foxglove:=false", "use_imu:=false", "odometry_rate_hz:=0.0",
             "odom_reset_countdown:=0", "keep_latest_frame:=true",
             "odom_key_frame_thr:=0.3", "odom_vis_keyframe_thr:=150",
             ["use_sim_time:=", LaunchConfiguration("use_sim_time")],
             ["voxel_size:=", LaunchConfiguration("voxel_size")],
             ["camera_config:=", PathJoinSubstitution([share, "config", "oak_v3_rtabmap_tracked.yaml"])],
             ["odom_config_path:=", PathJoinSubstitution([share, "config", "rtabmap_ros_parity.ini"])],
             ["test_start_clear_radius:=", LaunchConfiguration("test_start_clear_radius")],
             *[[key + ":=", LaunchConfiguration(key)] for key in defaults
               if key.startswith("camera_position_frd_")],
             ["max_marker_voxels:=", LaunchConfiguration("max_marker_voxels")]],
    )
    monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [share, "launch", "global_planner_3d_monitor.launch.py"])),
        launch_arguments={
            **{key: LaunchConfiguration(key) for key in (
                "foxglove", "record_bag", "use_sim_time", "min_z", "max_z",
                "planning_radius_xy", "robot_radius", "safety_margin", "voxel_size",
                "max_marker_voxels", "process_monitor")},
            "octomap_producer": "false", "simulate": "false",
            "pose_snapshot_topic": "/planner3d/pose_snapshot", "vio_frame_id": "rtabmap_odom",
        }.items(),
    )
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        OpaqueFunction(function=validate_test_override),
        RegisterEventHandler(OnProcessExit(target_action=adapter, on_exit=required_exited)),
        RegisterEventHandler(OnProcessExit(target_action=backend, on_exit=required_exited)),
        adapter, monitor, backend,
    ])
