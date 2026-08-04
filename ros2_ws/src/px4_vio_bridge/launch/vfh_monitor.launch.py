"""Run the VFH planner in observation mode, optionally recording the session.

The monitor publishes nothing to PX4 — it only reads the obstacle cloud and the
SLAM pose — so this launch is safe to run at any time, including while another
flight node is flying the vehicle.

The stack must be up WITH clouds, which is not the default:

    ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py slam_publish_clouds:=true
    ros2 launch px4_vio_bridge vfh_monitor.launch.py

`record_bag:=true` writes a `flight_logs/vfh_monitor_<UTC>` MCAP with the same
`fastwrite` profile the flight launches use, excluding the camera and the raw
cloud (a 15 Hz XYZRGB cloud is far larger than everything else combined).
"""
from datetime import datetime, timezone
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

STORAGE_PRESET = "fastwrite"


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    bag_root = Path.cwd() / "flight_logs"
    bag_root.mkdir(parents=True, exist_ok=True)
    default_bag_output = str(bag_root / f"vfh_monitor_{stamp}")

    arguments = [
        DeclareLaunchArgument("rate_hz", default_value="10.0"),
        DeclareLaunchArgument("cloud_topic", default_value="/rtabmap/obstacle_cloud"),
        DeclareLaunchArgument("pose_topic", default_value="/rtabmap/pose"),
        DeclareLaunchArgument("goal_topic", default_value="/waypoint/clicked"),
        DeclareLaunchArgument("obstacle_timeout", default_value="1.0"),
        # Histogram shape.
        DeclareLaunchArgument("sectors", default_value="72"),
        DeclareLaunchArgument("min_range", default_value="0.25"),
        DeclareLaunchArgument("max_range", default_value="2.0"),
        DeclareLaunchArgument("min_points", default_value="4"),
        DeclareLaunchArgument("tau_high", default_value="6.0"),
        DeclareLaunchArgument("tau_low", default_value="3.0"),
        DeclareLaunchArgument("smoothing", default_value="3"),
        # Vehicle envelope and steering limits.
        DeclareLaunchArgument("robot_radius", default_value="0.30"),
        DeclareLaunchArgument("safety_margin", default_value="0.10"),
        DeclareLaunchArgument("max_steer_deg", default_value="35.0"),
        DeclareLaunchArgument("display_fov_deg", default_value="90.0"),
        DeclareLaunchArgument("wide_valley_deg", default_value="40.0"),
        DeclareLaunchArgument("mu_target", default_value="5.0"),
        DeclareLaunchArgument("mu_heading", default_value="2.0"),
        DeclareLaunchArgument("mu_previous", default_value="2.0"),
        # Height slab around the vehicle: only these points can be hit.
        DeclareLaunchArgument("z_below", default_value="0.15"),
        DeclareLaunchArgument("z_above", default_value="0.60"),
        DeclareLaunchArgument("max_samples", default_value="1200"),
        # Keep world-frame obstacle voxels after they leave the camera view.
        DeclareLaunchArgument("memory_duration", default_value="30.0"),
        DeclareLaunchArgument("memory_voxel_size", default_value="0.10"),
        DeclareLaunchArgument("memory_max_points", default_value="20000"),
        DeclareLaunchArgument(
            "memory_correction_topic", default_value="/vio/map_correction_target"
        ),
        DeclareLaunchArgument("memory_reset_correction_m", default_value="0.05"),
        DeclareLaunchArgument("memory_reset_correction_deg", default_value="2.0"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument("bag_output", default_value=default_bag_output),
    ]

    node = Node(
        package="px4_vio_bridge",
        executable="vfh_monitor",
        name="vfh_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "rate_hz": typed("rate_hz", float),
                "cloud_topic": typed("cloud_topic", str),
                "pose_topic": typed("pose_topic", str),
                "goal_topic": typed("goal_topic", str),
                "obstacle_timeout": typed("obstacle_timeout", float),
                "sectors": typed("sectors", int),
                "min_range": typed("min_range", float),
                "max_range": typed("max_range", float),
                "min_points": typed("min_points", int),
                "tau_high": typed("tau_high", float),
                "tau_low": typed("tau_low", float),
                "smoothing": typed("smoothing", int),
                "robot_radius": typed("robot_radius", float),
                "safety_margin": typed("safety_margin", float),
                "max_steer_deg": typed("max_steer_deg", float),
                "display_fov_deg": typed("display_fov_deg", float),
                "wide_valley_deg": typed("wide_valley_deg", float),
                "mu_target": typed("mu_target", float),
                "mu_heading": typed("mu_heading", float),
                "mu_previous": typed("mu_previous", float),
                "z_below": typed("z_below", float),
                "z_above": typed("z_above", float),
                "max_samples": typed("max_samples", int),
                "memory_duration": typed("memory_duration", float),
                "memory_voxel_size": typed("memory_voxel_size", float),
                "memory_max_points": typed("memory_max_points", int),
                "memory_correction_topic": typed("memory_correction_topic", str),
                "memory_reset_correction_m": typed(
                    "memory_reset_correction_m", float
                ),
                "memory_reset_correction_deg": typed(
                    "memory_reset_correction_deg", float
                ),
            }
        ],
    )

    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--all-topics",
            "--storage",
            "mcap",
            "--storage-preset-profile",
            STORAGE_PRESET,
            "--disable-keyboard-controls",
            "--polling-interval",
            "100",
            "--exclude-regex",
            (
                r"^/(parameter_events|tf|tf_static|"
                r"rtabmap/(image.*|depth|camera_info|obstacle_cloud|ground_cloud|path)|"
                r"px4/local_position/path|vio/yaw_offset/path)$"
            ),
            "--output",
            LaunchConfiguration("bag_output"),
        ],
        name="vfh_monitor_recorder",
        output="screen",
        sigterm_timeout="20",
        sigkill_timeout="20",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(
                msg=(
                    "vfh_monitor: observation only. Needs the stack running with "
                    "slam_publish_clouds:=true."
                )
            ),
            recorder,
            node,
        ]
    )
