"""PARKED experimental VFH2D flight launch, retained for reproducibility.

This mode was never armed and is not part of the current path-planning
direction. No normal stack launch includes it. Do not treat it as a supported
flight mode merely because this dedicated launch file remains available.

Run the stack first, WITH clouds — without `slam_publish_clouds:=true` this node
has no obstacle data at all and will hold position and then land:

    ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py slam_publish_clouds:=true

Historical non-flight validation:

    # 1. observation only, nothing can move
    ros2 launch px4_vio_bridge vfh_monitor.launch.py

    # 2. props-off dry run: the whole state machine, no arm command
    ros2 launch px4_vio_bridge offboard_vfh.launch.py auto_arm:=false climb_timeout:=5.0

There is intentionally no armed procedure here. Reassess the planning
architecture and the dedicated handoff before any future flight use.

`max_flight_time` is 90 s as for the waypoint launch, because the session is
interactive; hover draw is ~269 W, so watch `/battery/level`.
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
    """Drop a `<bag_output>.launchinfo` sidecar before recording starts.

    Same reasoning as the other flight launches: a bag that comes back empty
    must still be able to say which launch file and storage profile ran.
    """
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
    default_bag_output = timestamped_bag("offboard_vfh")

    arguments = [
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("hover_height", default_value="0.3"),
        DeclareLaunchArgument("max_flight_time", default_value="90.0"),
        # Slower than the waypoint default: every extra 0.1 m/s is another
        # 0.1 m of PX4 setpoint lag to absorb before the transit gate trips,
        # and less distance in which to react to something new in the cloud.
        DeclareLaunchArgument("waypoint_speed", default_value="0.20"),
        DeclareLaunchArgument("lookahead", default_value="0.60"),
        DeclareLaunchArgument("plan_period", default_value="0.20"),
        DeclareLaunchArgument("geofence_radius", default_value="1.5"),
        DeclareLaunchArgument("goal_tol", default_value="0.20"),
        DeclareLaunchArgument("idle_timeout", default_value="20.0"),
        DeclareLaunchArgument("waypoint_frame", default_value="world"),
        DeclareLaunchArgument("transit_horizontal_error", default_value="0.60"),
        DeclareLaunchArgument("transit_settle_time", default_value="1.0"),
        DeclareLaunchArgument(
            "pre_waypoint_max_horizontal_error", default_value="0.15"
        ),
        # 15 deg/s as in offboard_square: the vehicle turns into the direction
        # it is flying, and 5 deg/s cannot keep up with a 35 deg steer.
        DeclareLaunchArgument("yaw_rate_deg", default_value="15.0"),
        DeclareLaunchArgument("yaw_feedforward", default_value="false"),
        DeclareLaunchArgument("yaw_follows_direction", default_value="true"),
        # After reaching a stable hover, rotate in place to seed the bounded
        # world-frame obstacle memory before any goal can move the vehicle.
        DeclareLaunchArgument("startup_sweep_min_deg", default_value="-90.0"),
        DeclareLaunchArgument("startup_sweep_max_deg", default_value="90.0"),
        DeclareLaunchArgument("startup_sweep_rate_deg", default_value="15.0"),
        DeclareLaunchArgument("startup_sweep_settle_time", default_value="1.0"),
        DeclareLaunchArgument(
            "startup_sweep_heading_tol_deg", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "startup_sweep_return_timeout", default_value="5.0"
        ),
        DeclareLaunchArgument("climb_timeout", default_value="15.0"),
        # Obstacle input and its watchdogs.
        DeclareLaunchArgument("cloud_topic", default_value="/rtabmap/obstacle_cloud"),
        DeclareLaunchArgument("obstacle_pose_topic", default_value="/rtabmap/pose"),
        DeclareLaunchArgument("obstacle_timeout", default_value="1.0"),
        DeclareLaunchArgument("obstacle_stale_land_time", default_value="2.0"),
        DeclareLaunchArgument("stop_distance", default_value="0.90"),
        DeclareLaunchArgument("abort_distance", default_value="0.50"),
        DeclareLaunchArgument("abort_time", default_value="0.50"),
        DeclareLaunchArgument("blocked_timeout", default_value="10.0"),
        # VFH tunables — tune these against vfh_monitor BEFORE arming.
        DeclareLaunchArgument("sectors", default_value="72"),
        DeclareLaunchArgument("vfh_min_range", default_value="0.25"),
        DeclareLaunchArgument("vfh_max_range", default_value="2.0"),
        DeclareLaunchArgument("min_points", default_value="4"),
        DeclareLaunchArgument("tau_high", default_value="6.0"),
        DeclareLaunchArgument("tau_low", default_value="3.0"),
        DeclareLaunchArgument("smoothing", default_value="3"),
        DeclareLaunchArgument("robot_radius", default_value="0.30"),
        DeclareLaunchArgument("safety_margin", default_value="0.10"),
        DeclareLaunchArgument("max_steer_deg", default_value="35.0"),
        DeclareLaunchArgument("display_fov_deg", default_value="90.0"),
        DeclareLaunchArgument("wide_valley_deg", default_value="40.0"),
        DeclareLaunchArgument("mu_target", default_value="5.0"),
        DeclareLaunchArgument("mu_heading", default_value="2.0"),
        DeclareLaunchArgument("mu_previous", default_value="2.0"),
        DeclareLaunchArgument("z_below", default_value="0.15"),
        DeclareLaunchArgument("z_above", default_value="0.60"),
        DeclareLaunchArgument("max_samples", default_value="1200"),
        # Persistent-enough local map to survive yawing an obstacle out of view.
        DeclareLaunchArgument("memory_duration", default_value="30.0"),
        DeclareLaunchArgument("memory_voxel_size", default_value="0.10"),
        DeclareLaunchArgument("memory_max_points", default_value="20000"),
        DeclareLaunchArgument(
            "memory_correction_topic", default_value="/rtabmap/odom_correction"
        ),
        DeclareLaunchArgument("memory_reset_correction_m", default_value="0.05"),
        DeclareLaunchArgument("memory_reset_correction_deg", default_value="2.0"),
        # As in offboard_waypoint: this flight translates and repoints the
        # camera at whatever the room offers, so it samples worse scenes than a
        # station-keeping hover.
        DeclareLaunchArgument("min_vio_features", default_value="80"),
        DeclareLaunchArgument("vio_feature_loss_time", default_value="0.25"),
        DeclareLaunchArgument("max_vio_yaw_error_deg", default_value="20.0"),
        DeclareLaunchArgument("vio_yaw_error_time", default_value="0.20"),
        DeclareLaunchArgument("max_yaw_rate_deg", default_value="60.0"),
        DeclareLaunchArgument("yaw_rate_loss_time", default_value="0.10"),
        DeclareLaunchArgument("max_horizontal_error", default_value="0.35"),
        DeclareLaunchArgument("horizontal_error_time", default_value="0.25"),
        DeclareLaunchArgument("tracking_loss_land", default_value="true"),
        DeclareLaunchArgument("keyboard_kill", default_value="true"),
        DeclareLaunchArgument("keyboard_land", default_value="true"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("bag_output", default_value=default_bag_output),
    ]

    node = Node(
        package="px4_vio_bridge",
        executable="offboard_vfh",
        name="offboard_vfh",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "auto_arm": typed("auto_arm", bool),
                "hover_height": typed("hover_height", float),
                "max_flight_time": typed("max_flight_time", float),
                "waypoint_speed": typed("waypoint_speed", float),
                "lookahead": typed("lookahead", float),
                "plan_period": typed("plan_period", float),
                "geofence_radius": typed("geofence_radius", float),
                "goal_tol": typed("goal_tol", float),
                "idle_timeout": typed("idle_timeout", float),
                "waypoint_frame": typed("waypoint_frame", str),
                "transit_horizontal_error": typed(
                    "transit_horizontal_error", float
                ),
                "transit_settle_time": typed("transit_settle_time", float),
                "pre_waypoint_max_horizontal_error": typed(
                    "pre_waypoint_max_horizontal_error", float
                ),
                "yaw_rate_deg": typed("yaw_rate_deg", float),
                "yaw_feedforward": typed("yaw_feedforward", bool),
                "yaw_follows_direction": typed("yaw_follows_direction", bool),
                "startup_sweep_min_deg": typed("startup_sweep_min_deg", float),
                "startup_sweep_max_deg": typed("startup_sweep_max_deg", float),
                "startup_sweep_rate_deg": typed(
                    "startup_sweep_rate_deg", float
                ),
                "startup_sweep_settle_time": typed(
                    "startup_sweep_settle_time", float
                ),
                "startup_sweep_heading_tol_deg": typed(
                    "startup_sweep_heading_tol_deg", float
                ),
                "startup_sweep_return_timeout": typed(
                    "startup_sweep_return_timeout", float
                ),
                "climb_timeout": typed("climb_timeout", float),
                "cloud_topic": typed("cloud_topic", str),
                "obstacle_pose_topic": typed("obstacle_pose_topic", str),
                "obstacle_timeout": typed("obstacle_timeout", float),
                "obstacle_stale_land_time": typed(
                    "obstacle_stale_land_time", float
                ),
                "stop_distance": typed("stop_distance", float),
                "abort_distance": typed("abort_distance", float),
                "abort_time": typed("abort_time", float),
                "blocked_timeout": typed("blocked_timeout", float),
                "sectors": typed("sectors", int),
                "vfh_min_range": typed("vfh_min_range", float),
                "vfh_max_range": typed("vfh_max_range", float),
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
                "min_vio_features": typed("min_vio_features", int),
                "vio_feature_loss_time": typed("vio_feature_loss_time", float),
                "max_vio_yaw_error_deg": typed("max_vio_yaw_error_deg", float),
                "vio_yaw_error_time": typed("vio_yaw_error_time", float),
                "max_yaw_rate_deg": typed("max_yaw_rate_deg", float),
                "yaw_rate_loss_time": typed("yaw_rate_loss_time", float),
                "max_horizontal_error": typed("max_horizontal_error", float),
                "horizontal_error_time": typed("horizontal_error_time", float),
                "tracking_loss_land": typed("tracking_loss_land", bool),
                "keyboard_kill": typed("keyboard_kill", bool),
                "keyboard_land": typed("keyboard_land", bool),
            }
        ],
    )

    # `fastwrite` is REQUIRED, not a performance tweak — see HANDOFF.md. The
    # raw obstacle cloud is excluded: at 15 Hz it dwarfs everything else, and
    # `/vfh/*` already records what the planner decided from it.
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
        name="offboard_flight_recorder",
        output="screen",
        sigterm_timeout="20",
        sigkill_timeout="20",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    start_flight = TimerAction(period=3.0, actions=[node])
    stop_after_flight = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="offboard_vfh finished; finalizing flight bag"
                    )
                )
            ],
        )
    )
    stop_if_recorder_fails = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="flight recorder exited; stopping offboard launch"
                    )
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(msg=["Flight bag output: ", LaunchConfiguration("bag_output")]),
            LogInfo(
                msg=(
                    "offboard_vfh needs the stack running with "
                    "slam_publish_clouds:=true, or it will hold and land."
                )
            ),
            OpaqueFunction(function=write_run_marker),
            recorder,
            stop_after_flight,
            stop_if_recorder_fails,
            start_flight,
        ]
    )
