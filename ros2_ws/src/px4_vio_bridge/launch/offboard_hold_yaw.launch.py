"""Launch the bounded PX4 offboard position-hold/yaw test."""
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

    A flight bag that comes back empty tells you nothing about *why*: you cannot
    tell a stale launch file from a storage-profile problem from a recorder that
    never started, and `launch.log` is itself lost whenever the process tree dies
    without flushing. This sidecar is written eagerly, so it survives regardless
    and says exactly which launch file and storage profile actually ran.
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
    except OSError as exc:  # never let diagnostics block a flight
        return [LogInfo(msg=f"could not write run marker: {exc}")]
    return [LogInfo(msg=f"Run marker: {marker}")]


def generate_launch_description():
    default_bag_output = timestamped_bag("offboard_hold_yaw")

    arguments = [
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("hover_height", default_value="0.3"),
        DeclareLaunchArgument("hold_time", default_value="3.0"),
        DeclareLaunchArgument("yaw_angle_deg", default_value="15.0"),
        DeclareLaunchArgument("yaw_rate_deg", default_value="5.0"),
        DeclareLaunchArgument("yaw_feedforward", default_value="false"),
        DeclareLaunchArgument("yaw_tolerance_deg", default_value="5.0"),
        DeclareLaunchArgument("yaw_timeout", default_value="6.0"),
        DeclareLaunchArgument("return_hold_time", default_value="2.0"),
        DeclareLaunchArgument("climb_timeout", default_value="15.0"),
        DeclareLaunchArgument("max_flight_time", default_value="40.0"),
        DeclareLaunchArgument("min_vio_features", default_value="160"),
        DeclareLaunchArgument("vio_feature_loss_time", default_value="0.25"),
        DeclareLaunchArgument("max_vio_yaw_error_deg", default_value="20.0"),
        DeclareLaunchArgument("vio_yaw_error_time", default_value="0.20"),
        DeclareLaunchArgument("max_yaw_rate_deg", default_value="60.0"),
        DeclareLaunchArgument("yaw_rate_loss_time", default_value="0.10"),
        DeclareLaunchArgument("max_horizontal_error", default_value="0.35"),
        DeclareLaunchArgument("pre_yaw_max_horizontal_error", default_value="0.15"),
        DeclareLaunchArgument("horizontal_error_time", default_value="0.25"),
        DeclareLaunchArgument("tracking_loss_land", default_value="true"),
        DeclareLaunchArgument("keyboard_kill", default_value="true"),
        DeclareLaunchArgument("keyboard_land", default_value="true"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("bag_output", default_value=default_bag_output),
    ]
    node = Node(
        package="px4_vio_bridge",
        executable="offboard_hold_yaw",
        name="offboard_hold_yaw",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "auto_arm": typed("auto_arm", bool),
                "hover_height": typed("hover_height", float),
                "hold_time": typed("hold_time", float),
                "yaw_angle_deg": typed("yaw_angle_deg", float),
                "yaw_rate_deg": typed("yaw_rate_deg", float),
                "yaw_feedforward": typed("yaw_feedforward", bool),
                "yaw_tolerance_deg": typed("yaw_tolerance_deg", float),
                "yaw_timeout": typed("yaw_timeout", float),
                "return_hold_time": typed("return_hold_time", float),
                "climb_timeout": typed("climb_timeout", float),
                "max_flight_time": typed("max_flight_time", float),
                "min_vio_features": typed("min_vio_features", int),
                "vio_feature_loss_time": typed("vio_feature_loss_time", float),
                "max_vio_yaw_error_deg": typed("max_vio_yaw_error_deg", float),
                "vio_yaw_error_time": typed("vio_yaw_error_time", float),
                "max_yaw_rate_deg": typed("max_yaw_rate_deg", float),
                "yaw_rate_loss_time": typed("yaw_rate_loss_time", float),
                "max_horizontal_error": typed("max_horizontal_error", float),
                "pre_yaw_max_horizontal_error": typed(
                    "pre_yaw_max_horizontal_error", float
                ),
                "horizontal_error_time": typed("horizontal_error_time", float),
                "tracking_loss_land": typed("tracking_loss_land", bool),
                "keyboard_kill": typed("keyboard_kill", bool),
                "keyboard_land": typed("keyboard_land", bool),
            }
        ],
    )

    # Capture all dynamically appearing control/estimator topics, but keep the
    # camera and visualization products out of the bag so recording cannot
    # compete with flight control for CPU, DDS bandwidth, or storage writes.
    #
    # `fastwrite` (no chunking, no compression) is REQUIRED, not a performance
    # tweak -- do not "optimise" it back to zstd_fast. MCAP buffers a whole
    # chunk in memory before touching the disk, and a flight bag (~1.3 MB
    # compressed) never fills one, so with zstd_fast the entire recording lives
    # in RAM until the writer is closed cleanly. Any death that skips cleanup
    # (terminal closed -> SIGHUP, SIGKILL, power loss) leaves a 0-byte .mcap and
    # the whole flight is gone. That destroyed the 20260725T063349Z and
    # 20260726T041845Z bags. Measured on 2026-07-26 by SIGKILLing a recorder
    # mid-run: zstd_fast -> 0 bytes, 0 messages; fastwrite -> 155 KB, 1442
    # messages recovered. fastwrite also costs less CPU in flight; the only
    # price is a larger file, which is irrelevant at 35 GB free.
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
        # Give the writer room to finalize (index + metadata.yaml) on a clean
        # shutdown instead of being escalated to SIGTERM/SIGKILL mid-flush.
        # fastwrite already makes the data survive without this; these just buy
        # a properly indexed bag in the normal case.
        sigterm_timeout="20",
        sigkill_timeout="20",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    # A short delay lets rosbag discovery finish before the node can request
    # OFFBOARD/ARM. When the flight node reaches DONE after normal completion,
    # LAND, KILL, or abort, shutting down the launch also finalizes the MCAP.
    start_flight = TimerAction(period=3.0, actions=[node])
    stop_after_flight = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="offboard_hold_yaw finished; finalizing flight bag"
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
            OpaqueFunction(function=write_run_marker),
            recorder,
            stop_after_flight,
            stop_if_recorder_fails,
            start_flight,
        ]
    )
