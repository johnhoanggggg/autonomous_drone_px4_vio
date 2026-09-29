"""Launch the fixed-heading open-area lateral braking test."""
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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

# Wraps `ros2 bag record` so the bag is flushed to the SD card and finalized
# even when the Pi loses power shortly after the flight (see the script).
DURABLE_RECORD = PathJoinSubstitution(
    [FindPackagePrefix("px4_vio_bridge"), "lib", "px4_vio_bridge", "durable_bag_record"])

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
    default_bag_output = timestamped_bag("offboard_lateral_stop")

    arguments = [
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("hover_height", default_value="0.3"),
        DeclareLaunchArgument("axis", default_value="lateral",
                              description="lateral (body-right legs, roll brakes) or forward (nose legs, pitch brakes)"),
        DeclareLaunchArgument("distance", default_value="0.35"),
        DeclareLaunchArgument("speed", default_value="0.20"),
        DeclareLaunchArgument("acceleration", default_value="0.30"),
        DeclareLaunchArgument("brake_damping", default_value="false"),
        DeclareLaunchArgument("damping_gain", default_value="0.5"),
        DeclareLaunchArgument("damping_limit", default_value="0.10"),
        DeclareLaunchArgument("damping_time_constant", default_value="0.10"),
        DeclareLaunchArgument("settle_hold", default_value="3.0"),
        DeclareLaunchArgument("settle_dwell", default_value="1.0"),
        DeclareLaunchArgument("settle_timeout", default_value="10.0"),
        DeclareLaunchArgument("settle_position", default_value="0.10"),
        DeclareLaunchArgument("settle_speed", default_value="0.06"),
        DeclareLaunchArgument("settle_altitude", default_value="0.02",
                              description="post-climb altitude tolerance (m) before the first leg"),
        DeclareLaunchArgument("climb_settle_hold", default_value="4.0",
                              description="seconds at height, counted from arrival, before the first leg"),
        DeclareLaunchArgument("settle_vz", default_value="0.05",
                              description="post-climb vertical speed limit (m/s) before the first leg"),
        DeclareLaunchArgument("climb_settle_position", default_value="0.03",
                              description="post-climb hold-point error (m) before the first leg (lean trimmed out)"),
        DeclareLaunchArgument("climb_settle_speed", default_value="0.04",
                              description="post-climb horizontal speed limit (m/s) before the first leg"),
        DeclareLaunchArgument("geofence_radius", default_value="1.0"),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        # 25 s / 60 s leave room for the post-climb settle gates (climb ~3 s,
        # 4 s at height, then the trim wait) on top of the ~30 s leg sequence.
        DeclareLaunchArgument("climb_timeout", default_value="25.0"),
        DeclareLaunchArgument("max_flight_time", default_value="60.0"),
        DeclareLaunchArgument("min_vio_features", default_value="160"),
        DeclareLaunchArgument("max_horizontal_error", default_value="0.30"),
        DeclareLaunchArgument("tracking_loss_land", default_value="true"),
        DeclareLaunchArgument("keyboard_kill", default_value="true"),
        DeclareLaunchArgument("keyboard_land", default_value="true"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("bag_output", default_value=default_bag_output),
    ]
    node = Node(
        package="px4_vio_bridge", executable="offboard_lateral_stop",
        name="offboard_lateral_stop", output="screen", emulate_tty=True,
        parameters=[{
            "auto_arm": typed("auto_arm", bool),
            "hover_height": typed("hover_height", float),
            "axis": typed("axis", str),
            "distance": typed("distance", float),
            "speed": typed("speed", float),
            "acceleration": typed("acceleration", float),
            "brake_damping": typed("brake_damping", bool),
            "damping_gain": typed("damping_gain", float),
            "damping_limit": typed("damping_limit", float),
            "damping_time_constant": typed("damping_time_constant", float),
            "settle_hold": typed("settle_hold", float),
            "settle_dwell": typed("settle_dwell", float),
            "settle_timeout": typed("settle_timeout", float),
            "settle_position": typed("settle_position", float),
            "settle_speed": typed("settle_speed", float),
            "settle_altitude": typed("settle_altitude", float),
            "settle_vz": typed("settle_vz", float),
            "climb_settle_hold": typed("climb_settle_hold", float),
            "climb_settle_position": typed("climb_settle_position", float),
            "climb_settle_speed": typed("climb_settle_speed", float),
            "geofence_radius": typed("geofence_radius", float),
            "rate_hz": typed("rate_hz", float),
            "climb_timeout": typed("climb_timeout", float),
            "max_flight_time": typed("max_flight_time", float),
            "min_vio_features": typed("min_vio_features", int),
            "max_horizontal_error": typed("max_horizontal_error", float),
            "tracking_loss_land": typed("tracking_loss_land", bool),
            "keyboard_kill": typed("keyboard_kill", bool),
            "keyboard_land": typed("keyboard_land", bool),
        }],
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
            DURABLE_RECORD,
            "--",
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
                        reason="offboard_lateral_stop finished; finalizing flight bag"
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
