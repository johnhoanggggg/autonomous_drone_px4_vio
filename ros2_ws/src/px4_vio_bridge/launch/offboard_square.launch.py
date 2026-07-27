"""Launch the scripted square-path flight, with the flight bag.

Run the SLAM/PX4/Foxglove stack (`rtabmap_slam_px4.launch.py`) first; this launch
adds only the flight node and its recorder.

Sequence per side: fly `side_m` along the current heading, settle, yaw by
`turn_deg`, settle. `sides` repetitions close the shape and return to the latched
start position and start heading.

The yaw rate defaults to 15 deg/s here, not the 5 deg/s the hold-yaw test uses. A
90 deg turn at 5 deg/s takes 18 s, so four of them plus four legs do not fit in
any sane max_flight_time. Leg and turn timeouts are derived from the geometry by
the node, which logs the worst-case budget and complains if it exceeds
max_flight_time.
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
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    bag_root = Path.cwd() / "flight_logs"
    bag_root.mkdir(parents=True, exist_ok=True)
    default_bag_output = str(bag_root / f"offboard_square_{stamp}")

    arguments = [
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("hover_height", default_value="0.3"),
        # Must cover the node's worst-case budget (it logs the number and
        # errors if this is too low): sides x (leg timeout + settle + turn
        # timeout + settle) plus the climb. Typical completion is far quicker,
        # around 50 s. Hover draw is ~262 W -- start this one on a full pack.
        DeclareLaunchArgument("max_flight_time", default_value="150.0"),
        DeclareLaunchArgument("waypoint_speed", default_value="0.25"),
        DeclareLaunchArgument("geofence_radius", default_value="1.5"),
        DeclareLaunchArgument("arrival_tol", default_value="0.12"),
        DeclareLaunchArgument("waypoint_frame", default_value="world"),
        DeclareLaunchArgument("side_m", default_value="0.40"),
        DeclareLaunchArgument("turn_deg", default_value="90.0"),
        DeclareLaunchArgument("sides", default_value="4"),
        DeclareLaunchArgument("corner_tol", default_value="0.15"),
        DeclareLaunchArgument("yaw_tolerance_deg", default_value="5.0"),
        DeclareLaunchArgument("leg_settle_time", default_value="1.5"),
        DeclareLaunchArgument("turn_settle_time", default_value="1.0"),
        DeclareLaunchArgument("leg_timeout_margin", default_value="6.0"),
        DeclareLaunchArgument("turn_timeout_margin", default_value="4.0"),
        DeclareLaunchArgument("velocity_feedforward", default_value="false"),
        DeclareLaunchArgument("transit_horizontal_error", default_value="0.60"),
        DeclareLaunchArgument("transit_settle_time", default_value="1.0"),
        DeclareLaunchArgument(
            "pre_waypoint_max_horizontal_error", default_value="0.15"
        ),
        # 15 deg/s, not the hold-yaw 5: a 90 deg turn at 5 deg/s is 18 s and
        # four of them do not fit in a flight. Well inside MC_YAWRATE_MAX=60
        # and the 60 deg/s abort, but it is 3x the largest rate flown so far.
        DeclareLaunchArgument("yaw_rate_deg", default_value="15.0"),
        DeclareLaunchArgument("yaw_feedforward", default_value="false"),
        DeclareLaunchArgument("climb_timeout", default_value="15.0"),
        # 80, not the 160 (40% of the 400-feature target) used by the hover and
        # yaw tests. Waypoint flight deliberately translates and repoints the
        # camera at whatever the room offers, so it samples worse scenes than a
        # station-keeping hover ever did. Lowered on 2026-07-27 after a bench run
        # aborted at 134 features. This buys tolerance, not tracking quality --
        # if counts are routinely this low, fix the scene (lighting, texture,
        # avoid blank walls), because low-feature VIO is what drifts.
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
        executable="offboard_square",
        name="offboard_square",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "auto_arm": typed("auto_arm", bool),
                "hover_height": typed("hover_height", float),
                "max_flight_time": typed("max_flight_time", float),
                "waypoint_speed": typed("waypoint_speed", float),
                "geofence_radius": typed("geofence_radius", float),
                "arrival_tol": typed("arrival_tol", float),
                "waypoint_frame": typed("waypoint_frame", str),
                "side_m": typed("side_m", float),
                "turn_deg": typed("turn_deg", float),
                "sides": typed("sides", int),
                "corner_tol": typed("corner_tol", float),
                "yaw_tolerance_deg": typed("yaw_tolerance_deg", float),
                "leg_settle_time": typed("leg_settle_time", float),
                "turn_settle_time": typed("turn_settle_time", float),
                "leg_timeout_margin": typed("leg_timeout_margin", float),
                "turn_timeout_margin": typed("turn_timeout_margin", float),
                "velocity_feedforward": typed("velocity_feedforward", bool),
                "transit_horizontal_error": typed(
                    "transit_horizontal_error", float
                ),
                "transit_settle_time": typed("transit_settle_time", float),
                "pre_waypoint_max_horizontal_error": typed(
                    "pre_waypoint_max_horizontal_error", float
                ),
                "yaw_rate_deg": typed("yaw_rate_deg", float),
                "yaw_feedforward": typed("yaw_feedforward", bool),
                "climb_timeout": typed("climb_timeout", float),
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

    # Capture all dynamically appearing control/estimator topics (including
    # /waypoint/*, so a flight can be replayed click by click), but keep the
    # camera and visualization products out of the bag so recording cannot
    # compete with flight control for CPU, DDS bandwidth, or storage writes.
    #
    # `fastwrite` (no chunking, no compression) is REQUIRED, not a performance
    # tweak -- do not "optimise" it back to zstd_fast. MCAP buffers a whole
    # chunk in memory before touching the disk, and a flight bag never fills
    # one, so with zstd_fast the entire recording lives in RAM until the writer
    # is closed cleanly. Any death that skips cleanup (terminal closed ->
    # SIGHUP, SIGKILL, power loss) leaves a 0-byte .mcap and the whole flight is
    # gone. See HANDOFF.md for the measurements.
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
                        reason="offboard_square finished; finalizing flight bag"
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
