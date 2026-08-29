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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from px4_vio_bridge.log_paths import timestamped_bag


STORAGE_PRESET = "fastwrite"

# Keep the flight bag useful for safety review and offline planner replay without
# subscribing the recorder to every high-rate PX4 and visualization topic on the
# graph.  `/perf/processes` is the canonical per-process record; the scalar perf
# topics remain for direct Foxglove plotting.  Marker arrays, camera/depth/cloud
# data, inflated-map playback and duplicate ROS odometry representations are
# deliberately omitted.
FLIGHT_RECORD_TOPICS = (
    "/rosout",
    # Commands sent to PX4 and the state needed to reconstruct the flight.
    "/fmu/in/offboard_control_mode",
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/vehicle_command",
    "/fmu/in/vehicle_visual_odometry",
    "/fmu/out/battery_status_v1",
    "/fmu/out/estimator_status_flags",
    "/fmu/out/failsafe_flags",
    "/fmu/out/sensor_combined",
    "/fmu/out/vehicle_attitude",
    "/fmu/out/vehicle_command_ack",
    "/fmu/out/vehicle_control_mode",
    "/fmu/out/vehicle_land_detected",
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_status_v1",
    # SLAM/VIO health, map and effective configuration.
    "/rtabmap/config",
    "/rtabmap/grid",
    "/rtabmap/odom_correction",
    "/rtabmap/pose",
    "/rtabmap/vio_feature_count",
    "/rtabmap/vio_pose",
    "/vio/yaw_offset/pose",
    # Requested goal and A* outputs needed by evaluate_planner_bags.py.
    "/waypoint/clicked",
    "/planner/candidate_path",
    "/planner/config",
    "/planner/effective_goal",
    "/planner/expanded_cells",
    "/planner/goal_exact",
    "/planner/goal_terminal",
    "/planner/path",
    "/planner/path_length",
    "/planner/planning_ms",
    "/planner/status",
    # Route-follower geometry, validity and flight-adapter decisions.
    "/planner/follower/carrot",
    "/planner/follower/config",
    "/planner/follower/cross_track",
    "/planner/follower/displacement",
    "/planner/follower/goal_reached",
    "/planner/follower/lookahead",
    "/planner/follower/path_generation",
    "/planner/follower/path_progress",
    "/planner/follower/progress",
    "/planner/follower/remaining",
    "/planner/follower/status",
    "/planner/follower/valid",
    "/planner/follower/vio_displacement",
    "/planner/flight/status",
    "/planner/flight/teleop",
    # Non-commanding C++ clearance shadow. Absent unless cpp_shadow:=true,
    # but keeping them listed makes a parity run self-contained.
    "/planner/flight/cpp_shadow/clearance_valid",
    "/planner/flight/cpp_shadow/endpoint",
    "/planner/flight/cpp_shadow/status",
    # Full machine/process snapshot plus plot-friendly scalar companions.
    "/perf/processes",
    "/perf/cpu_percent",
    "/perf/cpu_temp_c",
    "/perf/load1",
    "/perf/mem_percent",
    "/perf/throttled",
    "/perf/process/astar/cpu_percent",
    "/perf/process/astar_cpp/cpu_percent",
    "/perf/process/bag_record/cpu_percent",
    "/perf/process/follower/cpu_percent",
    "/perf/process/follower_cpp/cpu_percent",
    "/perf/process/foxglove/cpu_percent",
    "/perf/process/planner/cpu_percent",
    "/perf/process/planner_cpp/cpu_percent",
    "/perf/process/planner_cpp_shadow/cpu_percent",
    "/perf/process/px4_pos/cpu_percent",
    "/perf/process/slam/cpu_percent",
    "/perf/process/vio_bridge/cpu_percent",
    "/perf/process/xrce_agent/cpu_percent",
    # Low-rate battery topics used by the existing Foxglove panels.
    "/battery/cell_voltage",
    "/battery/current",
    "/battery/level",
    "/battery/percent",
    "/battery/power",
    "/battery/status",
    "/battery/voltage",
)


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


# Truthy spellings accepted by launch's IfCondition, shared by the C++
# selection guard so the guard cannot disagree with the node conditions.
_ENABLED = {"1", "true", "yes", "on"}


def announce_cpp_flight(context, *args, **kwargs):
    """Say plainly which adapter is about to fly, and how proven it is."""
    cpp_mode = LaunchConfiguration("cpp_mode").perform(context).lower() in _ENABLED
    cpp_shadow = LaunchConfiguration("cpp_shadow").perform(context).lower() in _ENABLED
    auto_arm = LaunchConfiguration("auto_arm").perform(context).lower() in _ENABLED
    if not cpp_mode:
        if cpp_shadow:
            return [LogInfo(msg=(
                "FLIGHT AUTHORITY: Python adapter. The non-commanding C++ "
                "shadow runs alongside for CPU/parity measurement."
            ))]
        return []
    messages = [LogInfo(msg=(
        "FLIGHT AUTHORITY: C++ adapter (cpp_flight_adapter). Its "
        "command math is parity-tested against the Python limiters to 1e-9; "
        "its state machine and watchdogs are NOT covered by that test."
    ))]
    if auto_arm:
        messages.append(LogInfo(msg=(
            "ARMED FLIGHT ON THE C++ ADAPTER. Fly the props-off dry run "
            "(auto_arm:=false) first if this build has not been dry-run yet. "
            "Keep the RC kill switch (ch9) ready; K force-disarms."
        )))
    return messages


def generate_launch_description():
    default_bag = timestamped_bag("offboard_global")
    arguments = [
        DeclareLaunchArgument("auto_arm", default_value="false"),
        # The planner/follower inputs update at 2/10 Hz and observed VIO is
        # around 11-13 Hz. 20 Hz retains a wide PX4 offboard-stream margin
        # while avoiding the former 50 Hz repetition of adapter work.
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        # MASTER C++ TOGGLE. One flag flips every node in this launch that has
        # a C++ port; the per-node flags below default to it, so
        # `cpp_nodes:=true` is the whole switch and a single node can still be
        # pinned back to Python (`cpp_nodes:=true cpp_mode:=false`) to bisect a
        # regression. Spell it the same way on global_planner_monitor.launch.py
        # to move the whole flight stack together.
        DeclareLaunchArgument("cpp_nodes", default_value="false"),
        # Selection contract: false starts the Python adapter; true replaces
        # that process with the C++ one. Both command PX4 and take the same
        # parameters. The C++ command math is parity-tested against the Python
        # limiters to 1e-9; its state machine and watchdogs are not.
        DeclareLaunchArgument("cpp_mode", default_value=LaunchConfiguration("cpp_nodes")),
        # Parity/CPU measurement. Runs the C++ adapter *alongside* the Python
        # one, which keeps Python as the sole flight authority while
        # process_monitor reports both processes' CPU. This is the only way to
        # get a C++ CPU number in flight until cpp_mode is airworthy.
        DeclareLaunchArgument("cpp_shadow", default_value="false"),
        DeclareLaunchArgument("hover_height", default_value="0.40"),
        DeclareLaunchArgument("climb_timeout", default_value="15.0"),
        # Altitude ramp + vz feedforward (see OffboardHover.ramp_z). climb_rate
        # 0 restores the pre-2026-08-28 position step that took 20.4 s to climb
        # 0.30 m.
        DeclareLaunchArgument("climb_rate", default_value="0.25"),
        DeclareLaunchArgument("climb_leash", default_value="0.12"),
        DeclareLaunchArgument("climb_feedforward", default_value="true"),
        # Horizontal equivalent. Default since the 2026-08-28 03:5x runs;
        # false restores the position-only command.
        DeclareLaunchArgument("horizontal_feedforward", default_value="true"),
        DeclareLaunchArgument("max_flight_time", default_value="45.0"),
        DeclareLaunchArgument("perf_monitor", default_value="true"),
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
        # Carry speed through bends instead of stopping at each one.
        DeclareLaunchArgument("corner_blending", default_value="false"),
        DeclareLaunchArgument("junction_deviation", default_value="0.05"),
        DeclareLaunchArgument("climb_release", default_value="0.05"),
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
    # One dict, both adapters. The C++ port declares exactly the same
    # parameter names as the Python one, so sharing this is what keeps a
    # cpp_mode flight flying the configuration the launch line asked for.
    flight_parameters = {
        "auto_arm": typed("auto_arm", bool),
        "rate_hz": typed("rate_hz", float),
        "hover_height": typed("hover_height", float),
        "climb_timeout": typed("climb_timeout", float),
        "climb_rate": typed("climb_rate", float),
        "climb_leash": typed("climb_leash", float),
        "climb_feedforward": typed("climb_feedforward", bool),
        "horizontal_feedforward": typed("horizontal_feedforward", bool),
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
        "corner_blending": typed("corner_blending", bool),
        "junction_deviation": typed("junction_deviation", float),
        "climb_release": typed("climb_release", float),
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
    }
    node = Node(
        package="px4_vio_bridge",
        executable="offboard_global_planner",
        name="offboard_global_planner",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("cpp_mode")),
        parameters=[flight_parameters],
    )
    cpp_node = Node(
        package="px4_vio_bridge",
        executable="cpp_flight_adapter",
        name="cpp_flight_adapter",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("cpp_mode")),
        parameters=[flight_parameters],
    )
    cpp_shadow = Node(
        package="px4_vio_bridge",
        executable="cpp_clearance_shadow",
        name="cpp_clearance_shadow",
        output="screen",
        condition=IfCondition(LaunchConfiguration("cpp_shadow")),
        parameters=[{
            "rate_hz": typed("rate_hz", float),
        }],
    )
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--storage", "mcap",
            "--storage-preset-profile", STORAGE_PRESET,
            "--disable-keyboard-controls", "--polling-interval", "100",
            "--topics", *FLIGHT_RECORD_TOPICS,
            "--output", LaunchConfiguration("bag_output"),
        ],
        name="global_planner_flight_recorder",
        output="screen",
        sigterm_timeout="20",
        sigkill_timeout="20",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    # Started with the recorder rather than with the flight node, so the CPU /
    # memory trace covers the whole recorded window and a late setpoint can be
    # checked against Pi load on the same time base.
    perf_monitor = Node(
        package="px4_vio_bridge",
        executable="process_monitor",
        name="process_monitor",
        output="log",
        condition=IfCondition(LaunchConfiguration("perf_monitor")),
    )
    start = TimerAction(period=3.0, actions=[node, cpp_node, cpp_shadow])
    stop_after_node = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(
                reason="global planner flight node finished; finalizing bag"
            ))],
        ),
        condition=UnlessCondition(LaunchConfiguration("cpp_mode")),
    )
    stop_after_cpp = RegisterEventHandler(
        OnProcessExit(
            target_action=cpp_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason="C++ global planner flight node finished; finalizing bag"
            ))],
        ),
        condition=IfCondition(LaunchConfiguration("cpp_mode")),
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
        OpaqueFunction(function=announce_cpp_flight),
        OpaqueFunction(function=write_run_marker),
        recorder,
        perf_monitor,
        stop_after_node,
        stop_after_cpp,
        stop_if_recorder_exits,
        start,
    ])
