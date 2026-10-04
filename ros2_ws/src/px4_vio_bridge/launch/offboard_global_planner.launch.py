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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

# Wraps `ros2 bag record` so the bag is flushed to the SD card and finalized
# even when the Pi loses power shortly after the flight (see the script).
DURABLE_RECORD = PathJoinSubstitution(
    [FindPackagePrefix("px4_vio_bridge"), "lib", "px4_vio_bridge", "durable_bag_record"])


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
    "/fmu/out/vehicle_acceleration",
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
    "/rtabmap/body_pose",
    "/rtabmap/body_pose/config",
    "/rtabmap/vio_feature_count",
    "/rtabmap/vio_pose",
    "/rtabmap/body_vio_pose",
    "/vio/yaw_offset/pose",
    "/vio/level/tilt_deg",
    # Requested goal and A* outputs needed by evaluate_planner_bags.py.
    "/waypoint/clicked",
    "/planner/candidate_path",
    "/planner/config",
    "/planner/effective_goal",
    "/planner/expanded_cells",
    "/planner/goal_exact",
    "/planner/goal_terminal",
    "/planner/map_generation",
    "/planner/path",
    "/planner/path_map_generation",
    "/planner/correction_epoch",
    "/planner/path_length",
    "/planner/planning_ms",
    "/planner/status",
    # Route-follower geometry, validity and flight-adapter decisions.
    "/planner/follower/carrot",
    "/planner/follower/config",
    "/planner/follower/cross_track",
    # The low-passed value the recovery gate actually tests.
    "/planner/follower/cross_track_filtered",
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
    # Numeric companions to the status string, so an analysis does not have to
    # regex prose out of it, plus the adapter's own effective configuration --
    # corner_blending, the command limits and the climb parameters were
    # previously invisible in a bag.
    "/planner/flight/config",
    "/planner/flight/command_speed",
    "/planner/flight/path_offset",
    "/planner/flight/joining",
    "/planner/flight/heading_debug",
    "/planner/flight/damping_debug",
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
        DeclareLaunchArgument("cpp_nodes", default_value="true"),
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
        DeclareLaunchArgument("hover_height", default_value="0.30"),
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
        # Opt-in velocity-error feedback during validated route braking and
        # normal route holds. The flown lateral-stop setting is 0.75 gain with
        # a 0.10 m/s² correction cap and 0.10 s filter; default OFF preserves
        # the proven tunnel command stream.
        DeclareLaunchArgument("brake_damping", default_value="false"),
        DeclareLaunchArgument("damping_gain", default_value="0.75"),
        DeclareLaunchArgument("damping_limit", default_value="0.10"),
        DeclareLaunchArgument("damping_time_constant", default_value="0.10"),
        DeclareLaunchArgument("damping_velocity_timeout", default_value="0.10"),
        DeclareLaunchArgument("max_flight_time", default_value="60.0"),
        DeclareLaunchArgument("perf_monitor", default_value="true"),
        DeclareLaunchArgument("command_speed", default_value="0.20"),
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
            # The fault-free tunnel flight republished the same physical path
            # with 1.1-2.4 cm map-correction shifts. Three centimetres lets the
            # shared-suffix path preserve speed across those coordinate updates
            # while genuinely different routes still use the bounded rejoin.
            "path_command_suffix_tolerance", default_value="0.03"
        ),
        DeclareLaunchArgument("path_corner_tolerance", default_value="0.05"),
        # Carry speed through bends instead of stopping at each one.
        DeclareLaunchArgument("corner_blending", default_value="true"),
        DeclareLaunchArgument("junction_deviation", default_value="0.05"),
        # EXPERIMENTAL (C++ adapter only). Strict mode stops the command at
        # every path bend even with corner_blending on. This rounds each bend
        # by at most junction_deviation where the arc keeps the corner's
        # clearance, so the command keeps moving; unrounded bends still stop.
        DeclareLaunchArgument("corner_fillets", default_value="false"),
        DeclareLaunchArgument("climb_release", default_value="0.05"),
        DeclareLaunchArgument("route_command_grace", default_value="2.0"),
        DeclareLaunchArgument("replan_during_yaw_align", default_value="false"),
        DeclareLaunchArgument("geofence_radius", default_value="3.0"),
        DeclareLaunchArgument("planner_fault_land_time", default_value="6.0"),
        # A follower cross-track latch is held, not counted toward a landing,
        # while the vehicle sheds the lateral drift the climb puts in.
        DeclareLaunchArgument("route_entry_grace", default_value="3.0"),
        DeclareLaunchArgument("map_pose_topic", default_value="/rtabmap/body_pose"),
        DeclareLaunchArgument("goal_hold_time", default_value="3.0"),
        # A large map correction must never end a flight: the offset grows with
        # session drift (0.8 m before takeoff on 09-30) and loop closures jump it.
        # 100 m / 180 deg never trips; lower them to restore the gate.
        DeclareLaunchArgument("max_correction_m", default_value="100.0"),
        DeclareLaunchArgument("max_correction_yaw_deg", default_value="180.0"),
        # Flight 20260831T033736Z ran below 200 features for 42% of its samples
        # in the doorway approach with nothing raised. Hold rather than commit
        # into a narrow gap when the tracker is starving.
        #
        # Lowered 150 -> 75 on 2026-09-06. At 150 this landed 20260906T113122Z
        # 5.7 s into a route that was tracking well (heading/course median 3.4
        # deg, VIO 13.3 Hz with a 0.103 s worst gap) on `latest=146` -- a 3%
        # dip below the threshold, not a tracking failure.
        DeclareLaunchArgument("min_vio_features", default_value="75"),
        DeclareLaunchArgument("vio_feature_loss_time", default_value="0.25"),
        # A fault hold pins the aircraft where it stands. 20260906T131950Z was
        # held inside its own clearance envelope for 4.5 s and landed on the
        # fault timer; these let the hold walk out of that, bounded.
        # A follower dropout is a blink far more often than a failure: eight
        # episodes across 20260906T135902Z and T140117Z ran 1.00-2.80 s, and
        # stopping dead for each of them cost 14% and 27% of the route. Crawl
        # the accepted path for this long before the hold latches.
        DeclareLaunchArgument("follower_latch_grace", default_value="1.5"),
        DeclareLaunchArgument("degraded_speed", default_value="0.06"),
        DeclareLaunchArgument("fault_escape_step", default_value="0.05"),
        DeclareLaunchArgument("fault_escape_limit", default_value="0.50"),
        DeclareLaunchArgument("max_horizontal_error", default_value="0.35"),
        DeclareLaunchArgument("transit_horizontal_error", default_value="0.60"),
        DeclareLaunchArgument("pre_route_max_horizontal_error", default_value="0.15"),
        # Yaw follows the path heading. yaw_rate_deg is the slew of the published
        # yaw setpoint (the node's own watchdog lands above max_yaw_rate_deg=60);
        # 20 deg/s turns 90 deg in ~4.5s without stepping the mixer.
        DeclareLaunchArgument("yaw_follows_heading", default_value="true"),
        DeclareLaunchArgument("yaw_rate_deg", default_value="20.0"),
        # Preview across short route legs and slew the internal heading target.
        # Both zero restores the point-tangent / stepped-target behavior.
        DeclareLaunchArgument("yaw_heading_lookahead", default_value="0.35"),
        DeclareLaunchArgument("yaw_reference_rate_deg", default_value="20.0"),
        DeclareLaunchArgument("yaw_track_min_displacement", default_value="0.15"),
        DeclareLaunchArgument("yaw_track_deadband_deg", default_value="15.0"),
        DeclareLaunchArgument("yaw_align_error_deg", default_value="40.0"),
        DeclareLaunchArgument("yaw_resume_error_deg", default_value="15.0"),
        # Taper speed from resume error to align error, retaining the hard hold.
        DeclareLaunchArgument("yaw_slowdown", default_value="true"),
        DeclareLaunchArgument("tracking_loss_land", default_value="true"),
        DeclareLaunchArgument("keyboard_kill", default_value="true"),
        DeclareLaunchArgument("keyboard_land", default_value="true"),
        DeclareLaunchArgument("foxglove_teleop", default_value="true"),
        DeclareLaunchArgument(
            "foxglove_teleop_topic", default_value="/planner/flight/teleop"
        ),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("bag_output", default_value=default_bag),
        # Adds the camera to the flight bag. The depthai backend publishes
        # 640x400 JPEG at slam_fps (20 Hz), roughly 0.5-1 MB/s on the SD card.
        # With backend:=ros use /foxglove/camera/image/compressed (5 Hz).
        DeclareLaunchArgument("record_camera", default_value="false"),
        DeclareLaunchArgument(
            "camera_topic", default_value="/rtabmap/image/compressed"
        ),
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
        "brake_damping": typed("brake_damping", bool),
        "damping_gain": typed("damping_gain", float),
        "damping_limit": typed("damping_limit", float),
        "damping_time_constant": typed("damping_time_constant", float),
        "damping_velocity_timeout": typed("damping_velocity_timeout", float),
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
        "corner_fillets": typed("corner_fillets", bool),
        "climb_release": typed("climb_release", float),
        "route_command_grace": typed("route_command_grace", float),
        "replan_during_yaw_align": typed("replan_during_yaw_align", bool),
        "geofence_radius": typed("geofence_radius", float),
        "planner_fault_land_time": typed("planner_fault_land_time", float),
        "route_entry_grace": typed("route_entry_grace", float),
        "map_pose_topic": LaunchConfiguration("map_pose_topic"),
        "goal_hold_time": typed("goal_hold_time", float),
        "max_correction_m": typed("max_correction_m", float),
        "max_correction_yaw_deg": typed("max_correction_yaw_deg", float),
        "min_vio_features": typed("min_vio_features", int),
        "follower_latch_grace": typed("follower_latch_grace", float),
        "degraded_speed": typed("degraded_speed", float),
        "fault_escape_step": typed("fault_escape_step", float),
        "fault_escape_limit": typed("fault_escape_limit", float),
        "vio_feature_loss_time": typed("vio_feature_loss_time", float),
        "max_horizontal_error": typed("max_horizontal_error", float),
        "transit_horizontal_error": typed("transit_horizontal_error", float),
        "pre_route_max_horizontal_error": typed(
            "pre_route_max_horizontal_error", float
        ),
        "yaw_follows_heading": typed("yaw_follows_heading", bool),
        "yaw_rate_deg": typed("yaw_rate_deg", float),
        "yaw_slowdown": typed("yaw_slowdown", bool),
        "yaw_heading_lookahead": typed("yaw_heading_lookahead", float),
        "yaw_reference_rate_deg": typed("yaw_reference_rate_deg", float),
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
            "map_pose_topic": LaunchConfiguration("map_pose_topic"),
        }],
    )
    def flight_recorder(context, *args, **kwargs):
        # Built at launch time so record_camera can add to the topic list.
        topics = list(FLIGHT_RECORD_TOPICS)
        messages = []
        if LaunchConfiguration("record_camera").perform(context).lower() in _ENABLED:
            camera_topic = LaunchConfiguration("camera_topic").perform(context)
            topics.append(camera_topic)
            messages.append(LogInfo(msg=f"Recording camera feed: {camera_topic}"))
        recorder = ExecuteProcess(
            cmd=[
                DURABLE_RECORD, "--",
                "ros2", "bag", "record", "--storage", "mcap",
                "--storage-preset-profile", STORAGE_PRESET,
                "--disable-keyboard-controls", "--polling-interval", "100",
                "--topics", *topics,
                "--output", LaunchConfiguration("bag_output"),
            ],
            name="global_planner_flight_recorder",
            output="screen",
            # A bag whose metadata.yaml is 0 bytes was SIGKILLed before it could
            # write the footer -- it needs `ros2 bag reindex <dir> -s mcap` to
            # open and loses the tail of the flight. Both 20260831T033736Z and
            # T064828Z ended that way, and the first of those hid four
            # door-frame impacts that happened three seconds after its last
            # recorded message. Finalizing competes for cores that RTAB-Map VIO
            # and SLAM already pin, so the old 20 s was not a generous
            # allowance. Landing takes far longer than this costs.
            sigterm_timeout="90",
            sigkill_timeout="90",
        )
        stop_if_recorder_exits = RegisterEventHandler(
            OnProcessExit(
                target_action=recorder,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="flight recorder exited; stopping flight launch"
                ))],
            ),
        )
        return messages + [recorder, stop_if_recorder_exits]

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
    return LaunchDescription(arguments + [
        LogInfo(msg="GLOBAL PLANNER FLIGHT: auto_arm defaults false; K=kill, L=land"),
        LogInfo(msg=["Flight bag output: ", LaunchConfiguration("bag_output")]),
        OpaqueFunction(function=announce_cpp_flight),
        OpaqueFunction(function=write_run_marker),
        OpaqueFunction(
            function=flight_recorder,
            condition=IfCondition(LaunchConfiguration("record_bag")),
        ),
        perf_monitor,
        stop_after_node,
        stop_after_cpp,
        start,
    ])
