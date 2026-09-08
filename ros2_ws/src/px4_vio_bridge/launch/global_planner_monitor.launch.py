"""Launch the observation-only global A* monitor, optionally with a simulator."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from px4_vio_bridge.log_paths import timestamped_bag


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def check_clearance_budget(context, *args, **kwargs):
    """Refuse to start a combination of tolerances that cannot be flown.

    These four numbers are declared in one file but consumed by two nodes, and
    nothing at runtime compares them. The two flights on 20260831 were flown at
    max_cross_track 0.15 with path_retain_tolerance 0.12 and safety_margin 0.00,
    which is:

      * a 0.03 m band between "rebuild the path" and "fault", narrower than a
        single measured loop-closure correction (118 mm), so a correction
        faulted where it should have replanned; and
      * permission for the airframe to sit 0.15 m inside an obstacle, in a
        corridor whose narrowest gap left 0.09 m of slack per side.

    Checked at launch because that is the only place both nodes' parameters are
    visible at once.
    """
    def value(name):
        return float(LaunchConfiguration(name).perform(context))

    max_cross_track = value("max_cross_track")
    retain = value("path_retain_tolerance")
    resume = value("cross_track_resume")
    margin = value("safety_margin")
    radius = value("robot_radius")

    errors = []
    if resume >= max_cross_track:
        errors.append(
            f"cross_track_resume ({resume:.3f}) must be below "
            f"max_cross_track ({max_cross_track:.3f}); recovery could never close"
        )
    if retain >= max_cross_track:
        errors.append(
            f"path_retain_tolerance ({retain:.3f}) must be below "
            f"max_cross_track ({max_cross_track:.3f}); the follower would fault "
            f"before the planner rebuilt the path"
        )
    if errors:
        return [LogInfo(msg="ERROR: " + e) for e in errors] + [
            EmitEvent(event=Shutdown(reason="clearance budget is not flyable"))
        ]

    messages = []
    if margin < max_cross_track:
        # A trade, not a bug: raising the margin costs corridor width. State the
        # penetration it buys so the number is chosen rather than inherited.
        # Worst case the vehicle centre sits (lethal_radius - max_cross_track)
        # from an obstacle, so the airframe is inside it by
        # robot_radius - (lethal_radius - max_cross_track), which reduces to
        # max_cross_track - safety_margin.
        messages.append(LogInfo(msg=(
            f"clearance budget: contact is POSSIBLE -- the airframe may reach "
            f"{max_cross_track - margin:.3f} m inside an obstacle "
            f"(max_cross_track {max_cross_track:.2f} - safety_margin {margin:.2f}); "
            f"worst-case centre-to-obstacle {radius + margin - max_cross_track:.3f} m "
            f"against a {radius:.2f} m airframe radius. "
            f"safety_margin >= max_cross_track is what makes contact impossible."
        )))
    messages.append(LogInfo(msg=(
        f"clearance budget: min plannable gap {2 * (radius + margin):.2f} m, "
        f"replan at {retain:.2f} m, fault at {max_cross_track:.2f} m, "
        f"resume below {resume:.2f} m"
    )))
    return messages


def recorder_exited(event, _context):
    # ros2 bag refuses to overwrite an existing directory. Do not let the
    # planner continue and give the operator an apparently successful but
    # unrecorded run when the recorder failed at startup.
    if event.returncode in (0, -2, -15):
        return []
    return [
        LogInfo(msg=f"ERROR: bag recorder exited with code {event.returncode}; stopping this run"),
        EmitEvent(event=Shutdown(reason="bag recorder failed")),
    ]


def required_node_exited(event, context):
    if context.is_shutdown:
        return []
    return [
        LogInfo(msg=(
            f"ERROR: required planner process '{event.process_name}' exited "
            f"with code {event.returncode}; stopping this launch"
        )),
        EmitEvent(event=Shutdown(reason="required planner process exited")),
    ]


def generate_launch_description():
    args = [
        # MASTER C++ TOGGLE. One flag flips every node in this launch that has
        # a C++ port; the per-node flags below default to it, so
        # `cpp_nodes:=true` is the whole switch and a single node can still be
        # pinned back to Python (`cpp_nodes:=true cpp_astar:=false`) to bisect a
        # regression. Spell it the same way on offboard_global_planner.launch.py
        # to move the whole flight stack together.
        DeclareLaunchArgument("cpp_nodes", default_value="true"),
        # The A* planner. The C++ port is parity-tested against grid_planner.py
        # over randomized maps (costmap, start recovery, goal selection, A*
        # cells, expansions and the simplified path all compared exactly) and
        # takes the same singleton lock as the Python node, so the two can never
        # both drive /planner/path.
        DeclareLaunchArgument("cpp_astar", default_value=LaunchConfiguration("cpp_nodes")),
        # Observation-only route follower. Both implementations publish the
        # same topics and share one singleton lock.
        DeclareLaunchArgument("cpp_follower", default_value=LaunchConfiguration("cpp_nodes")),
        DeclareLaunchArgument("simulate", default_value="false"),
        DeclareLaunchArgument("dynamic_obstacle", default_value="true"),
        DeclareLaunchArgument("rate_hz", default_value="2.0"),
        DeclareLaunchArgument("map_timeout", default_value="3.0"),
        DeclareLaunchArgument("pose_timeout", default_value="1.0"),
        DeclareLaunchArgument("pose_topic", default_value="/rtabmap/body_pose"),
        DeclareLaunchArgument(
            "raw_vio_topic", default_value="/rtabmap/body_vio_pose"
        ),
        DeclareLaunchArgument("occupied_threshold", default_value="65"),
        DeclareLaunchArgument("robot_radius", default_value="0.25"),
        # Sets BOTH the planner's lethal radius and the follower's required
        # clearance: each is robot_radius + this.
        #
        # Tight-space profile, chosen 2026-08-31 after the 064x flights. At
        # 0.30 m of lethal radius the neck on 20260831T064828Z (0.266-0.294 m of
        # clearance) is not routable and 44% of that flight's track sat inside
        # the requirement; back at 0.25 m the necks and the 0.68 m doorway are
        # both flyable. The cost is that contact stays possible: worst case the
        # airframe reaches max_cross_track (0.08 m) inside an obstacle, about
        # half the 0.15 m budget it hit the door frame under on 20260831T033736Z.
        #
        # 2026-09-08: that cost was being paid. ULogs 22_41_51/22_42_49 and
        # 23_21_06/23_22_16/23_23_08 carry twelve |a_xy| > 5 m/s2 contacts over
        # five flights, peaking at 3.32 g. Aligned to the bags (position rms
        # 0.001-0.004 m) every wall contact sat at 0.212-0.297 m of pose
        # clearance with cross-track 0.03-0.12 m -- the vehicle was tracking its
        # route, and the route was in the wall. Across those flights the 251
        # published routes ran to a median min clearance of 0.335 m and a p10 of
        # 0.283 m, against a 0.25 m requirement with no margin for the
        # airframe's own 0.08-0.12 m of tracking error.
        #
        # 0.03 puts the requirement at 0.28 m. On those same 251 routes 90%
        # still plan, against 73% at the 0.30 m that 2026-08-31 rejected. It
        # trims the worst routes rather than eliminating contact -- worst-case
        # penetration falls from 0.08 to 0.05 m -- so treat it as a reduction,
        # not a fix, and re-measure the contact count before trusting it.
        DeclareLaunchArgument("safety_margin", default_value="0.03"),
        DeclareLaunchArgument("inflation_extra", default_value="0.25"),
        DeclareLaunchArgument("start_recovery_radius", default_value="0.30"),
        DeclareLaunchArgument("heuristic_weight", default_value="1.0"),
        DeclareLaunchArgument("planning_timeout_ms", default_value="100.0"),
        DeclareLaunchArgument("route_follower", default_value="true"),
        DeclareLaunchArgument("follower_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("lookahead", default_value="0.25"),
        DeclareLaunchArgument("lookahead_step", default_value="0.03"),
        DeclareLaunchArgument("min_lookahead", default_value="0.05"),
        DeclareLaunchArgument("max_carrot_speed", default_value="0.20"),
        DeclareLaunchArgument("max_carrot_acceleration", default_value="0.30"),
        # The corridor, not comfort, sets this. At lethal_radius 0.30 the
        # doorway leaves 0.09 m of slack per side, and the old 0.15 m budget was
        # wider than the gap. It is also below the airframe's own p90 horizontal
        # hold error (0.15-0.16 m on 20260831), so faults get MORE frequent, not
        # less -- that is only tenable with the climb-phase grace and the
        # filtered recovery gate below.
        DeclareLaunchArgument("max_cross_track", default_value="0.08"),
        # Ceiling the fault gate may open to where the pose has clearance to
        # spare (pose clearance - robot_radius - safety_margin). max_cross_track
        # stays the floor, so this never gates tighter than the fixed threshold.
        #
        # A fixed threshold answers "am I about to hit something" without
        # knowing how much room there is. Over 20260908T064933Z-T065344Z the
        # gate fired ten times and six were at 0.40-0.63 m of clearance, with
        # 0.15-0.38 m of bubble to spare and nothing in reach. Each of those
        # stops replanned from where the vehicle stopped, re-aiming the heading
        # and driving the next excursion; T065022Z, the one flight that stayed
        # under the gate (peak 0.075 m), had no faults and reached its goal.
        #
        # The widened branch cannot put the bubble closer to an obstacle than
        # required_clearance: the vehicle may sit `gate` off the route, the
        # route has `headroom + required` of clearance, and gate <= headroom.
        # Set at or below max_cross_track to restore the fixed gate.
        DeclareLaunchArgument("cross_track_clearance_cap", default_value="0.20"),
        DeclareLaunchArgument("cross_track_resume", default_value="0.04"),
        DeclareLaunchArgument("cross_track_recovery_time", default_value="0.6"),
        # Applies to the recovery gate only, never to the fault threshold.
        DeclareLaunchArgument("cross_track_filter_time_constant", default_value="0.30"),
        DeclareLaunchArgument("vio_timeout", default_value="0.5"),
        DeclareLaunchArgument("correction_timeout", default_value="1.0"),
        DeclareLaunchArgument("max_correction_m", default_value="1.0"),
        DeclareLaunchArgument("max_correction_yaw_deg", default_value="10.0"),
        DeclareLaunchArgument("switch_improvement", default_value="0.10"),
        # Must stay a full correction jump below max_cross_track, or a loop
        # closure faults instead of replanning. Measured material corrections
        # reached 118 mm on 20260831T033557Z.
        DeclareLaunchArgument("path_retain_tolerance", default_value="0.04"),
        DeclareLaunchArgument("path_head_margin", default_value="0.50"),
        # Distinct occupancy grids that must agree before a semantic mode change
        # (PATH_VALID / SAFE_APPROACH / EXPLORING) is committed. Counted on map
        # generations, not planner ticks.
        DeclareLaunchArgument("mode_confirmation_maps", default_value="2"),
        # Clearance an escape must gain at its endpoint before it counts as
        # recovery rather than sliding along the same unsafe contour.
        DeclareLaunchArgument("escape_minimum_improvement", default_value="0.01"),
        # Replaces the old blind 8 s correction cooldown.
        DeclareLaunchArgument("correction_rearm_guard", default_value="0.20"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument("bag_output", default_value=timestamped_bag("global_planner_monitor")),
    ]
    simulator = Node(
        package="px4_vio_bridge",
        executable="global_planner_sim",
        name="global_planner_sim",
        output="screen",
        condition=IfCondition(LaunchConfiguration("simulate")),
        parameters=[{"dynamic_obstacle": typed("dynamic_obstacle", bool)}],
    )
    planner_parameters = [{
        "rate_hz": typed("rate_hz", float),
        "map_timeout": typed("map_timeout", float),
        "pose_timeout": typed("pose_timeout", float),
        "pose_topic": typed("pose_topic", str),
        "occupied_threshold": typed("occupied_threshold", int),
        "robot_radius": typed("robot_radius", float),
        "safety_margin": typed("safety_margin", float),
        "inflation_extra": typed("inflation_extra", float),
        "start_recovery_radius": typed("start_recovery_radius", float),
        "heuristic_weight": typed("heuristic_weight", float),
        "planning_timeout_ms": typed("planning_timeout_ms", float),
        "switch_improvement": typed("switch_improvement", float),
        "path_retain_tolerance": typed("path_retain_tolerance", float),
        "path_head_margin": typed("path_head_margin", float),
        "mode_confirmation_maps": typed("mode_confirmation_maps", int),
        "correction_timeout": typed("correction_timeout", float),
        "max_correction_m": typed("max_correction_m", float),
        "max_correction_yaw_deg": typed("max_correction_yaw_deg", float),
    }]
    monitor = Node(
        package="px4_vio_bridge",
        executable="global_planner_monitor",
        name="global_planner_monitor",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("cpp_astar")),
        parameters=planner_parameters,
    )
    cpp_monitor = Node(
        package="px4_vio_bridge",
        executable="cpp_astar_planner",
        name="global_planner_monitor",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("cpp_astar")),
        parameters=planner_parameters,
    )
    follower_parameters = [{
        "rate_hz": typed("follower_rate_hz", float),
        "map_timeout": typed("map_timeout", float),
        "pose_topic": typed("pose_topic", str),
        "raw_vio_topic": typed("raw_vio_topic", str),
        "occupied_threshold": typed("occupied_threshold", int),
        "robot_radius": typed("robot_radius", float),
        "safety_margin": typed("safety_margin", float),
        "lookahead": typed("lookahead", float),
        "lookahead_step": typed("lookahead_step", float),
        "min_lookahead": typed("min_lookahead", float),
        "max_carrot_speed": typed("max_carrot_speed", float),
        "max_carrot_acceleration": typed("max_carrot_acceleration", float),
        "max_cross_track": typed("max_cross_track", float),
        "cross_track_clearance_cap": typed("cross_track_clearance_cap", float),
        "cross_track_resume": typed("cross_track_resume", float),
        "cross_track_recovery_time": typed("cross_track_recovery_time", float),
        "cross_track_filter_time_constant": typed(
            "cross_track_filter_time_constant", float),
        "vio_timeout": typed("vio_timeout", float),
        "correction_timeout": typed("correction_timeout", float),
        "max_correction_m": typed("max_correction_m", float),
        "max_correction_yaw_deg": typed("max_correction_yaw_deg", float),
        "escape_minimum_improvement": typed("escape_minimum_improvement", float),
        "correction_rearm_guard": typed("correction_rearm_guard", float),
    }]
    follower = Node(
        package="px4_vio_bridge",
        executable="route_follower_monitor",
        name="route_follower_monitor",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("cpp_follower")),
        parameters=follower_parameters,
    )
    cpp_follower = Node(
        package="px4_vio_bridge",
        executable="cpp_route_follower",
        name="route_follower_monitor",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("cpp_follower")),
        parameters=follower_parameters,
    )
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--storage", "mcap",
            "--storage-preset-profile", "fastwrite",
            "--disable-keyboard-controls",
            "--topics",
            "/rtabmap/grid", "/rtabmap/pose", "/rtabmap/vio_pose",
            "/rtabmap/body_pose", "/rtabmap/body_vio_pose",
            "/rtabmap/body_pose/config",
            "/rtabmap/odom_correction", "/rtabmap/vio_feature_count",
            "/waypoint/clicked",
            "/planner/path", "/planner/candidate_path", "/planner/inflated_map",
            "/planner/markers", "/planner/status", "/planner/config",
            "/planner/planning_ms",
            "/planner/path_length", "/planner/expanded_cells",
            "/planner/goal_exact", "/planner/goal_terminal",
            "/planner/effective_goal",
            # Generation pairing and correction episodes: without these a bag
            # cannot show why a post-correction hold was or was not released.
            "/planner/map_generation", "/planner/path_map_generation",
            "/planner/correction_epoch",
            "/planner/follower/carrot", "/planner/follower/lookahead",
            "/planner/follower/displacement", "/planner/follower/status",
            "/planner/follower/config",
            "/planner/follower/valid",
            "/planner/follower/vio_displacement",
            "/planner/follower/goal_reached",
            "/planner/follower/progress", "/planner/follower/remaining",
            "/planner/follower/path_progress",
            "/planner/follower/cross_track", "/planner/follower/path_generation",
            "/planner/follower/markers",
            "--output", LaunchConfiguration("bag_output"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    recorder_watchdog = RegisterEventHandler(
        OnProcessExit(target_action=recorder, on_exit=recorder_exited)
    )
    monitor_watchdog = RegisterEventHandler(
        OnProcessExit(target_action=monitor, on_exit=required_node_exited)
    )
    # Only one of the two planners ever starts, so the other's handler simply
    # never fires -- but whichever did start must still take the launch down
    # with it, including when it exits on a held singleton lock.
    cpp_monitor_watchdog = RegisterEventHandler(
        OnProcessExit(target_action=cpp_monitor, on_exit=required_node_exited)
    )
    follower_watchdog = RegisterEventHandler(
        OnProcessExit(target_action=follower, on_exit=required_node_exited)
    )
    cpp_follower_watchdog = RegisterEventHandler(
        OnProcessExit(target_action=cpp_follower, on_exit=required_node_exited)
    )
    # Let the planner singleton gate run first. If a stale planner exists, the
    # launch shuts down before starting a follower that would immediately race
    # the stale follower on the flight-validity topics.
    follower_start = TimerAction(period=2.0, actions=[GroupAction(
        condition=IfCondition(LaunchConfiguration("route_follower")),
        actions=[follower, cpp_follower],
    )])
    return LaunchDescription(args + [
        OpaqueFunction(function=check_clearance_budget),
        LogInfo(msg="global planner monitor is observation only; it cannot command PX4"),
        LogInfo(
            msg=["A* planner: C++ (cpp_astar_planner). cpp_astar=",
                 LaunchConfiguration("cpp_astar")],
            condition=IfCondition(LaunchConfiguration("cpp_astar")),
        ),
        LogInfo(
            msg="A* planner: Python (global_planner_monitor).",
            condition=UnlessCondition(LaunchConfiguration("cpp_astar")),
        ),
        LogInfo(
            msg="Route follower: C++ (cpp_route_follower).",
            condition=IfCondition(LaunchConfiguration("cpp_follower")),
        ),
        LogInfo(
            msg="Route follower: Python (route_follower_monitor).",
            condition=UnlessCondition(LaunchConfiguration("cpp_follower")),
        ),
        recorder_watchdog, monitor_watchdog, cpp_monitor_watchdog,
        follower_watchdog, cpp_follower_watchdog,
        recorder, simulator, monitor, cpp_monitor, follower_start,
    ])
