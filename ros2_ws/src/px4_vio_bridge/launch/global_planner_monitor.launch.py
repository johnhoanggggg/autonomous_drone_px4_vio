"""Launch the observation-only global A* monitor, optionally with a simulator."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    LogInfo,
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
        DeclareLaunchArgument("cpp_nodes", default_value="false"),
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
        DeclareLaunchArgument("occupied_threshold", default_value="65"),
        DeclareLaunchArgument("robot_radius", default_value="0.25"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),
        DeclareLaunchArgument("inflation_extra", default_value="0.20"),
        DeclareLaunchArgument("start_recovery_radius", default_value="0.30"),
        DeclareLaunchArgument("heuristic_weight", default_value="1.0"),
        DeclareLaunchArgument("planning_timeout_ms", default_value="100.0"),
        DeclareLaunchArgument("route_follower", default_value="true"),
        DeclareLaunchArgument("follower_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("lookahead", default_value="0.60"),
        DeclareLaunchArgument("lookahead_step", default_value="0.05"),
        DeclareLaunchArgument("min_lookahead", default_value="0.05"),
        DeclareLaunchArgument("max_carrot_speed", default_value="0.10"),
        DeclareLaunchArgument("max_carrot_acceleration", default_value="0.30"),
        DeclareLaunchArgument("max_cross_track", default_value="0.60"),
        DeclareLaunchArgument("cross_track_resume", default_value="0.05"),
        DeclareLaunchArgument("cross_track_recovery_time", default_value="1.0"),
        DeclareLaunchArgument("vio_timeout", default_value="0.5"),
        DeclareLaunchArgument("correction_timeout", default_value="1.0"),
        DeclareLaunchArgument("max_correction_m", default_value="0.50"),
        DeclareLaunchArgument("max_correction_yaw_deg", default_value="15.0"),
        DeclareLaunchArgument("switch_improvement", default_value="0.10"),
        DeclareLaunchArgument("path_retain_tolerance", default_value="0.35"),
        DeclareLaunchArgument("path_head_margin", default_value="0.50"),
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
        "occupied_threshold": typed("occupied_threshold", int),
        "robot_radius": typed("robot_radius", float),
        "safety_margin": typed("safety_margin", float),
        "lookahead": typed("lookahead", float),
        "lookahead_step": typed("lookahead_step", float),
        "min_lookahead": typed("min_lookahead", float),
        "max_carrot_speed": typed("max_carrot_speed", float),
        "max_carrot_acceleration": typed("max_carrot_acceleration", float),
        "max_cross_track": typed("max_cross_track", float),
        "cross_track_resume": typed("cross_track_resume", float),
        "cross_track_recovery_time": typed("cross_track_recovery_time", float),
        "vio_timeout": typed("vio_timeout", float),
        "correction_timeout": typed("correction_timeout", float),
        "max_correction_m": typed("max_correction_m", float),
        "max_correction_yaw_deg": typed("max_correction_yaw_deg", float),
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
            "/rtabmap/odom_correction", "/rtabmap/vio_feature_count",
            "/waypoint/clicked",
            "/planner/path", "/planner/candidate_path", "/planner/inflated_map",
            "/planner/markers", "/planner/status", "/planner/config",
            "/planner/planning_ms",
            "/planner/path_length", "/planner/expanded_cells",
            "/planner/goal_exact", "/planner/goal_terminal",
            "/planner/effective_goal",
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
