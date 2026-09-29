"""Command the full unknown-aware 3D reference route.

Attach to ros_planner_3d_monitor. This launch owns PX4 output; auto_arm defaults
false. The visual odometry bridge always consumes continuous raw odometry.
"""
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent,
    LogInfo, OpaqueFunction, TimerAction,
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

FLIGHT_RECORD_TOPICS = (
    '/rosout', '/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint',
    '/fmu/in/vehicle_command', '/fmu/in/vehicle_visual_odometry',
    '/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_control_mode',
    '/fmu/out/vehicle_command_ack', '/fmu/out/vehicle_status_v1',
    '/fmu/out/vehicle_land_detected', '/fmu/out/failsafe_flags',
    '/fmu/out/battery_status_v1', '/fmu/out/sensor_combined',
    '/rtabmap/vio_odometry', '/rtabmap/vio_pose', '/rtabmap/vio_odom_info_lite',
    # OctoMap plus paired metadata is the independent command-audit evidence.
    # MapData is a high-volume planner-replay input and is intentionally left
    # to an explicit diagnostic recording, not the time-critical flight bag.
    '/rtabmap/octomap', '/rtabmap/octomap_metadata',
    '/planner3d/pose_snapshot', '/planner3d/reference_snapshot',
    '/planner3d/reference_path', '/planner3d/path_snapshot', '/planner3d/status',
    '/planner3d/flight/status', '/planner3d/flight/config', '/planner3d/flight/teleop',
    '/waypoint/clicked', '/tf_static', '/perf/processes', '/perf/cpu_percent',
    '/perf/cpu_temp_c', '/perf/throttled', '/perf/mem_percent',
)


def existing_bag_recorders(proc_root=Path('/proc')):
    """Return live `ros2 bag record` processes without requiring psutil."""
    recorders = []
    try:
        entries = proc_root.iterdir()
    except OSError:
        return recorders
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            argv = [part.decode(errors='replace') for part in
                    (entry / 'cmdline').read_bytes().split(b'\0') if part]
        except (OSError, PermissionError):
            continue
        if any(argv[index:index + 2] == ['bag', 'record']
               for index in range(len(argv) - 1)):
            recorders.append((int(entry.name), ' '.join(argv)))
    return sorted(recorders)


def validate_flight_start(context):
    speed = float(LaunchConfiguration('command_speed').perform(context))
    if not math.isfinite(speed) or speed <= 0.0:
        raise RuntimeError('route_speed/command_speed must be finite and positive')
    recorders = existing_bag_recorders()
    if recorders:
        details = '; '.join(f'PID {pid}: {command}' for pid, command in recorders)
        raise RuntimeError(
            'Refusing 3D flight start while another ros2 bag recorder is alive. '
            'Stop the old recorder and keep the monitor at record_bag:=false. ' + details)
    return []


def generate_launch_description():
    defaults = {
        'auto_arm': 'false', 'start_vio_bridge': 'true', 'record_bag': 'true',
        # The ROS 3D backend starts cold on the Pi. Give the VIO bridge time to
        # establish PX4 external-vision fusion before the adapter starts its
        # own 10-second local-position watchdog. This mirrors the proven 2D
        # flow, where the bridge is already warm before the flight launch.
        'adapter_start_delay': '5.0',
        'bag_output': str(timestamped_bag('offboard_planner_3d_unknown')),
        # Use the same takeoff profile that has flown successfully in the 2D
        # adapter. Route motion keeps its separate conservative 3D limits.
        'hover_height': '0.30', 'min_height': '0.15', 'max_height': '1.25',
        'climb_rate': '0.25', 'climb_leash': '0.12', 'climb_release': '0.05',
        'climb_feedforward': 'true',
        'geofence_radius': '3.0', 'required_clearance': '0.35',
        # route_speed is the operator-facing control. command_speed remains a
        # compatible override for existing scripts and is what the node sees.
        'route_speed': '0.15', 'command_speed': LaunchConfiguration('route_speed'),
        'command_vertical_speed': '0.05',
        'command_acceleration': '0.30', 'command_vertical_acceleration': '0.20',
        # 10 Hz remains above PX4's offboard requirement and halves repeated
        # swept-chord work relative to the previous 20 Hz flight loop.
        'rate_hz': '10.0', 'climb_timeout': '15.0', 'max_flight_time': '180.0',
        'planner_fault_land_time': '3.0', 'goal_hold_time': '3.0',
        # The flight backend now bounds VIO queueing to about 0.12 s p95.
        # Retain the original fail-closed 0.30 s source-time budget rather than
        # masking a future perception backlog with a wider acceptance window.
        'map_timeout': '3.0', 'pose_timeout': '0.5', 'max_vio_source_age': '0.30',
        'path_timeout': '3.5', 'px4_timeout': '0.50',
        'vehicle_control_mode_timeout': '0.75', 'follower_rejoin_grace': '1.5',
        'route_equivalence_tolerance': '0.10',
        'require_px4_preflight_status': 'true', 'px4_preflight_timeout': '1.0',
        'vio_info_topic': '/rtabmap/vio_odom_info_lite',
        'vio_yaw_offset_deg': '0.0', 'min_vio_features': '20',
        # Match the 2D adapter: an existing goal may be used, but only after the
        # planner publishes a new coherent route snapshot from the hover state.
        # Set true to require a second user click after takeoff.
        'require_goal_after_hover': 'false', 'yaw_follows_heading': 'true',
        'yaw_rate_deg': '20.0', 'keyboard_kill': 'true', 'keyboard_land': 'true',
        'foxglove_teleop': 'true',
        # Opt-in hold damping (gain 0.75, 0.10 m/s2 cap): opposes measured drift
        # only while ROUTE holds a frozen command. Default OFF until flown.
        'brake_damping': 'false',
        # Hold the line (OFF by default until flown): keep the route across
        # replans that only differ in their drone-anchored first metres, and pull
        # the command back toward the route. Path carrot: the command moves along
        # the route and slows for corners instead of stopping at every vertex.
        # 2026-09-29 planner-in-loop replay (8 flights x 5 disturbance phases):
        # hold alone does not help; path_carrot:=true hold_line_skip:=0.35
        # cross_track_gain:=0.3 cross_track_correction_max:=0.05 reached the goal
        # 39/40 vs 34/40, 13% faster, holds 0.95 -> 0.06 s, time inside 0.30 m
        # clearance 0.86 -> 0.12 s; lateral tracking unchanged (~20 cm p95).
        'hold_line_skip': '0.0', 'hold_line_max_offset': '0.10',
        'cross_track_gain': '0.0', 'cross_track_correction_max': '0.08',
        'path_carrot': 'false', 'corner_speed_min': '0.03', 'max_carrot_lead': '0.35',
    }
    booleans = {'auto_arm', 'climb_feedforward', 'require_goal_after_hover', 'yaw_follows_heading',
                'require_px4_preflight_status',
                'keyboard_kill', 'keyboard_land', 'foxglove_teleop', 'brake_damping', 'path_carrot'}
    strings = {'vio_info_topic'}
    excluded = {'start_vio_bridge', 'record_bag', 'bag_output', 'adapter_start_delay',
                'route_speed'}
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=(
        bool if k in booleans else str if k in strings else int if k == 'min_vio_features' else float))
        for k in defaults if k not in excluded}
    params.update({'allow_unknown': True,
                   'path_snapshot_topic': '/planner3d/reference_snapshot',
                   'foxglove_teleop_topic': '/planner3d/flight/teleop'})
    adapter = Node(package='px4_vio_bridge', executable='cpp_flight_adapter_3d',
                   name='cpp_flight_adapter_3d', output='screen', emulate_tty=True,
                   parameters=[params])
    bridge = Node(package='px4_vio_bridge', executable='vio_to_px4_odometry',
                  name='vio_to_px4_odometry', output='screen',
                  condition=IfCondition(LaunchConfiguration('start_vio_bridge')),
                  parameters=[{'input_odometry_topic': '/rtabmap/vio_odometry',
                               'input_pose_topic': '/rtabmap/vio_pose',
                               'frame_transform': 'enu_flu_to_ned_frd',
                               'max_raw_odometry_age': ParameterValue(
                                   LaunchConfiguration('max_vio_source_age'), value_type=float),
                               'vio_yaw_offset_deg': ParameterValue(
                                   LaunchConfiguration('vio_yaw_offset_deg'), value_type=float)}])
    recorder = ExecuteProcess(
        cmd=[DURABLE_RECORD, '--', 'ros2', 'bag', 'record', '--storage', 'mcap', '--storage-preset-profile',
             'fastwrite', '--disable-keyboard-controls', '--output',
             LaunchConfiguration('bag_output'), '--topics', *FLIGHT_RECORD_TOPICS],
        output='screen', sigterm_timeout='90', sigkill_timeout='90',
        condition=IfCondition(LaunchConfiguration('record_bag')))
    stop_adapter = RegisterEventHandler(OnProcessExit(target_action=adapter,
        on_exit=[EmitEvent(event=Shutdown(reason='3D flight adapter stopped'))]))
    stop_bridge = RegisterEventHandler(OnProcessExit(target_action=bridge,
        on_exit=[EmitEvent(event=Shutdown(reason='VIO bridge stopped'))]),
        condition=IfCondition(LaunchConfiguration('start_vio_bridge')))
    stop_recorder = RegisterEventHandler(OnProcessExit(target_action=recorder,
        on_exit=[EmitEvent(event=Shutdown(reason='3D flight recorder stopped'))]),
        condition=IfCondition(LaunchConfiguration('record_bag')))
    delayed_adapter = TimerAction(
        period=LaunchConfiguration('adapter_start_delay'), actions=[adapter])
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        LogInfo(msg=[
            '3D FLIGHT: unknown traversable, occupied blocked. VIO bridge warm-up ',
            LaunchConfiguration('adapter_start_delay'),
            ' s; route speed ', LaunchConfiguration('command_speed'),
            ' m/s; auto-arm defaults false; K=kill, L=land.',
        ]),
        OpaqueFunction(function=validate_flight_start),
        stop_adapter, stop_bridge, stop_recorder, recorder, bridge, delayed_adapter,
    ])
