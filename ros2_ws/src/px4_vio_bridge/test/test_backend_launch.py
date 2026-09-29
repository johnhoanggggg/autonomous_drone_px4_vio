"""Launch construction/selection only. Never start hardware or flight processes."""
import importlib.util
from pathlib import Path
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('source_count', [1, 2])
def test_pi_backend_library_loading_from_already_sourced_shell(source_count):
    """Load the real message library, but never start ROS nodes or the camera."""
    import subprocess
    base = Path('/home/john/rtabmap_parity_0_21_12/base_ros_env.bash')
    if not base.exists():
        pytest.skip('Pi-specific external overlays are not installed')
    preamble = (
        'source /opt/ros/jazzy/setup.bash && '
        'source /home/john/ros2_ws/install/setup.bash && '
        'source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash && '
    )
    result = subprocess.run(
        ['bash', '-c', preamble * source_count + 'export ROS_DOMAIN_ID=178; exec bash "$1" --check-environment',
         'backend-env-test', str(ROOT/'scripts'/'ros_rtabmap_backend')],
        capture_output=True, text=True, timeout=25)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'environment verified (no graph started)' in result.stdout
    assert 'ROS_DOMAIN_ID=178' in result.stdout


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'launch'/name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


@pytest.mark.parametrize('enabled', [None, 'true'])
def test_lateral_damping_is_opt_in_and_parameters_reach_node(enabled):
    from launch_ros.utilities import evaluate_parameters
    context = LaunchContext()
    if enabled is not None:
        context.launch_configurations.update(brake_damping=enabled, damping_gain='0.4')
    ld = load('offboard_lateral_stop.launch.py')
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    timer = next(a for a in ld.entities if isinstance(a, TimerAction))
    node = next(a for a in timer.actions if isinstance(a, Node))
    # Inspect descriptions only; never execute the timer or flight node.
    params = evaluate_parameters(context, node._Node__parameters)[0]
    assert params['brake_damping'] is (enabled == 'true')
    assert params['damping_gain'] == (0.4 if enabled else 0.5)
    assert params['damping_limit'] == 0.10
    assert params['damping_time_constant'] == 0.10
    assert params['speed'] == 0.20
    assert params['acceleration'] == 0.30
    assert params['auto_arm'] is False


@pytest.mark.parametrize('enabled', [None, 'true'])
def test_global_damping_is_opt_in_and_parameters_reach_both_adapters(enabled):
    from launch_ros.utilities import evaluate_parameters
    context = LaunchContext()
    if enabled is not None:
        context.launch_configurations.update(brake_damping=enabled, damping_gain='0.6')
    ld = load('offboard_global_planner.launch.py')
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    timer = next(action for action in ld.entities if isinstance(action, TimerAction))
    nodes = [action for action in timer.actions if isinstance(action, Node)]
    flight_nodes = [node for node in nodes if node.node_executable in (
        'offboard_global_planner', 'cpp_flight_adapter')]
    assert len(flight_nodes) == 2
    for node in flight_nodes:
        params = evaluate_parameters(context, node._Node__parameters)[0]
        assert params['brake_damping'] is (enabled == 'true')
        assert params['damping_gain'] == (0.6 if enabled else 0.75)
        assert params['damping_limit'] == 0.10
        assert params['damping_time_constant'] == 0.10
        assert params['damping_velocity_timeout'] == 0.10
        assert params['command_acceleration'] == 0.30


def camera_actions():
    ld = load('rtabmap_slam_px4.launch.py')
    timer = next(a for a in ld.entities if isinstance(a, TimerAction))
    group = next(a for a in timer.actions if isinstance(a, GroupAction))
    return group.get_sub_entities()


@pytest.mark.parametrize('backend', ['depthai', 'ros'])
def test_exactly_one_camera_backend_selected(backend):
    context = LaunchContext()
    context.launch_configurations['backend'] = backend
    candidates = [a for a in camera_actions() if isinstance(a, (IncludeLaunchDescription, ExecuteProcess))]
    selected = [a for a in candidates if a.condition.evaluate(context)]
    assert len(selected) == 1
    assert isinstance(selected[0], IncludeLaunchDescription if backend == 'depthai' else ExecuteProcess)
    assert not isinstance(selected[0], Node) if backend == 'ros' else True


def test_depthai_default_and_restricted_choices():
    arg = next(a for a in load('rtabmap_slam_px4.launch.py').entities
               if isinstance(a, DeclareLaunchArgument) and a.name == 'backend')
    context = LaunchContext()
    arg.execute(context)
    assert context.launch_configurations['backend'] == 'depthai'
    assert set(arg.choices) == {'depthai', 'ros'}


def test_3d_wrapper_pins_depthai():
    ld = load('rtabmap_slam_px4_3d.launch.py')
    include = next(a for a in ld.entities if isinstance(a, IncludeLaunchDescription))
    assert dict(include.launch_arguments)['backend'] == 'depthai'


def test_foxglove_exposes_ros_octomap_and_raw_odometry():
    import ast, re
    arg = next(a for a in load('rtabmap_slam_px4.launch.py').entities
               if isinstance(a, DeclareLaunchArgument) and a.name == 'foxglove_topic_whitelist')
    context = LaunchContext()
    arg.execute(context)
    patterns = ast.literal_eval(context.launch_configurations[arg.name])
    for topic in ['/rtabmap/octomap', '/rtabmap/octomap_ground_markers',
                  '/rtabmap/octomap_obstacle_markers', '/rtabmap/vio_odometry', '/rtabmap/mapPath']:
        assert any(re.fullmatch(p, topic) for p in patterns)
    assert any(re.fullmatch(p, '/foxglove/camera/image/compressed') for p in patterns)
    for topic in ['/oak/left/image_rect', '/oak/left/image_rect/compressed']:
        assert not any(re.fullmatch(p, topic) for p in patterns)


def test_ros_navigation_monitor_uses_one_map_producer_and_atomic_body_pose():
    ld = load('ros_planner_3d_monitor.launch.py')
    monitor = next(a for a in ld.entities if isinstance(a, IncludeLaunchDescription))
    args = dict(monitor.launch_arguments)
    assert args['octomap_producer'] == 'false'
    assert args['route_follower'] == 'false'
    assert args['pose_snapshot_topic'] == '/planner3d/pose_snapshot'
    assert args['vio_frame_id'] == 'rtabmap_odom'
    defaults = {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}
    context = LaunchContext()
    for action in defaults.values():
        action.execute(context)
    assert context.launch_configurations['start_backend'] == 'false'
    assert context.launch_configurations['current_goal_min_height_above_start'] == '0.30'
    assert 'current_goal_min_height_above_start' in args
    # The route floor sits 5 cm above the flight adapter's 0.15 m min_height.
    assert context.launch_configurations['min_route_height_above_start'] == '0.20'
    assert 'min_route_height_above_start' in args
    assert not any('offboard' in str(a) for a in ld.entities)


def test_ros_perception_tuning_reaches_backend_and_mapper():
    from launch.utilities import perform_substitutions
    from launch_ros.actions import LoadComposableNodes
    from launch_ros.utilities import evaluate_parameters
    context = LaunchContext()
    context.launch_configurations.update({
        'range_min': '0.80', 'grid_depth_decimation': '2',
        'grid_noise_min_neighbors': '6', 'mapping_rate_hz': '0.5',
    })
    monitor = load('ros_planner_3d_monitor.launch.py')
    for action in monitor.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    backend = next(a for a in monitor.entities if type(a) is ExecuteProcess)
    command = [perform_substitutions(context, part) for part in backend.cmd]
    for argument in ('range_min:=0.80', 'grid_depth_decimation:=2',
                     'grid_noise_min_neighbors:=6', 'mapping_rate_hz:=0.5',
                     'odometry_topic_queue_size:=2',
                     'odometry_sync_queue_size:=2',
                     # IMU aiding is off unless launched with imu_aiding:=true.
                     'use_imu:=false', 'filter_imu:=false'):
        assert argument in command
    camera_argument = next(argument for argument in command
                           if argument.startswith('camera_config:='))
    assert camera_argument.endswith('/config/oak_v3_rtabmap_tracked_10hz.yaml')
    native = load('ros_rtabmap_oak_v3_native_features.launch.py')
    for action in native.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    include = next(a for a in native.entities if isinstance(a, IncludeLaunchDescription))
    forwarded = dict(include.launch_arguments)
    assert perform_substitutions(context, [forwarded['grid_depth_decimation']]) == '2'
    base = load('ros_rtabmap_oak_v3.launch.py')
    for action in base.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    # Jazzy exposes no public getter; inspect descriptions without executing
    # the load action, which would contact a live component manager.
    components = [node for action in base.entities if isinstance(action, LoadComposableNodes)
                  for node in action._LoadComposableNodes__composable_node_descriptions]
    mapper = next(node for node in components
                  if perform_substitutions(context, node.node_plugin) == 'rtabmap_slam::CoreWrapper')
    parameters = evaluate_parameters(context, mapper.parameters)[0]
    remappings = {perform_substitutions(context, source): perform_substitutions(context, target)
                  for source, target in mapper.remappings}
    assert remappings['odom_info'] == '/rtabmap/vio_odom_info_lite'
    assert parameters['Grid/DepthDecimation'] == '2'
    assert parameters['Grid/RangeMin'] == '0.80'
    assert parameters['Grid/NoiseFilteringMinNeighbors'] == '6'
    assert parameters['Rtabmap/DetectionRate'] == '0.5'
    odometry = next(node for node in components
                    if perform_substitutions(context, node.node_plugin) ==
                    'rtabmap_odom::RGBDOdometry')
    odometry_parameters = evaluate_parameters(context, odometry.parameters)[0]
    assert odometry_parameters['topic_queue_size'] == 2
    assert odometry_parameters['sync_queue_size'] == 2


def test_test_clear_requires_owned_backend_and_is_disabled_by_default():
    from launch.actions import OpaqueFunction
    ld = load('ros_planner_3d_monitor.launch.py')
    context = LaunchContext()
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    validate = next(a for a in ld.entities if isinstance(a, OpaqueFunction))
    assert context.launch_configurations['test_start_clear_radius'] == '0.0'
    validate.execute(context)
    context.launch_configurations['test_start_clear_radius'] = '0.5'
    with pytest.raises(RuntimeError, match='requires start_backend'):
        validate.execute(context)
    context.launch_configurations['start_backend'] = 'true'
    validate.execute(context)
    for radius in ('0.6', '-0.1', 'nan'):
        context.launch_configurations['test_start_clear_radius'] = radius
        with pytest.raises(RuntimeError, match='between 0 and 0.5'):
            validate.execute(context)


def test_3d_recording_includes_cpu_and_artificial_start_evidence():
    from launch.utilities import perform_substitutions
    ld = load('global_planner_3d_monitor.launch.py')
    context = LaunchContext()
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['process_monitor'] == 'true'
    assert context.launch_configurations['exploration_route'] == 'true'
    assert context.launch_configurations['map_preparation_period'] == '2.0'
    assert context.launch_configurations['planning_pose_drift_tolerance'] == '0.10'
    recorder = next(a for a in ld.entities if type(a) is ExecuteProcess)
    command = [perform_substitutions(context, part) for part in recorder.cmd]
    for topic in ('/perf/processes', '/perf/cpu_percent', '/perf/throttled',
                  '/perf/process/rtabmap_octomap_node/cpu_percent',
                  '/perf/process/cpp_astar_planner_3d/cpu_percent', '/planner3d/test_start_clear',
                  '/planner3d/reference_path', '/planner3d/reference_snapshot'):
        assert topic in command


def test_unknown_aware_flight_launch_uses_raw_vio_and_verified_prefix_policy():
    from launch_ros.utilities import evaluate_parameters
    from launch.utilities import perform_substitutions
    ld = load('offboard_planner_3d_unknown.launch.py')
    context = LaunchContext()
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['auto_arm'] == 'false'
    assert context.launch_configurations['require_goal_after_hover'] == 'false'
    assert context.launch_configurations['hover_height'] == '0.30'
    assert context.launch_configurations['climb_rate'] == '0.25'
    assert context.launch_configurations['climb_leash'] == '0.12'
    assert context.launch_configurations['climb_release'] == '0.05'
    assert context.launch_configurations['climb_feedforward'] == 'true'
    assert context.launch_configurations['climb_timeout'] == '15.0'
    assert context.launch_configurations['max_vio_source_age'] == '0.30'
    assert context.launch_configurations['adapter_start_delay'] == '5.0'
    assert context.launch_configurations['route_speed'] == '0.15'
    assert context.launch_configurations['command_speed'] == '0.15'
    assert context.launch_configurations['rate_hz'] == '10.0'
    assert context.launch_configurations['path_timeout'] == '3.5'
    assert context.launch_configurations['px4_timeout'] == '0.50'
    assert context.launch_configurations['vehicle_control_mode_timeout'] == '0.75'
    assert context.launch_configurations['follower_rejoin_grace'] == '1.5'
    assert context.launch_configurations['route_equivalence_tolerance'] == '0.10'
    assert context.launch_configurations['require_px4_preflight_status'] == 'true'
    assert context.launch_configurations['px4_preflight_timeout'] == '1.0'
    adapter_timer = next(a for a in ld.entities if isinstance(a, TimerAction))
    assert len(adapter_timer.actions) == 1
    assert isinstance(adapter_timer.actions[0], Node)
    nodes = ([a for a in ld.entities if isinstance(a, Node)] +
             [a for a in adapter_timer.actions if isinstance(a, Node)])
    configs = [evaluate_parameters(context, a._Node__parameters)[0] for a in nodes]
    flight = next(p for p in configs if 'allow_unknown' in p)
    bridge = next(p for p in configs if 'input_odometry_topic' in p)
    assert flight['allow_unknown'] is True
    assert flight['path_snapshot_topic'] == '/planner3d/reference_snapshot'
    assert flight['hover_height'] == 0.30
    assert flight['climb_rate'] == 0.25
    assert flight['climb_feedforward'] is True
    assert flight['require_goal_after_hover'] is False
    assert flight['vio_info_topic'] == '/rtabmap/vio_odom_info_lite'
    assert flight['max_vio_source_age'] == 0.30
    assert flight['command_speed'] == 0.15
    assert flight['rate_hz'] == 10.0
    assert flight['path_timeout'] == 3.5
    # Hold the line is off unless explicitly launched with it.
    assert flight['hold_line_skip'] == 0.0 and flight['cross_track_gain'] == 0.0
    assert flight['hold_line_max_offset'] == 0.10 and flight['cross_track_correction_max'] == 0.08
    assert flight['path_carrot'] is False
    assert flight['corner_speed_min'] == 0.03 and flight['max_carrot_lead'] == 0.35
    assert flight['px4_timeout'] == 0.50
    assert flight['vehicle_control_mode_timeout'] == 0.75
    assert flight['follower_rejoin_grace'] == 1.5
    assert flight['route_equivalence_tolerance'] == 0.10
    assert flight['require_px4_preflight_status'] is True
    assert flight['px4_preflight_timeout'] == 1.0
    assert bridge['input_odometry_topic'] == '/rtabmap/vio_odometry'
    assert bridge['frame_transform'] == 'enu_flu_to_ned_frd'
    assert bridge['vio_yaw_offset_deg'] == flight['vio_yaw_offset_deg']
    assert bridge['max_raw_odometry_age'] == flight['max_vio_source_age']
    recorder = next(a for a in ld.entities if type(a) is ExecuteProcess)
    command = [perform_substitutions(context, part) for part in recorder.cmd]
    for topic in ['/planner3d/flight/status', '/planner3d/flight/config', '/rtabmap/octomap',
                  '/rtabmap/octomap_metadata', '/planner3d/pose_snapshot',
                  '/fmu/in/trajectory_setpoint', '/perf/processes', '/rtabmap/vio_odom_info_lite']:
        assert topic in command
    assert '/rtabmap/vio_odom_info' not in command
    assert '/rtabmap/mapData' not in command


def test_3d_flight_detects_existing_bag_recorders(tmp_path):
    module_name = 'offboard_planner_3d_unknown.launch.py'
    spec = importlib.util.spec_from_file_location(module_name, ROOT/'launch'/module_name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path/'100').mkdir()
    (tmp_path/'100'/'cmdline').write_bytes(
        b'/usr/bin/python3\0/opt/ros/jazzy/bin/ros2\0bag\0record\0--all\0')
    (tmp_path/'101').mkdir()
    (tmp_path/'101'/'cmdline').write_bytes(b'ros2\0topic\0list\0')
    (tmp_path/'self').mkdir()
    assert module.existing_bag_recorders(tmp_path) == [
        (100, '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag record --all')]



def test_madgwick_reads_the_frame_corrected_imu():
    from launch.utilities import perform_substitutions
    from launch_ros.actions import LoadComposableNodes
    context = LaunchContext()
    base = load('ros_rtabmap_oak_v3.launch.py')
    for action in base.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    madgwick = next(a for a in base.entities if isinstance(a, LoadComposableNodes) and any(
        perform_substitutions(context, node.node_plugin) == 'ImuFilterMadgwickRos'
        for node in a._LoadComposableNodes__composable_node_descriptions))
    nodes = {perform_substitutions(context, node.node_plugin): node
             for node in madgwick._LoadComposableNodes__composable_node_descriptions}
    remaps = {perform_substitutions(context, source): perform_substitutions(context, target)
              for source, target in nodes['ImuFilterMadgwickRos'].remappings}
    # The driver's oak_imu_frame TF is 180 deg off the raw samples.
    assert remaps['imu/data_raw'] == '/rtabmap/imu_frame_fixed'
    assert 'px4_vio_bridge::OakImuFrameFix' in nodes
    assert not madgwick.condition.evaluate(context)  # opt-in via filter_imu only
