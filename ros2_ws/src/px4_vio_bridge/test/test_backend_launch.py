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
        ['bash', '-c', preamble * source_count + 'exec bash "$1" --check-environment',
         'backend-env-test', str(ROOT/'scripts'/'ros_rtabmap_backend')],
        capture_output=True, text=True, timeout=25)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'environment verified (no graph started)' in result.stdout


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'launch'/name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


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
