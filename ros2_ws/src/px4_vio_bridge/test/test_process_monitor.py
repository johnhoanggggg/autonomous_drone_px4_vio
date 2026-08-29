import math

from px4_vio_bridge.process_monitor import (
    DEFAULT_TARGETS,
    ProcessMonitor,
    match_label,
    parse_throttled,
)


def test_throttled_word_parses_the_vcgencmd_format() -> None:
    assert parse_throttled("throttled=0x0\n") == 0
    # Under-voltage now (bit 0) plus under-voltage and throttling latched.
    assert parse_throttled("throttled=0x50005") == 0x50005
    assert parse_throttled("throttled=0") == 0


def test_throttled_word_is_minus_one_when_unavailable() -> None:
    """A non-Pi host, or a vcgencmd that failed, must not publish a real value."""
    assert parse_throttled("") == -1
    assert parse_throttled(None) == -1
    assert parse_throttled("command not found") == -1


def test_multi_token_patterns_tolerate_intervening_arguments() -> None:
    """`ros2 bag record` is three argv entries with flags mixed in."""
    cmdline = "/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag record --all-topics --storage mcap"
    assert match_label(cmdline, "ros2 bag record")
    assert not match_label(cmdline, "ros2 bag play")


def test_labels_do_not_match_each_other() -> None:
    """A label must not also count another flight-stack process's CPU."""
    for label, pattern in DEFAULT_TARGETS.items():
        for other, other_pattern in DEFAULT_TARGETS.items():
            if label == other:
                continue
            assert not match_label(other_pattern, pattern), f"{label} matches {other}"


def test_sample_drops_processes_that_have_exited() -> None:
    """A relaunched stack must not leave dead pids inflating the totals."""

    class Dead:
        def cpu_percent(self, _):
            raise __import__("psutil").NoSuchProcess(1)

    class Live:
        def cpu_percent(self, _):
            return 40.0

        def memory_info(self):
            return type("M", (), {"rss": 10 * 1024 * 1024})()

        def num_threads(self):
            return 3

    monitor = ProcessMonitor.__new__(ProcessMonitor)
    monitor.cores = 4
    monitor.tracked = {"slam": {1: Dead(), 2: Live()}}

    stats = monitor.sample()["slam"]

    assert stats["pids"] == [2]
    assert stats["count"] == 1
    assert stats["cpu_percent"] == 40.0
    assert stats["cpu_percent_of_machine"] == 10.0
    assert stats["mem_mb"] == 10.0
    assert stats["running"]
    assert monitor.tracked["slam"] == {2: monitor.tracked["slam"][2]}


def test_sample_reports_a_label_with_nothing_running() -> None:
    monitor = ProcessMonitor.__new__(ProcessMonitor)
    monitor.cores = 4
    monitor.tracked = {"planner": {}}

    stats = monitor.sample()["planner"]

    assert not stats["running"]
    assert stats["count"] == 0
    assert stats["cpu_percent"] == 0.0


def test_cpu_temp_is_nan_when_the_thermal_zone_is_missing(monkeypatch) -> None:
    monkeypatch.setattr("px4_vio_bridge.process_monitor.THERMAL_PATH", "/nonexistent")
    monitor = ProcessMonitor.__new__(ProcessMonitor)

    assert math.isnan(monitor.cpu_temp())


def test_launch_processes_do_not_match_node_labels() -> None:
    """`ros2 launch <pkg> <name>.launch.py` carries the launch file's name.

    Before 2026-08-28 the bare "offboard_global_planner" pattern matched that
    command line as well as the node, adding the launch process's CPU to the
    adapter's row -- and hiding, in a cpp_mode run where no Python adapter
    exists at all, the fact that the row was measuring nothing else.
    """
    launches = (
        "/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch px4_vio_bridge "
        "offboard_global_planner.launch.py auto_arm:=true cpp_mode:=true",
        "/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch px4_vio_bridge "
        "global_planner_monitor.launch.py robot_radius:=0.25",
        "/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch px4_vio_bridge "
        "rtabmap_slam_px4.launch.py slam_publish_grid:=true",
    )
    for cmdline in launches:
        for label, pattern in DEFAULT_TARGETS.items():
            if label == "bag_record":
                continue
            assert not match_label(cmdline, pattern), f"{label} matches a launch process"


def test_node_executables_still_match_their_labels() -> None:
    """The install-path patterns must still find the processes they name."""
    prefix = "/home/john/autonomous_drone_px4_vio/ros2_ws/install/px4_vio_bridge"
    cases = {
        "planner": f"/usr/bin/python3 {prefix}/lib/px4_vio_bridge/"
                   "offboard_global_planner --ros-args -p auto_arm:=true",
        "planner_cpp": f"{prefix}/lib/px4_vio_bridge/cpp_flight_adapter "
                       "--ros-args -r __node:=cpp_flight_adapter",
        "planner_cpp_shadow": f"{prefix}/lib/px4_vio_bridge/cpp_clearance_shadow "
                              "--ros-args",
        "astar": f"/usr/bin/python3 {prefix}/lib/px4_vio_bridge/"
                 "global_planner_monitor --ros-args",
        "follower": f"/usr/bin/python3 {prefix}/lib/px4_vio_bridge/"
                    "route_follower_monitor --ros-args",
        "follower_cpp": f"{prefix}/lib/px4_vio_bridge/cpp_route_follower "
                        "--ros-args -r __node:=route_follower_monitor",
    }
    for label, cmdline in cases.items():
        assert match_label(cmdline, DEFAULT_TARGETS[label]), f"{label} no longer matches"
        for other, pattern in DEFAULT_TARGETS.items():
            if other != label:
                assert not match_label(cmdline, pattern), f"{other} also matches {label}"
