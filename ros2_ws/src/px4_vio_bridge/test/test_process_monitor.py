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
