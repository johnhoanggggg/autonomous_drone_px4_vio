"""Keep replay pacing bounded when its clock crosses a frame deadline."""
import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def replay(monkeypatch):
    script = Path(__file__).resolve().parents[4] / "scripts/ros_rtabmap_replay_realtime.py"
    monkeypatch.syspath_prepend(str(script.parent))
    spec = importlib.util.spec_from_file_location("parity_realtime_replay", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_deadline_crossing_never_requests_an_infinite_ros_wait(replay):
    # The old loop read the first value for its condition and the second for
    # the timeout, passing a negative value to ROS and blocking indefinitely.
    readings = iter([9.999999, 10.000001])
    waits = []
    node = object()

    def spin_once(received_node, *, timeout_sec):
        assert received_node is node
        assert 0 <= timeout_sec <= 0.005
        waits.append(timeout_sec)

    replay.spin_until_deadline(
        node, 10.0, spin_once=spin_once, monotonic=lambda: next(readings))
    assert len(waits) == 1
    assert waits[0] == pytest.approx(0.000001)


def test_pacing_services_callbacks_in_bounded_slices_and_skips_late_waits(replay):
    readings = iter([9.0, 9.999, 10.1])
    waits = []
    replay.spin_until_deadline(
        None, 10.0, spin_once=lambda node, *, timeout_sec: waits.append(timeout_sec),
        monotonic=lambda: next(readings))
    assert waits == pytest.approx([0.005, 0.001])
    replay.spin_until_deadline(
        None, 10.0, spin_once=lambda *args, **kwargs: pytest.fail("late wait"),
        monotonic=lambda: 10.1)
