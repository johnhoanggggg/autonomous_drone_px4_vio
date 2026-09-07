"""A drift comparison needs two stable holds, not just populated windows."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
SCRIPT = Path(__file__).resolve().parents[4] / "scripts/loop_closure_test.py"
spec = importlib.util.spec_from_file_location("loop_closure_test", SCRIPT)
loop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loop)


def result(points, end=30.0):
    node = SimpleNamespace(raw=points, raw_odom_stamps=[None]*len(points),
                           lost_odom_stamps=set(), corrected=[], corrections=[],
                           closures=[], odom=[])
    return loop.report(node, 10, 0.0, end)["raw_vio_pose_the_one_px4_uses"]


def test_distinct_stable_holds_can_measure_large_drift():
    points = [(i/10, 0.0 if i <= 100 else 0.3, 0., 0.) for i in range(301)]
    r = result(points)
    assert r["hold_comparison_valid"]
    assert r["return_to_origin_error_mm"] == 300.0


def test_motion_in_ending_window_is_not_return_error():
    points = [(i/10, 0.0 if i <= 100 else (i-100)/100, 0., 0.) for i in range(301)]
    r = result(points)
    assert not r["hold_comparison_valid"]
    assert r["return_to_origin_error_mm"] is None
    assert "end hold position span" in r["hold_comparison_error"]


def test_stale_ending_pose_is_not_a_completed_hold():
    r = result([(i/10, 0., 0., 0.) for i in range(256)])
    assert not r["hold_comparison_valid"]
    assert "stale" in r["hold_comparison_error"]


def test_small_stationary_jitter_is_accepted():
    r = result([(i/10, (-1)**i * 0.002, 0., 0.) for i in range(301)])
    assert r["hold_comparison_valid"]
    assert r["end_hold_span_m"] == pytest.approx(0.004)
