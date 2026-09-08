"""Frame correspondence and invalid-pose handling for estimator comparisons."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[4] / "scripts/depthai_rtabmap_bag.py"
spec = importlib.util.spec_from_file_location("depthai_rtabmap_bag", SCRIPT)
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def message(stamp):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))


def audit():
    return {"matched_frames": 0, "unmatched_messages": 0, "max_tuple_skew_ns": 0}


def test_submicrosecond_tracks_match_without_neighboring_frame():
    inputs = [(t, message(1_000_000_000 + offset))
              for t, offset in zip(replay.TOPICS, (0, 0, 300, 0))]
    inputs += [(t, message(1_033_333_333)) for t in replay.TOPICS]
    stats = audit()
    frames = list(replay.matched_frames(inputs, stats))
    assert len(frames) == 2
    assert replay.stamp_ns(frames[0][0]) == 1_000_000_000
    assert replay.stamp_ns(frames[1][0]) == 1_033_333_333
    assert stats == {"matched_frames": 2, "unmatched_messages": 0, "max_tuple_skew_ns": 300}


def test_missing_track_is_counted_not_borrowed_from_next_frame():
    inputs = [(t, message(1_000_000_000)) for t in replay.TOPICS if t != replay.TOPICS[2]]
    inputs += [(t, message(1_300_000_000)) for t in replay.TOPICS]
    stats = audit()
    frames = list(replay.matched_frames(inputs, stats))
    assert len(frames) == 1
    assert replay.stamp_ns(frames[0][0]) == 1_300_000_000
    assert stats["unmatched_messages"] == 3


def test_duplicate_input_is_rejected():
    inputs = [(replay.TOPICS[0], message(1_000_000_000))] * 2
    with pytest.raises(ValueError, match="Duplicate"):
        list(replay.matched_frames(inputs, audit()))


def test_sdk_null_rotation_is_not_normalized_into_valid_pose():
    assert replay.pose_is_valid([0, 0, 0], [0, 0, 0, 1])
    assert not replay.pose_is_valid([0, 0, 0], [0, 0, 0.5, 0])
    assert not replay.pose_is_valid([0, 0, 0], [0.5, 0.5, 0.5, 0.5])
    assert replay.pose_is_valid([1, 0, 0], [0.5, 0.5, 0.5, 0.5])
    assert not replay.pose_is_valid([float("nan"), 0, 0], [0, 0, 0, 1])


def test_depth_byte_conversion_preserves_pixels_and_removes_row_padding():
    import numpy as np

    depth = SimpleNamespace(encoding="16UC1", width=2, height=2, step=6,
                            is_bigendian=True, data=bytes([
                                0x03, 0xe8, 0x07, 0xd0, 99, 99,
                                0x0b, 0xb8, 0x0f, 0xa0, 99, 99]))
    packed = replay.image_bytes(depth, "16UC1", np)
    assert packed.view("<u2").tolist() == [1000, 2000, 3000, 4000]


def test_invalid_depth_buffer_is_rejected():
    import numpy as np

    depth = SimpleNamespace(encoding="16UC1", width=2, height=2, step=4,
                            is_bigendian=False, data=bytes(7))
    with pytest.raises(ValueError, match="buffer size"):
        replay.image_bytes(depth, "16UC1", np)
