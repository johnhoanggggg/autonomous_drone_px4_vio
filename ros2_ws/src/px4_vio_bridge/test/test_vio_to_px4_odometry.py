import math

from px4_vio_bridge.vio_to_px4_odometry import pose_rejection_reason


def test_rejects_rtabmap_reset_sentinel() -> None:
    assert pose_rejection_reason(
        (0.0, 0.0, 0.0), (0.5, 0.5, 0.5, 0.5)
    ) == "RTABMap reset sentinel"


def test_rejects_equivalent_negative_reset_quaternion() -> None:
    assert pose_rejection_reason(
        (0.0, 0.0, 0.0), (-0.5, -0.5, -0.5, -0.5)
    ) == "RTABMap reset sentinel"


def test_accepts_legitimate_stationary_origin() -> None:
    assert pose_rejection_reason((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)) is None


def test_rejects_non_finite_pose() -> None:
    assert pose_rejection_reason(
        (math.nan, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    ) == "pose contains a non-finite value"


def test_rejects_zero_norm_quaternion() -> None:
    assert pose_rejection_reason(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 0.0)
    ) == "quaternion has zero norm"
