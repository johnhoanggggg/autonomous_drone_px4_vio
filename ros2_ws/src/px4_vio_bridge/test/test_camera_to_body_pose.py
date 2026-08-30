import math

from px4_vio_bridge.camera_to_body_pose import camera_to_body_position
import pytest


def quaternion_from_yaw(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def test_identity_orientation_converts_frd_offset_to_flu() -> None:
    assert camera_to_body_position(
        (1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0), (0.100, -0.036, 0.056)
    ) == pytest.approx((0.900, 1.964, 3.056))


def test_yaw_rotates_lever_arm_before_subtracting_it() -> None:
    assert camera_to_body_position(
        (1.0, 2.0, 3.0),
        quaternion_from_yaw(math.pi / 2.0),
        (0.100, -0.036, 0.056),
    ) == pytest.approx((1.036, 1.900, 3.056))


def test_non_unit_quaternion_is_normalized_for_rotation() -> None:
    assert camera_to_body_position(
        (0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0), (0.2, 0.1, -0.3)
    ) == pytest.approx((-0.2, 0.1, -0.3))


@pytest.mark.parametrize(
    "position,orientation,offset",
    [
        ((math.nan, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (math.inf, 0.0, 0.0)),
    ],
)
def test_rejects_invalid_inputs(position, orientation, offset) -> None:
    with pytest.raises(ValueError):
        camera_to_body_position(position, orientation, offset)
