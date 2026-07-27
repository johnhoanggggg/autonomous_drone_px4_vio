import math

from px4_vio_bridge.map_correction import (
    Correction,
    apply_correction,
    correction_from_pair,
    correction_rejection_reason,
    correction_residual,
    exceeds_deadband,
    nearest_sample,
    quaternion_from_yaw,
    slew_correction,
    wrap_pi,
    yaw_from_quaternion,
)


IDENTITY_Q = (1.0, 0.0, 0.0, 0.0)


def test_yaw_round_trips_through_quaternion() -> None:
    yaw = math.radians(-115.0)

    assert math.isclose(yaw_from_quaternion(quaternion_from_yaw(yaw)), yaw)


def test_correction_from_pair_reproduces_the_slam_pose() -> None:
    """The solved correction must be exact in position and yaw by construction."""
    vio_position = (1.5, -0.4, 0.8)
    vio_orientation = quaternion_from_yaw(math.radians(30.0))
    slam_position = (1.62, -0.31, 0.79)
    slam_orientation = quaternion_from_yaw(math.radians(34.0))

    correction = correction_from_pair(
        vio_position, vio_orientation, slam_position, slam_orientation
    )
    position, orientation = apply_correction(correction, vio_position, vio_orientation)

    for got, want in zip(position, slam_position):
        assert math.isclose(got, want, abs_tol=1.0e-12)
    assert math.isclose(
        yaw_from_quaternion(orientation), math.radians(34.0), abs_tol=1.0e-12
    )


def test_identical_poses_give_the_identity_correction() -> None:
    position = (0.3, 0.2, 0.1)

    correction = correction_from_pair(position, IDENTITY_Q, position, IDENTITY_Q)

    assert math.isclose(correction.translation_norm, 0.0, abs_tol=1.0e-12)
    assert math.isclose(correction.yaw, 0.0, abs_tol=1.0e-12)


def test_slew_caps_translation_on_its_norm_and_keeps_direction() -> None:
    """Capping the Euclidean norm makes the correction arrive along a straight line."""
    target = Correction(tx=0.3, ty=0.4, tz=0.0)

    stepped = slew_correction(Correction(), target, max_translation_step=0.05, max_yaw_step=1.0)

    assert math.isclose(stepped.translation_norm, 0.05, abs_tol=1.0e-12)
    assert math.isclose(stepped.tx / stepped.ty, 0.3 / 0.4, abs_tol=1.0e-12)


def test_slew_does_not_overshoot_the_target() -> None:
    target = Correction(tx=0.01, ty=0.0, tz=0.0, yaw=math.radians(0.1))

    stepped = slew_correction(
        Correction(), target, max_translation_step=0.5, max_yaw_step=math.radians(5.0)
    )

    assert stepped == target


def test_slew_takes_the_short_way_around_pi() -> None:
    applied = Correction(yaw=math.radians(170.0))
    target = Correction(yaw=math.radians(-170.0))

    stepped = slew_correction(applied, target, 1.0, math.radians(5.0))

    # 170 -> -170 is +20 deg the short way, so one 5 deg step lands on 175.
    assert math.isclose(stepped.yaw, math.radians(175.0), abs_tol=1.0e-9)


def test_slew_converges_at_the_configured_rate() -> None:
    """A 20 cm loop closure at 3 cm/s must take ~6.7 s, not arrive as a jump."""
    target = Correction(tx=0.20)
    applied = Correction()
    dt = 0.02
    step = 0.03 * dt

    ticks = 0
    while correction_residual(applied, target)[0] > 1.0e-9 and ticks < 10000:
        applied = slew_correction(applied, target, step, math.radians(1.0) * dt)
        ticks += 1

    assert math.isclose(ticks * dt, 0.20 / 0.03, rel_tol=0.02)
    assert math.isclose(applied.tx, 0.20, abs_tol=1.0e-12)


def test_oversized_corrections_are_rejected_not_ramped() -> None:
    gate_m = 0.5
    gate_yaw = math.radians(15.0)

    assert correction_rejection_reason(Correction(tx=0.2), gate_m, gate_yaw) is None
    assert "exceeds max_correction_m" in (
        correction_rejection_reason(Correction(tx=0.9), gate_m, gate_yaw) or ""
    )
    assert "exceeds max_correction_yaw_deg" in (
        correction_rejection_reason(
            Correction(yaw=math.radians(40.0)), gate_m, gate_yaw
        )
        or ""
    )
    assert (
        correction_rejection_reason(Correction(tx=math.nan), gate_m, gate_yaw)
        == "correction contains a non-finite value"
    )


def test_deadband_ignores_slam_reoptimization_jitter() -> None:
    latched = Correction(tx=0.10, ty=0.05)
    deadband_m = 0.01
    deadband_yaw = math.radians(0.2)

    jitter = Correction(tx=0.103, ty=0.051)
    closure = Correction(tx=0.24, ty=0.05)

    assert not exceeds_deadband(jitter, latched, deadband_m, deadband_yaw)
    assert exceeds_deadband(closure, latched, deadband_m, deadband_yaw)


def test_nearest_sample_picks_the_closest_stamp() -> None:
    history = [
        (10.00, (0.0, 0.0, 0.0), IDENTITY_Q),
        (10.08, (1.0, 0.0, 0.0), IDENTITY_Q),
        (10.16, (2.0, 0.0, 0.0), IDENTITY_Q),
    ]

    assert nearest_sample(history, 10.10)[1] == (1.0, 0.0, 0.0)
    assert nearest_sample([], 10.10) is None


def test_wrap_pi_handles_boundary() -> None:
    assert math.isclose(wrap_pi(math.radians(190.0)), math.radians(-170.0))
