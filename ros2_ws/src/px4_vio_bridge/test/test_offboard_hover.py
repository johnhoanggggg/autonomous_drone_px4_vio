import math
from types import SimpleNamespace

import pytest

from px4_vio_bridge.offboard_hover import OffboardHover, wrap_pi, yaw_from_quaternion


def test_yaw_from_px4_quaternion() -> None:
    yaw = math.radians(135.0)
    q = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))

    assert math.isclose(yaw_from_quaternion(q), yaw)


def test_wrap_pi_handles_boundary() -> None:
    assert math.isclose(wrap_pi(math.radians(190.0)), math.radians(-170.0))


def teleop_msg(linear_z):
    return SimpleNamespace(linear=SimpleNamespace(z=linear_z))


def test_foxglove_teleop_discrete_command_contract() -> None:
    assert OffboardHover.decode_foxglove_teleop(teleop_msg(0.0)) is None
    assert OffboardHover.decode_foxglove_teleop(teleop_msg(float("nan"))) is None
    assert OffboardHover.decode_foxglove_teleop(teleop_msg(-1.0)) == "LAND"
    assert OffboardHover.decode_foxglove_teleop(teleop_msg(-2.0)) == "KILL"


def test_foxglove_land_and_kill_dispatch() -> None:
    calls = []
    stub = SimpleNamespace(
        is_armed=True,
        decode_foxglove_teleop=OffboardHover.decode_foxglove_teleop,
        trigger_landing=lambda reason: calls.append(("LAND", reason)),
        trigger_kill=lambda reason: calls.append(("KILL", reason)),
    )
    OffboardHover.on_foxglove_teleop(stub, teleop_msg(-1.0))
    OffboardHover.on_foxglove_teleop(stub, teleop_msg(-2.0))
    assert calls == [
        ("LAND", "FOXGLOVE LAND PRESSED"),
        ("KILL", "FOXGLOVE EMERGENCY KILL PRESSED"),
    ]


def make_hover_stub(yaw_feedforward=False, rate_deg=5.0):
    """Minimal stand-in exercising the real OffboardHover methods unbound.

    Regression guard for the 2026-07-25 runaway-yaw flight: the configured slew
    rate and the measured gyro rate shared self.yaw_rate, so every
    sensor_combined callback silently rescaled the yaw ramp.
    """
    return SimpleNamespace(
        yaw_cmd=0.0,
        commanded_yaw_rate=math.radians(rate_deg),
        yaw_feedforward=yaw_feedforward,
        dt=0.02,
        measured_yaw_rate=None,
        max_yaw_rate=math.radians(60.0),
        excessive_yaw_rate_since=None,
        monotonic_time=lambda: 100.0,
    )


def gyro_msg(rate_z):
    return SimpleNamespace(gyro_rad=[0.0, 0.0, rate_z])


def test_gyro_callbacks_cannot_change_yaw_ramp_step() -> None:
    baseline = make_hover_stub()
    stub = make_hover_stub()
    # Replay the failure scenario: violent gyro readings arriving mid-slew.
    for rate in (0.5, -1.8, 3.0, -2.5, 1.2):
        OffboardHover.on_sensor_combined(stub, gyro_msg(rate))

    target = math.radians(90.0)
    expected_cmd, expected_speed = OffboardHover.ramp_yaw(baseline, target)
    cmd, speed = OffboardHover.ramp_yaw(stub, target)

    assert stub.commanded_yaw_rate == baseline.commanded_yaw_rate
    assert cmd == expected_cmd
    assert math.isclose(cmd, math.radians(5.0) * 0.02)
    assert math.isnan(speed) and math.isnan(expected_speed)


def test_gyro_is_stored_as_measured_rate_only() -> None:
    stub = make_hover_stub()
    OffboardHover.on_sensor_combined(stub, gyro_msg(-1.5))

    assert math.isclose(stub.measured_yaw_rate, 1.5)
    assert stub.commanded_yaw_rate == math.radians(5.0)
    # Above max_yaw_rate (60 deg/s), so the abort timer must have latched.
    assert stub.excessive_yaw_rate_since == 100.0


def test_yaw_ramp_publishes_nan_yawspeed_without_feedforward() -> None:
    stub = make_hover_stub(yaw_feedforward=False)
    for _ in range(10):
        _, speed = OffboardHover.ramp_yaw(stub, math.radians(90.0))
        assert math.isnan(speed)


def test_yaw_ramp_feedforward_matches_commanded_rate() -> None:
    stub = make_hover_stub(yaw_feedforward=True)
    OffboardHover.on_sensor_combined(stub, gyro_msg(2.0))
    _, speed = OffboardHover.ramp_yaw(stub, math.radians(90.0))

    assert math.isclose(speed, stub.commanded_yaw_rate)


def test_yaw_ramp_converges_at_commanded_rate() -> None:
    stub = make_hover_stub()
    target = math.radians(15.0)
    steps = 0
    while stub.yaw_cmd != target:
        prev = stub.yaw_cmd
        OffboardHover.on_sensor_combined(stub, gyro_msg(3.0))
        cmd, _ = OffboardHover.ramp_yaw(stub, target)
        assert abs(wrap_pi(cmd - prev)) <= math.radians(5.0) * 0.02 + 1e-12
        steps += 1
        assert steps < 10_000, "ramp failed to converge"
    # 15 deg at 5 deg/s and 50 Hz = 150 ticks.
    assert 145 <= steps <= 155


def make_climb_stub(rate=0.25, leash=0.12, feedforward=True, z_now=0.0, release=0.05):
    """Stand-in for the altitude ramp, mirroring make_hover_stub.

    Regression guard for the 2026-08-28 flight (ULog 209): a hard z step left
    PX4's vz integrator preloaded ~0.08 of collective low after the takeoff
    ramp, and the 0.30 m climb took 20.4 s. `pos.z` is NED, so z_now metres up
    is a negative z.
    """
    return SimpleNamespace(
        z_cmd=None,
        commanded_climb_rate=rate,
        climb_leash=leash,
        climb_release=release,
        climb_feedforward=feedforward,
        dt=0.02,
        pos=SimpleNamespace(z=-z_now),
    )


def test_climb_ramp_disabled_restores_the_raw_step() -> None:
    stub = make_climb_stub(rate=0.0)
    height, vz = OffboardHover.ramp_z(stub, 0.30)

    assert height == 0.30
    assert math.isnan(vz)


def test_climb_ramp_publishes_nan_velocity_without_feedforward() -> None:
    stub = make_climb_stub(feedforward=False)
    stub.z_cmd = 0.0
    for _ in range(10):
        _, vz = OffboardHover.ramp_z(stub, 0.30)
        assert math.isnan(vz)


def test_climb_feedforward_is_negative_when_climbing() -> None:
    """TrajectorySetpoint.velocity is NED, so a climb is a negative vz."""
    climbing = make_climb_stub(z_now=0.0)
    climbing.z_cmd = 0.0
    assert OffboardHover.ramp_z(climbing, 0.30)[1] == -0.25

    descending = make_climb_stub(z_now=0.30)
    descending.z_cmd = 0.30
    assert OffboardHover.ramp_z(descending, 0.0)[1] == 0.25


def test_climb_ramp_converges_at_commanded_rate_without_overshoot() -> None:
    stub = make_climb_stub()
    stub.z_cmd = 0.0
    steps = 0
    highest = 0.0
    while stub.z_cmd != 0.30:
        height, _ = OffboardHover.ramp_z(stub, 0.30)
        stub.pos = SimpleNamespace(z=-height)  # a vehicle that tracks perfectly
        highest = max(highest, height)
        steps += 1
        assert steps < 10_000, "ramp failed to converge"
    # 0.30 m at 0.25 m/s and 50 Hz = 60 ticks.
    assert 58 <= steps <= 62
    assert highest <= 0.30


def test_climb_ramp_holds_the_target_once_reached() -> None:
    stub = make_climb_stub(z_now=0.30)
    stub.z_cmd = 0.30
    height, vz = OffboardHover.ramp_z(stub, 0.30)

    assert height == 0.30
    assert vz == 0.0


def test_leash_bounds_the_setpoint_but_keeps_the_feedforward() -> None:
    """A vehicle that cannot climb must get a bounded error, not a late step.

    Without the leash the ramp runs away from a stuck vehicle and eventually
    delivers exactly the position step the ramp exists to avoid. The velocity
    demand must survive the clamp: that error is what the integrator winds on.
    """
    stub = make_climb_stub(z_now=0.05)
    for _ in range(500):
        height, vz = OffboardHover.ramp_z(stub, 0.30)

    assert height == pytest.approx(0.05 + 0.12)
    assert vz == -0.25


def test_leash_is_applied_even_on_the_first_call() -> None:
    """A node whose first setpoint is already the hover target must not step.

    z_cmd seeds at the target, so only the leash bounds that first publish.
    """
    stub = make_climb_stub(z_now=0.0)
    height, vz = OffboardHover.ramp_z(stub, 0.30)

    assert height == pytest.approx(0.12)
    assert vz == -0.25


def test_climb_ramp_runs_before_any_position_estimate() -> None:
    stub = make_climb_stub()
    stub.pos = None
    stub.z_cmd = 0.0
    for _ in range(500):
        height, _ = OffboardHover.ramp_z(stub, 0.30)

    assert height == pytest.approx(0.30)


def test_feedforward_persists_while_the_VEHICLE_is_short_of_target() -> None:
    """Regress ULog 03_53_16: the ramp finished, the vehicle had not.

    The ramp reached 0.30 while the drone was still at 0.21 -- inside
    climb_leash, so the leash never clamped -- and keying the release off the
    ramp switched the feedforward off with 0.09 m still to climb. The last
    stretch then reverted to the slow position-P crawl, 18 s of it.
    """
    stub = make_climb_stub(z_now=0.21)
    stub.z_cmd = 0.30
    height, vz = OffboardHover.ramp_z(stub, 0.30)

    assert height == 0.30                 # ramp stays converged
    assert vz == -0.25                    # but the feedforward keeps pulling


def test_feedforward_tapers_instead_of_switching_off() -> None:
    """No step at the target, and no chatter from altitude noise near it."""
    stub = make_climb_stub(z_now=0.275, release=0.05)   # 0.025 m short = half band
    stub.z_cmd = 0.30
    _, vz = OffboardHover.ramp_z(stub, 0.30)
    assert vz == pytest.approx(-0.125)

    stub = make_climb_stub(z_now=0.30, release=0.05)
    stub.z_cmd = 0.30
    _, vz = OffboardHover.ramp_z(stub, 0.30)
    assert vz == 0.0


def test_feedforward_reverses_gently_on_overshoot() -> None:
    """Above the target the feedforward pulls back, bounded by the taper."""
    stub = make_climb_stub(z_now=0.32, release=0.05)
    stub.z_cmd = 0.30
    _, vz = OffboardHover.ramp_z(stub, 0.30)

    assert 0.0 < vz <= 0.25               # NED positive = descending
    assert vz == pytest.approx(0.10)
