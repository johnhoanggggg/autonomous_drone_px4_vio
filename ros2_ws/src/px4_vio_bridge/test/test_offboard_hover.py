import math
from types import SimpleNamespace

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
