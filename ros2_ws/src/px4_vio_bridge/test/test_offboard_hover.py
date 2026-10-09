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


def latched_stub(z_now):
    stub = make_climb_stub(z_now=z_now)
    stub.climb_ff_latch = True
    stub.climb_ff_done_target = None
    stub.z_cmd = 0.30
    return stub


def test_climb_feedforward_latches_off_at_the_target() -> None:
    """2026-09-28: left running, the feedforward acted as a second altitude P loop
    (0.25 m/s per 0.05 m) and pumped a ~1 Hz bob. Once reached, it must stay 0."""
    stub = latched_stub(z_now=0.295)            # within 1 cm, ramp finished
    assert OffboardHover.ramp_z(stub, 0.30)[1] == 0.0
    for z in (0.27, 0.33, 0.26):                  # later height noise/bob: no feedforward
        stub.pos = SimpleNamespace(z=-z)
        assert OffboardHover.ramp_z(stub, 0.30)[1] == 0.0


def test_climb_feedforward_still_drives_until_the_vehicle_arrives() -> None:
    stub = latched_stub(z_now=0.27)              # ramp done, vehicle 3 cm short
    assert OffboardHover.ramp_z(stub, 0.30)[1] < 0.0
    assert stub.climb_ff_done_target is None


def test_new_altitude_target_rearms_the_climb_feedforward() -> None:
    stub = latched_stub(z_now=0.30)
    OffboardHover.ramp_z(stub, 0.30)
    assert stub.climb_ff_done_target == 0.30
    stub.pos = SimpleNamespace(z=-0.30)
    for _ in range(3):
        _, vz = OffboardHover.ramp_z(stub, 0.60)
    assert vz < 0.0 and stub.climb_ff_done_target is None


def test_latch_off_keeps_the_old_tracking_feedforward() -> None:
    stub = latched_stub(z_now=0.295)
    stub.climb_ff_latch = False
    OffboardHover.ramp_z(stub, 0.30)
    stub.pos = SimpleNamespace(z=-0.27)
    assert OffboardHover.ramp_z(stub, 0.30)[1] < 0.0


def battery_msg(volts=16.0, cells=4, remaining=0.80, warning=0, connected=True,
                current=25.0, discharged=100.0):
    return SimpleNamespace(
        connected=connected, voltage_v=volts, cell_count=cells, remaining=remaining,
        warning=warning, current_a=current, discharged_mah=discharged,
    )


def make_battery_stub(now=100.0, armed=True):
    """Low-battery LAND: PX4 COM_LOW_BAT_ACT=0 only warns, so a long hold would
    otherwise fly the pack flat (2026-10-05 battery-test setup)."""
    stub = SimpleNamespace(
        battery_land=True,
        battery_land_cell_volts=3.50,
        battery_land_percent=20.0,
        battery_land_on_px4_warning=True,
        battery_land_time=3.0,
        battery_timeout=5.0,
        battery=None,
        last_battery_time=None,
        battery_low_since=None,
        battery_low_detail=None,
        is_armed=armed,
        clock=now,
    )
    stub.monotonic_time = lambda: stub.clock
    stub.battery_low_condition = lambda msg: OffboardHover.battery_low_condition(stub, msg)
    stub.battery_text = OffboardHover.battery_text
    return stub


def feed(stub, msg, at):
    stub.clock = at
    OffboardHover.on_battery(stub, msg)


def test_healthy_battery_never_lands() -> None:
    stub = make_battery_stub()
    for t in range(10):
        feed(stub, battery_msg(), 100.0 + t)
    assert OffboardHover.battery_land_reason(stub) is None


def test_low_cell_voltage_lands_only_after_persistence() -> None:
    stub = make_battery_stub()
    feed(stub, battery_msg(volts=13.9), 100.0)        # 3.475 V/cell
    feed(stub, battery_msg(volts=13.9), 102.0)
    assert OffboardHover.battery_land_reason(stub) is None
    feed(stub, battery_msg(volts=13.9), 103.0)
    reason = OffboardHover.battery_land_reason(stub)
    assert reason is not None and "cell voltage 3.48V" in reason


def test_transient_sag_resets_the_timer() -> None:
    stub = make_battery_stub()
    feed(stub, battery_msg(volts=13.9), 100.0)
    feed(stub, battery_msg(volts=14.8), 101.0)        # recovered
    feed(stub, battery_msg(volts=13.9), 102.0)
    feed(stub, battery_msg(volts=13.9), 104.0)
    assert OffboardHover.battery_land_reason(stub) is None


def test_low_remaining_and_px4_warning_land() -> None:
    stub = make_battery_stub()
    feed(stub, battery_msg(remaining=0.19), 100.0)
    feed(stub, battery_msg(remaining=0.19), 103.0)
    assert "remaining 19%" in OffboardHover.battery_land_reason(stub)

    stub = make_battery_stub()
    feed(stub, battery_msg(warning=1), 100.0)
    feed(stub, battery_msg(warning=1), 103.0)
    assert "PX4 battery warning 1" in OffboardHover.battery_land_reason(stub)


def test_invalid_remaining_and_cell_count_are_ignored() -> None:
    stub = make_battery_stub()
    feed(stub, battery_msg(remaining=-1.0, cells=0, volts=10.0), 100.0)
    feed(stub, battery_msg(remaining=-1.0, cells=0, volts=10.0), 104.0)
    assert OffboardHover.battery_land_reason(stub) is None


def test_stale_battery_telemetry_lands_while_armed() -> None:
    stub = make_battery_stub()
    feed(stub, battery_msg(), 100.0)
    stub.clock = 105.5
    assert "stale" in OffboardHover.battery_land_reason(stub)


def test_battery_land_inactive_when_disarmed_or_disabled() -> None:
    stub = make_battery_stub(armed=False)
    feed(stub, battery_msg(volts=13.0), 100.0)
    stub.clock = 200.0
    assert OffboardHover.battery_land_reason(stub) is None

    stub = make_battery_stub()
    stub.battery_land = False
    assert OffboardHover.battery_land_reason(stub) is None


def test_disconnected_battery_message_is_dropped() -> None:
    stub = make_battery_stub()
    feed(stub, battery_msg(connected=False, volts=13.0), 100.0)
    assert stub.battery is None and stub.last_battery_time is None


def test_height_keys_decode_both_cursor_encodings() -> None:
    assert OffboardHover.decode_height_keys(b"\x1b[A") == 1
    assert OffboardHover.decode_height_keys(b"\x1bOB") == -1
    assert OffboardHover.decode_height_keys(b"\x1b[A\x1b[A\x1b[B") == 1
    assert OffboardHover.decode_height_keys(b"kl") == 0


def make_height_stub(height=0.8, steps=0):
    logs = []
    stub = SimpleNamespace(
        hover_height=height,
        height_step=0.10,
        min_hover_height=0.20,
        max_hover_height=1.50,
        pending_height_steps=steps,
        get_logger=lambda: SimpleNamespace(warn=logs.append),
    )
    return stub, logs


def test_height_keys_step_and_clamp() -> None:
    stub, _ = make_height_stub(steps=2)
    OffboardHover.apply_height_keys(stub)
    assert math.isclose(stub.hover_height, 1.0)
    assert stub.pending_height_steps == 0

    stub.pending_height_steps = 50
    OffboardHover.apply_height_keys(stub)
    assert math.isclose(stub.hover_height, 1.50)

    stub.pending_height_steps = -50
    OffboardHover.apply_height_keys(stub)
    assert math.isclose(stub.hover_height, 0.20)


def test_no_pending_height_keys_leave_height_untouched() -> None:
    stub, logs = make_height_stub()
    OffboardHover.apply_height_keys(stub)
    assert stub.hover_height == 0.8
    assert logs == []
