import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from px4_vio_bridge.offboard_global_planner import OffboardGlobalPlanner
from px4_vio_bridge.planner_flight import (
    HorizontalCommandLimiter,
    clamp_to_disc,
    vio_enu_displacement_to_ned,
)


def test_vio_enu_vector_converts_to_px4_ned_axes():
    assert vio_enu_displacement_to_ned((0.3, -0.4)) == (-0.4, 0.3)
    with pytest.raises(ValueError):
        vio_enu_displacement_to_ned((math.nan, 0.0))


def test_geofence_clamps_without_changing_bearing():
    assert clamp_to_disc((10.5, -4.5), (10.0, -4.0), 1.0) == (10.5, -4.5)
    clamped = clamp_to_disc((13.0, -4.0), (10.0, -4.0), 1.0)
    assert clamped == pytest.approx((11.0, -4.0))


def test_final_command_obeys_speed_and_acceleration_limits():
    limiter = HorizontalCommandLimiter(max_speed=0.20, max_acceleration=0.40)
    limiter.reset((0.0, 0.0))
    dt = 0.02
    previous_position = limiter.position
    previous_velocity = limiter.velocity
    for _ in range(300):
        position = limiter.update((2.0, 0.0), dt)
        speed = math.dist(position, previous_position) / dt
        acceleration = math.dist(limiter.velocity, previous_velocity) / dt
        assert speed <= 0.20 + 1.0e-9
        assert acceleration <= 0.40 + 1.0e-9
        previous_position = position
        previous_velocity = limiter.velocity


def test_final_command_limits_a_route_reversal():
    limiter = HorizontalCommandLimiter(max_speed=0.20, max_acceleration=0.40)
    limiter.reset((0.0, 0.0))
    for _ in range(100):
        limiter.update((2.0, 0.0), 0.02)
    before = limiter.velocity
    limiter.update((-2.0, 0.0), 0.02)
    assert math.dist(before, limiter.velocity) <= 0.40 * 0.02 + 1.0e-9


def test_final_command_does_not_snap_to_noisy_rebased_target():
    """Regress the acceleration discontinuities in props-off bag `_1`."""
    limiter = HorizontalCommandLimiter(max_speed=0.15, max_acceleration=0.30)
    limiter.reset((0.0, 0.0))
    dt = 0.02
    previous_position = limiter.position
    previous_output_velocity = (0.0, 0.0)

    def checked_update(target):
        nonlocal previous_position, previous_output_velocity
        position = limiter.update(target, dt)
        output_velocity = (
            (position[0] - previous_position[0]) / dt,
            (position[1] - previous_position[1]) / dt,
        )
        assert output_velocity == pytest.approx(limiter.velocity)
        assert math.hypot(*output_velocity) <= 0.15 + 1.0e-9
        assert math.dist(
            output_velocity, previous_output_velocity
        ) / dt <= 0.30 + 1.0e-9
        previous_position = position
        previous_output_velocity = output_velocity

    for _ in range(40):
        checked_update((2.0, 0.0))

    # Once moving at the speed limit, put the continuously rebased target just
    # ahead of and behind the current command. The old target-snap branch reset
    # velocity instantly here, producing up to 5.02 m/s^2 in the real bag.
    for index in range(300):
        direction = 1.0 if index % 2 else -1.0
        checked_update(
            (
                limiter.position[0] + direction * 0.0001,
                limiter.position[1] + 0.0001,
            )
        )


def test_final_command_still_settles_without_target_snap():
    limiter = HorizontalCommandLimiter(max_speed=0.15, max_acceleration=0.30)
    limiter.reset((0.0, 0.0))
    target = (0.40, 0.30)
    for _ in range(1000):
        limiter.update(target, 0.02)
    assert limiter.position == pytest.approx(target, abs=1.0e-6)
    assert limiter.velocity == pytest.approx((0.0, 0.0), abs=1.0e-6)


def planner_stub(**overrides):
    stub = SimpleNamespace(
        follower_timeout=0.30,
        correction_timeout=1.0,
        follower_valid_received=99.9,
        goal_received=99.9,
        displacement_received=99.9,
        follower_valid=True,
        vio_displacement=(0.3, 0.1),
        require_follower_after=0.0,
        correction_received=99.9,
        correction_valid=True,
        correction_reason="",
        monotonic_time=lambda: 100.0,
        follower_valid_topic="/planner/follower/valid",
        follower_displacement_topic="/planner/follower/vio_displacement",
        follower_goal_topic="/planner/follower/goal_reached",
        count_publishers=lambda _topic: 1,
    )
    stub.__dict__.update(overrides)
    return stub


def test_planner_health_requires_every_fresh_structured_input():
    stub = planner_stub()
    assert OffboardGlobalPlanner.planner_health_reason(stub) is None

    stub.follower_valid = False
    assert "validity is false" in OffboardGlobalPlanner.planner_health_reason(stub)
    stub.follower_valid = True
    stub.displacement_received = 99.0
    assert "VIO displacement stale" in OffboardGlobalPlanner.planner_health_reason(stub)


def test_invalid_follower_is_not_masked_by_the_staleness_it_causes():
    # An invalid follower stops publishing displacement and goal state, so both
    # go stale ~0.3s into the fault.  The reported reason must stay the cause,
    # not the consequence: offboard_global_props_on_22 logged three HOLDs whose
    # follow-up lines blamed a VIO fault that never happened.
    stub = planner_stub(
        follower_valid=False, displacement_received=99.0, goal_received=99.0
    )
    assert OffboardGlobalPlanner.planner_health_reason(stub) == (
        "follower validity is false"
    )


def test_a_dead_follower_still_reports_staleness_over_a_stale_valid_flag():
    stub = planner_stub(follower_valid=False, follower_valid_received=99.0)
    assert "follower validity stale" in OffboardGlobalPlanner.planner_health_reason(
        stub
    )


def test_planner_health_rejects_duplicate_publishers():
    counts = {"/planner/status": 2}
    stub = planner_stub(count_publishers=lambda topic: counts.get(topic, 1))
    reason = OffboardGlobalPlanner.planner_health_reason(stub)
    assert reason == "global planner status publisher count is 2, expected exactly 1"


def test_px4_reset_requires_a_new_follower_vector():
    stub = planner_stub(require_follower_after=99.95)
    assert "after PX4 local reset" in OffboardGlobalPlanner.planner_health_reason(stub)
    stub.displacement_received = 100.01
    stub.monotonic_time = lambda: 100.02
    assert OffboardGlobalPlanner.planner_health_reason(stub) is None


def test_preflight_delegates_battery_authority_to_px4():
    """No companion battery topic or state is required to permit arming."""
    stub = SimpleNamespace(
        pos_valid=lambda: True,
        vio_fault_reason=lambda: None,
        planner_health_reason=lambda: None,
        goal_reached=False,
    )
    assert OffboardGlobalPlanner.preflight_reason(stub) is None


def correction_stub():
    stub = SimpleNamespace(
        max_correction_m=0.25,
        max_correction_yaw=math.radians(5.0),
        monotonic_time=lambda: 10.0,
        correction_valid=False,
        correction_reason="",
        correction_received=None,
    )
    return stub


def correction_message(x=0.0, yaw=0.0):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    return msg


def test_flight_correction_gate_is_stricter_than_observation_gate():
    stub = correction_stub()
    OffboardGlobalPlanner.on_correction(stub, correction_message(x=0.20))
    assert stub.correction_valid

    OffboardGlobalPlanner.on_correction(stub, correction_message(x=0.30))
    assert not stub.correction_valid
    assert "max_correction_m" in stub.correction_reason

    OffboardGlobalPlanner.on_correction(
        stub, correction_message(yaw=math.radians(6.0))
    )
    assert not stub.correction_valid
    assert "max_correction_yaw_deg" in stub.correction_reason


def route_stub(**overrides):
    captured = []
    stub = SimpleNamespace(
        vio_displacement=(0.30, 0.40),  # ENU east/north -> NED north/east
        pos=SimpleNamespace(x=10.0, y=-4.0, heading=math.atan2(0.30, 0.40)),
        x0=10.0,
        y0=-4.0,
        geofence_radius=2.0,
        dt=0.02,
        captured=captured,
        hold_point=(10.0, -4.0),
        limiter=SimpleNamespace(
            position=(10.0, -4.0),
            update=lambda target, dt: captured.append((target, dt)),
        ),
        yaw_track=True,
        yaw_target=None,
        yaw_holding=False,
        yaw_cmd=0.0,
        yaw0=0.0,
        yaw_track_min_displacement=0.15,
        yaw_track_deadband=math.radians(8.0),
        yaw_align_error=math.radians(40.0),
        yaw_resume_error=math.radians(15.0),
    )
    stub.__dict__.update(overrides)
    for name in (
        "route_yaw", "yaw_track_error", "update_yaw_target", "freeze_yaw_target",
    ):
        setattr(stub, name, getattr(OffboardGlobalPlanner, name).__get__(stub))
    return stub


def test_route_target_is_rebased_from_current_px4_position():
    stub = route_stub()
    OffboardGlobalPlanner.update_route_command(stub)
    assert stub.captured == [((10.4, -3.7), 0.02)]


def test_yaw_target_tracks_the_ned_heading_of_the_commanded_leg():
    stub = route_stub()
    OffboardGlobalPlanner.update_route_command(stub)
    # ENU (0.30 east, 0.40 north) -> NED (0.40 north, 0.30 east).
    assert stub.yaw_target == pytest.approx(math.atan2(0.30, 0.40))
    assert not stub.yaw_holding

    # A bearing change inside the deadband must not re-latch the target.
    latched = stub.yaw_target
    stub.vio_displacement = (0.34, 0.40)
    OffboardGlobalPlanner.update_route_command(stub)
    assert stub.yaw_target == latched

    stub.vio_displacement = (0.40, 0.05)
    OffboardGlobalPlanner.update_route_command(stub)
    assert stub.yaw_target == pytest.approx(math.atan2(0.40, 0.05))


def test_short_displacement_holds_the_last_tracked_heading():
    stub = route_stub()
    OffboardGlobalPlanner.update_route_command(stub)
    latched = stub.yaw_target
    stub.vio_displacement = (0.01, -0.02)  # noise near the goal
    OffboardGlobalPlanner.update_route_command(stub)
    assert stub.yaw_target == latched


def test_large_heading_error_pauses_translation_until_the_turn_settles():
    stub = route_stub(pos=SimpleNamespace(x=10.0, y=-4.0, heading=math.radians(-120.0)))
    OffboardGlobalPlanner.update_route_command(stub)
    assert stub.yaw_holding
    # The command is still driven, but toward its own position: it decelerates
    # in place instead of translating sideways.
    assert stub.captured == [((10.0, -4.0), 0.02)]

    # Hysteresis: still holding between the resume and align thresholds.
    stub.pos.heading = stub.yaw_target - math.radians(25.0)
    OffboardGlobalPlanner.update_route_command(stub)
    assert stub.yaw_holding

    stub.pos.heading = stub.yaw_target - math.radians(10.0)
    OffboardGlobalPlanner.update_route_command(stub)
    assert not stub.yaw_holding
    assert stub.captured[-1] == ((10.4, -3.7), 0.02)


def test_align_gate_can_be_disabled_to_turn_while_translating():
    stub = route_stub(
        yaw_align_error=0.0,
        pos=SimpleNamespace(x=10.0, y=-4.0, heading=math.radians(-120.0)),
    )
    OffboardGlobalPlanner.update_route_command(stub)
    assert not stub.yaw_holding
    assert stub.yaw_target == pytest.approx(math.atan2(0.30, 0.40))
    assert stub.captured == [((10.4, -3.7), 0.02)]


def test_yaw_tracking_off_keeps_the_latched_takeoff_yaw():
    stub = route_stub(yaw_track=False, yaw0=1.25)
    OffboardGlobalPlanner.update_route_command(stub)
    assert stub.yaw_target is None
    assert not stub.yaw_holding
    assert OffboardGlobalPlanner.route_yaw(stub) == 1.25
    assert stub.captured == [((10.4, -3.7), 0.02)]


def test_land_holds_the_tracked_heading_instead_of_reverting_to_takeoff_yaw():
    """Regress props-on bag `_1`: LAND published yaw ramping 32deg -> yaw0=101deg.

    It was inert only because PX4 took AUTO.LAND 40 ms after the request and
    stopped following offboard setpoints. Had it stayed in OFFBOARD the vehicle
    would have been commanded through a 69 degree turn while descending.
    """
    published = []
    stub = route_stub(yaw0=math.radians(101.0), ramp_yaw=lambda target: (target, math.nan))
    stub.now_us = lambda: 0
    stub.ensure_limiter = lambda: None
    stub.sp_pub = SimpleNamespace(publish=published.append)
    OffboardGlobalPlanner.update_route_command(stub)
    tracked = stub.yaw_target
    assert tracked == pytest.approx(math.atan2(0.30, 0.40))

    # The base class LAND state calls publish_setpoint without naming a yaw.
    OffboardGlobalPlanner.publish_setpoint(stub, 0.40)
    assert published[-1].yaw == pytest.approx(tracked)
    assert published[-1].yaw != pytest.approx(stub.yaw0)


def test_planner_fault_freezes_the_turn_where_the_slew_reached():
    stub = route_stub()
    OffboardGlobalPlanner.update_route_command(stub)
    stub.yaw_cmd = 0.20  # mid-slew toward the tracked heading
    OffboardGlobalPlanner.freeze_yaw_target(stub)
    assert stub.yaw_target == 0.20
    assert not stub.yaw_holding


def test_continuous_planner_fault_lands_even_if_reason_changes():
    now = [100.0]
    landed = []
    stub = SimpleNamespace(
        holding_for_fault=True,
        planner_fault_since=None,
        planner_fault_reason_text="",
        planner_fault_land_time=3.0,
        pos=None,
        limiter=SimpleNamespace(reset=lambda *_: None),
        is_armed=True,
        monotonic_time=lambda: now[0],
        publish_route_status=lambda *args: None,
        trigger_landing=landed.append,
    )
    OffboardGlobalPlanner.latch_fault_hold(stub, "path stale")
    assert not landed

    now[0] = 103.1
    OffboardGlobalPlanner.latch_fault_hold(stub, "correction stale")
    assert landed and "correction stale" in landed[0]
