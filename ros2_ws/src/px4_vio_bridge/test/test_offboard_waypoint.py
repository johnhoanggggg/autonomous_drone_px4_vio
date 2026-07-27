import math
from types import SimpleNamespace

from px4_vio_bridge.offboard_hover import OffboardHover
from px4_vio_bridge.offboard_waypoint import (
    OffboardWaypoint,
    swap_enu_ned_xy,
    swap_enu_ned_yaw,
    yaw_from_ros_quaternion,
)


def test_enu_ned_xy_swap_is_its_own_inverse() -> None:
    # px4_local_position_to_ros publishes x_enu=y_ned (east), y_enu=x_ned (north).
    assert swap_enu_ned_xy(1.0, 2.0) == (2.0, 1.0)
    assert swap_enu_ned_xy(*swap_enu_ned_xy(1.0, 2.0)) == (1.0, 2.0)


def test_enu_ned_yaw_matches_the_publisher_convention() -> None:
    # A click facing ENU +x (east) is NED heading 90 deg.
    assert math.isclose(swap_enu_ned_yaw(0.0), math.pi / 2.0)
    heading = math.radians(37.0)
    assert math.isclose(swap_enu_ned_yaw(swap_enu_ned_yaw(heading)), heading)


def test_yaw_from_ros_quaternion() -> None:
    yaw = math.radians(-120.0)
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    assert math.isclose(yaw_from_ros_quaternion(q), yaw)


def make_waypoint_stub(**overrides):
    """Stand-in exercising the real OffboardWaypoint methods unbound."""
    stub = SimpleNamespace(
        x0=10.0,
        y0=-4.0,
        x_cmd=10.0,
        y_cmd=-4.0,
        x_target=10.0,
        y_target=-4.0,
        cmd_vx=0.0,
        cmd_vy=0.0,
        settled_t=99.0,
        idle_t=0.0,
        waypoint_speed=0.25,
        geofence_radius=1.5,
        arrival_tol=0.12,
        dt=0.02,
        auto_arm=True,
        pos=None,
        state="WAYPOINT",
        waypoint_frame="world",
        waypoint_yaw=None,
        accept_waypoint_yaw=False,
        velocity_feedforward=False,
        transit_horizontal_error=0.60,
        transit_settle_time=1.0,
        max_horizontal_error=0.35,
        horizontal_error=None,
        horizontal_error_since=None,
        horizontal_error_time=0.25,
        waypoints_accepted=0,
        waypoints_rejected=0,
        monotonic_time=lambda: 100.0,
    )
    stub.get_logger = lambda: SimpleNamespace(
        warn=lambda *a, **k: None, error=lambda *a, **k: None
    )
    stub.publish_status = lambda *a, **k: None
    stub.publish_target = lambda *a, **k: None
    # Methods the code under test calls on itself, bound back to the real class.
    stub.accepting_waypoints = lambda: OffboardWaypoint.accepting_waypoints(stub)
    stub.clamp_to_geofence = lambda x, y: OffboardWaypoint.clamp_to_geofence(stub, x, y)
    stub.ensure_commanded_position = lambda: OffboardWaypoint.ensure_commanded_position(
        stub
    )
    stub.reject = lambda reason: OffboardWaypoint.reject(stub, reason)
    stub.__dict__.update(overrides)
    return stub


def accept(stub, x_enu, y_enu, frame="world", yaw_enu=None):
    OffboardWaypoint.accept_click(stub, frame, x_enu, y_enu, yaw_enu)


def hold_point(stub):
    return OffboardWaypoint.hold_point.fget(stub)


def error_limit(stub):
    return OffboardWaypoint.horizontal_error_limit.fget(stub)


# --- click validation ---------------------------------------------------


def test_click_is_converted_from_enu_world_to_ned() -> None:
    stub = make_waypoint_stub()
    # ENU (east=-3.5, north=10.5) -> NED (north=10.5, east=-3.5); inside the fence.
    accept(stub, -3.5, 10.5)

    assert math.isclose(stub.x_target, 10.5)
    assert math.isclose(stub.y_target, -3.5)
    assert stub.waypoints_accepted == 1


def test_wrong_frame_is_rejected() -> None:
    stub = make_waypoint_stub()
    rejected = []
    stub.reject = rejected.append
    accept(stub, 0.0, 10.5, frame="odom")

    assert len(rejected) == 1
    assert "odom" in rejected[0]
    # The target must be untouched: a rejected click may not move the vehicle.
    assert (stub.x_target, stub.y_target) == (10.0, -4.0)


def test_click_is_rejected_outside_the_interactive_states() -> None:
    for state in ("WAIT_POS", "LAND", "KILL", "DONE", "ABORT"):
        stub = make_waypoint_stub(state=state)
        rejected = []
        stub.reject = rejected.append
        accept(stub, 0.0, 10.5)

        assert len(rejected) == 1, state
        assert (stub.x_target, stub.y_target) == (10.0, -4.0)


def test_non_finite_click_is_rejected() -> None:
    stub = make_waypoint_stub()
    rejected = []
    stub.reject = rejected.append
    accept(stub, float("nan"), 10.5)

    assert len(rejected) == 1
    assert (stub.x_target, stub.y_target) == (10.0, -4.0)


def test_click_yaw_is_ignored_unless_enabled() -> None:
    stub = make_waypoint_stub(accept_waypoint_yaw=False)
    accept(stub, -4.0, 10.0, yaw_enu=None)
    assert stub.waypoint_yaw is None

    stub = make_waypoint_stub(accept_waypoint_yaw=True)
    accept(stub, -4.0, 10.0, yaw_enu=0.0)
    assert math.isclose(stub.waypoint_yaw, math.pi / 2.0)


# --- geofence -----------------------------------------------------------


def test_distant_click_is_clamped_onto_the_geofence() -> None:
    stub = make_waypoint_stub()
    # 40 m north of the takeoff latch: far outside the room.
    accept(stub, -4.0, 50.0)

    radius = math.hypot(stub.x_target - stub.x0, stub.y_target - stub.y0)
    assert math.isclose(radius, stub.geofence_radius)
    # Clamped along the bearing of the click, i.e. due north.
    assert math.isclose(stub.x_target, stub.x0 + 1.5)
    assert math.isclose(stub.y_target, stub.y0)


def test_click_inside_the_geofence_is_untouched() -> None:
    stub = make_waypoint_stub()
    x, y, clamped = OffboardWaypoint.clamp_to_geofence(stub, 11.0, -4.5)

    assert not clamped
    assert (x, y) == (11.0, -4.5)


# --- setpoint slew ------------------------------------------------------


def test_commanded_point_never_steps_faster_than_waypoint_speed() -> None:
    stub = make_waypoint_stub()
    accept(stub, -4.0, 50.0)  # clamped to 1.5 m north

    step_limit = stub.waypoint_speed * stub.dt
    previous = (stub.x_cmd, stub.y_cmd)
    ticks = 0
    while (stub.x_cmd, stub.y_cmd) != (stub.x_target, stub.y_target):
        OffboardWaypoint.slew_commanded_position(stub)
        moved = math.hypot(stub.x_cmd - previous[0], stub.y_cmd - previous[1])
        assert moved <= step_limit + 1e-12
        previous = (stub.x_cmd, stub.y_cmd)
        ticks += 1
        assert ticks < 10_000, "slew failed to converge"

    # 1.5 m at 0.25 m/s and 50 Hz = 300 ticks.
    assert 295 <= ticks <= 305


def test_slew_reports_settled_only_after_arrival() -> None:
    stub = make_waypoint_stub()
    accept(stub, -4.0, 10.4)  # 0.4 m north

    OffboardWaypoint.slew_commanded_position(stub)
    assert stub.settled_t == 0.0

    for _ in range(400):
        OffboardWaypoint.slew_commanded_position(stub)
    assert (stub.x_cmd, stub.y_cmd) == (stub.x_target, stub.y_target)
    assert stub.settled_t > 0.0


def test_velocity_feedforward_is_opt_in() -> None:
    stub = make_waypoint_stub(velocity_feedforward=False)
    accept(stub, -4.0, 11.0)
    OffboardWaypoint.slew_commanded_position(stub)

    assert all(math.isnan(v) for v in OffboardWaypoint.setpoint_velocity(stub))

    stub.velocity_feedforward = True
    vx, vy, vz = OffboardWaypoint.setpoint_velocity(stub)
    assert math.isclose(vx, stub.waypoint_speed)  # due north
    assert math.isclose(vy, 0.0, abs_tol=1e-12)
    assert math.isnan(vz)


# --- watchdog re-pointing ----------------------------------------------


def test_horizontal_watchdog_follows_the_commanded_point() -> None:
    """Regression guard: OffboardHover measures hold error from the takeoff
    latch, which would land a waypoint flight for successfully translating."""
    stub = make_waypoint_stub(settled_t=99.0)
    accept(stub, -4.0, 11.0)  # 1.0 m north
    for _ in range(400):
        OffboardWaypoint.slew_commanded_position(stub)

    # Vehicle sitting exactly on the new waypoint, 1.0 m from takeoff.
    stub.pos = SimpleNamespace(x=11.0, y=-4.0, z=-0.3, xy_valid=True, z_valid=True)
    stub.hold_point = hold_point(stub)
    stub.horizontal_error_limit = error_limit(stub)
    OffboardHover.on_local_position(stub, stub.pos)

    assert math.isclose(stub.horizontal_error, 0.0, abs_tol=1e-9)
    assert stub.horizontal_error_since is None


def test_takeoff_latch_is_used_before_the_first_command() -> None:
    stub = make_waypoint_stub(x_cmd=None, y_cmd=None)

    assert hold_point(stub) == (10.0, -4.0)


def test_transit_uses_the_looser_gate_and_reverts_once_settled() -> None:
    stub = make_waypoint_stub(settled_t=0.0)
    assert error_limit(stub) == stub.transit_horizontal_error

    stub.settled_t = stub.transit_settle_time
    assert error_limit(stub) == stub.max_horizontal_error


# --- arrival ------------------------------------------------------------


def test_armed_arrival_requires_the_vehicle_not_just_the_setpoint() -> None:
    stub = make_waypoint_stub(auto_arm=True)
    accept(stub, -4.0, 11.0)
    for _ in range(400):
        OffboardWaypoint.slew_commanded_position(stub)

    stub.pos = SimpleNamespace(x=10.5, y=-4.0, z=-0.3, xy_valid=True, z_valid=True)
    assert not OffboardWaypoint.at_waypoint(stub)  # 0.5 m short

    stub.pos = SimpleNamespace(x=10.95, y=-4.0, z=-0.3, xy_valid=True, z_valid=True)
    assert OffboardWaypoint.at_waypoint(stub)


def test_accepting_a_waypoint_clears_the_arrival_latch() -> None:
    stub = make_waypoint_stub(arrived=True, idle_t=12.0)
    accept(stub, -4.0, 11.0)

    assert stub.arrived is False
    assert stub.idle_t == 0.0


def test_arrival_latch_survives_station_keeping_jitter() -> None:
    """Regression guard for the 2026-07-27 flight: settled hold error averaged
    0.135 m against arrival_tol 0.12, so an instantaneous arrival test flickered
    and idle_t could never accumulate to the timeout."""
    stub = make_waypoint_stub(auto_arm=True)
    accept(stub, -4.0, 11.0)
    for _ in range(400):
        OffboardWaypoint.slew_commanded_position(stub)

    # Momentarily inside arrival_tol -> latches.
    stub.pos = SimpleNamespace(x=11.0, y=-4.0, z=-0.3, xy_valid=True, z_valid=True)
    assert OffboardWaypoint.at_waypoint(stub)
    stub.arrived = True

    # Drifts back outside tol, as it really did in flight.
    stub.pos = SimpleNamespace(x=11.15, y=-4.0, z=-0.3, xy_valid=True, z_valid=True)
    assert not OffboardWaypoint.at_waypoint(stub)
    assert stub.arrived is True  # the latch must not clear on jitter


def test_dry_run_arrival_tracks_the_commanded_point() -> None:
    stub = make_waypoint_stub(auto_arm=False, pos=None)
    accept(stub, -4.0, 11.0)

    assert not OffboardWaypoint.at_waypoint(stub)
    for _ in range(400):
        OffboardWaypoint.slew_commanded_position(stub)
    assert OffboardWaypoint.at_waypoint(stub)
