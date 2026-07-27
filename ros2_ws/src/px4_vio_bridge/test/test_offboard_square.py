import math
from types import SimpleNamespace

from px4_vio_bridge.offboard_square import OffboardSquare, wrap_pi


def make_square_stub(**overrides):
    """Stand-in exercising the real OffboardSquare methods unbound."""
    stub = SimpleNamespace(
        x0=2.0,
        y0=-1.0,
        yaw0=0.0,          # facing north
        side_m=0.40,
        turn=math.radians(90.0),
        sides=4,
        corner_tol=0.15,
        yaw_tolerance=math.radians(5.0),
        geofence_radius=1.5,
        waypoint_frame="world",
        waypoint_speed=0.25,
        hover_height=0.30,
        auto_arm=True,
        pos=None,
        yaw_cmd=0.0,
        x_cmd=2.0,
        y_cmd=-1.0,
        x_target=2.0,
        y_target=-1.0,
        waypoint_yaw=None,
        arrived=False,
        leg=0,
        corners=[],
        headings=[],
    )
    stub.get_logger = lambda: SimpleNamespace(
        warn=lambda *a, **k: None, error=lambda *a, **k: None
    )
    stub.publish_path = lambda: None
    stub.trigger_landing = lambda reason: stub.__dict__.setdefault("landed", reason)
    stub.__dict__.update(overrides)
    return stub


def plan(stub):
    return OffboardSquare.plan_square(stub)


# --- geometry -----------------------------------------------------------


def test_square_closes_exactly() -> None:
    stub = make_square_stub()
    assert plan(stub) is True

    assert len(stub.corners) == 5  # start + one per side
    # Last corner must land back on the first, or the shape is not a square.
    assert math.isclose(stub.corners[-1][0], stub.x0, abs_tol=1e-9)
    assert math.isclose(stub.corners[-1][1], stub.y0, abs_tol=1e-9)


def test_corners_are_a_square_of_the_requested_side() -> None:
    stub = make_square_stub()
    plan(stub)

    for a, b in zip(stub.corners, stub.corners[1:]):
        assert math.isclose(math.hypot(b[0] - a[0], b[1] - a[1]), 0.40, abs_tol=1e-9)
    # Facing north, turning right: N, E, S, W.
    assert math.isclose(stub.corners[1][0], 2.40)   # north
    assert math.isclose(stub.corners[1][1], -1.00)
    assert math.isclose(stub.corners[2][0], 2.40, abs_tol=1e-9)
    assert math.isclose(stub.corners[2][1], -0.60)  # east
    assert math.isclose(stub.corners[3][0], 2.00, abs_tol=1e-9)
    assert math.isclose(stub.corners[3][1], -0.60)


def test_headings_advance_by_the_turn_and_return_to_start() -> None:
    stub = make_square_stub()
    plan(stub)

    assert len(stub.headings) == 4
    for i, h in enumerate(stub.headings):
        assert math.isclose(h, wrap_pi(stub.yaw0 + i * stub.turn), abs_tol=1e-9)
    # Four right-angle turns from the last heading is back to the start heading.
    assert math.isclose(
        wrap_pi(stub.headings[-1] + stub.turn), stub.yaw0, abs_tol=1e-9
    )


def test_square_is_planned_from_the_latch_not_chained_off_the_vehicle() -> None:
    """Chaining corners off actual position would fold tracking error into the
    shape and walk it across the room."""
    stub = make_square_stub()
    plan(stub)
    ideal = list(stub.corners)

    # Same latch, but the vehicle is half a metre off course.
    stub2 = make_square_stub()
    stub2.pos = SimpleNamespace(x=2.5, y=-0.5, z=-0.3, xy_valid=True, z_valid=True)
    plan(stub2)

    assert stub2.corners == ideal


def test_rotated_start_heading_rotates_the_square() -> None:
    stub = make_square_stub(yaw0=math.radians(90.0))  # facing east
    plan(stub)

    assert math.isclose(stub.corners[1][0], 2.00, abs_tol=1e-9)
    assert math.isclose(stub.corners[1][1], -0.60)  # first leg goes east


def test_left_turns_mirror_the_square() -> None:
    stub = make_square_stub(turn=math.radians(-90.0))
    plan(stub)

    assert math.isclose(stub.corners[-1][0], stub.x0, abs_tol=1e-9)
    assert math.isclose(stub.corners[-1][1], stub.y0, abs_tol=1e-9)
    assert math.isclose(stub.corners[2][1], -1.40)  # second leg goes west


# --- geofence -----------------------------------------------------------


def test_square_too_big_for_the_geofence_is_refused_not_clamped() -> None:
    """Clamping a corner would deform the square, so the node must refuse."""
    stub = make_square_stub(side_m=2.0)
    assert plan(stub) is False
    assert "geofence" in stub.landed


def test_square_inside_the_geofence_is_accepted() -> None:
    stub = make_square_stub(side_m=0.40)
    # Furthest corner of a 0.40 m square from a corner start is 0.40*sqrt(2).
    assert plan(stub) is True
    worst = max(math.hypot(cx - stub.x0, cy - stub.y0) for cx, cy in stub.corners)
    assert math.isclose(worst, 0.40 * math.sqrt(2.0), abs_tol=1e-9)
    assert worst < stub.geofence_radius


# --- leg / turn completion ---------------------------------------------


def test_set_leg_target_walks_the_corner_list() -> None:
    stub = make_square_stub()
    plan(stub)
    for leg in range(4):
        stub.leg = leg
        OffboardSquare.set_leg_target(stub)
        assert (stub.x_target, stub.y_target) == stub.corners[leg + 1]
        assert stub.waypoint_yaw == stub.headings[leg]
        assert stub.arrived is False


def test_corner_arrival_requires_the_vehicle_when_armed() -> None:
    stub = make_square_stub(auto_arm=True)
    plan(stub)
    stub.leg = 0
    OffboardSquare.set_leg_target(stub)
    stub.x_cmd, stub.y_cmd = stub.x_target, stub.y_target

    stub.pos = SimpleNamespace(x=2.20, y=-1.0, z=-0.3, xy_valid=True, z_valid=True)
    assert not OffboardSquare.at_corner(stub)  # 0.20 m short of a 0.15 tol

    stub.pos = SimpleNamespace(x=2.30, y=-1.0, z=-0.3, xy_valid=True, z_valid=True)
    assert OffboardSquare.at_corner(stub)


def test_corner_arrival_needs_the_setpoint_to_have_finished_slewing() -> None:
    stub = make_square_stub(auto_arm=True)
    plan(stub)
    stub.leg = 0
    OffboardSquare.set_leg_target(stub)
    # Vehicle is at the corner but the commanded point has not arrived yet.
    stub.x_cmd, stub.y_cmd = 2.20, -1.0
    stub.pos = SimpleNamespace(x=2.40, y=-1.0, z=-0.3, xy_valid=True, z_valid=True)

    assert not OffboardSquare.at_corner(stub)


def test_dry_run_corner_arrival_uses_the_commanded_point() -> None:
    stub = make_square_stub(auto_arm=False, pos=None)
    plan(stub)
    stub.leg = 0
    OffboardSquare.set_leg_target(stub)

    stub.x_cmd, stub.y_cmd = 2.20, -1.0
    assert not OffboardSquare.at_corner(stub)
    stub.x_cmd, stub.y_cmd = stub.x_target, stub.y_target
    assert OffboardSquare.at_corner(stub)


def test_yaw_error_uses_the_vehicle_when_armed_and_the_command_on_a_dry_run() -> None:
    stub = make_square_stub(auto_arm=True)
    stub.pos = SimpleNamespace(x=0.0, y=0.0, z=-0.3, heading=math.radians(80.0),
                               xy_valid=True, z_valid=True)
    err = OffboardSquare.yaw_error(stub, math.radians(90.0))
    assert math.isclose(math.degrees(err), 10.0, abs_tol=1e-9)

    stub.auto_arm = False
    stub.yaw_cmd = math.radians(88.0)
    err = OffboardSquare.yaw_error(stub, math.radians(90.0))
    assert math.isclose(math.degrees(err), 2.0, abs_tol=1e-9)


def test_yaw_error_wraps_across_the_boundary() -> None:
    stub = make_square_stub(auto_arm=True)
    stub.pos = SimpleNamespace(x=0.0, y=0.0, z=-0.3, heading=math.radians(-175.0),
                               xy_valid=True, z_valid=True)
    err = OffboardSquare.yaw_error(stub, math.radians(175.0))
    assert math.isclose(math.degrees(err), 10.0, abs_tol=1e-6)


def test_clicks_are_refused_during_a_scripted_square() -> None:
    stub = make_square_stub()
    assert OffboardSquare.accepting_waypoints(stub) is False
