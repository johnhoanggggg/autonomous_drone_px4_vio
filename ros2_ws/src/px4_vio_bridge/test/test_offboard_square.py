import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

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
        use_loop_corrections=False,
        loop_command_tolerance=0.02,
        map_corners=[],
        map_headings=[],
        map_target=None,
        map_pose=None,
        map_pose_valid=False,
        map_pose_reason="not received",
        map_pose_received=None,
        map_pose_timeout=1.0,
        correction=None,
        correction_valid=False,
        correction_reason="not received",
        correction_received=None,
        correction_timeout=1.0,
        max_correction_m=0.50,
        max_correction_yaw=math.radians(15.0),
        monotonic_time=lambda: 10.0,
        is_armed=False,
        state="LEG",
    )
    stub.get_logger = lambda: SimpleNamespace(
        warn=lambda *a, **k: None, error=lambda *a, **k: None
    )
    stub.publish_path = lambda: None
    stub.trigger_landing = lambda reason: stub.__dict__.setdefault("landed", reason)
    stub.set_state = lambda state: setattr(stub, "state", state)
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


# --- optional loop-corrected mode --------------------------------------


def corrected_stub(**overrides):
    stub = make_square_stub(
        use_loop_corrections=True,
        map_pose=(3.0, 4.0, 0.0),
        map_pose_valid=True,
        map_pose_reason="",
        map_pose_received=10.0,
        correction=(0.0, 0.0, 0.0, 0.0),
        correction_valid=True,
        correction_reason="",
        correction_received=10.0,
        pos=SimpleNamespace(
            x=2.0,
            y=-1.0,
            z=-0.3,
            heading=0.0,
            xy_valid=True,
            z_valid=True,
        ),
    )
    stub.__dict__.update(overrides)
    for name in ("loop_correction_health_reason", "corrected_ned_yaw"):
        setattr(stub, name, getattr(OffboardSquare, name).__get__(stub))
    return stub


def correction_message(x=0.0, yaw=0.0):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    return msg


def map_pose_message(x=3.0, y=4.0, yaw=0.0, frame="world"):
    msg = correction_message(yaw=yaw)
    msg.header.frame_id = frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    return msg


def test_loop_corrected_square_is_canonical_in_world_and_turns_right() -> None:
    stub = corrected_stub()
    assert plan(stub) is True

    assert len(stub.map_corners) == 5
    for first, second in zip(stub.map_corners, stub.map_corners[1:]):
        assert math.isclose(math.dist(first, second), stub.side_m, abs_tol=1e-9)
    assert math.dist(stub.map_corners[0], stub.map_corners[-1]) < 1e-9
    # ENU yaw 0 points east; a positive/right NED turn points south next.
    assert math.isclose(stub.map_corners[1][0], 3.40)
    assert math.isclose(stub.map_corners[1][1], 4.00)
    assert math.isclose(stub.map_corners[2][0], 3.40, abs_tol=1e-9)
    assert math.isclose(stub.map_corners[2][1], 3.60)


def test_corrected_target_is_rebased_into_px4_local_frame() -> None:
    stub = corrected_stub(map_target=(4.0, 4.0))
    original_command = (stub.x_cmd, stub.y_cmd)

    assert OffboardSquare.refresh_corrected_target(stub, 0.0)
    # One metre east in map/VIO ENU is +east in PX4 NED y.
    assert (stub.x_target, stub.y_target) == (2.0, 0.0)
    assert math.isclose(stub.waypoint_yaw, math.radians(90.0))

    # A +90-degree map<-VIO correction rotates that same map displacement
    # into -north in VIO, and rotates the desired local heading continuously.
    stub.correction = (0.0, 0.0, 0.0, math.radians(90.0))
    assert OffboardSquare.refresh_corrected_target(stub, 0.0)
    assert math.isclose(stub.x_target, 1.0, abs_tol=1e-9)
    assert math.isclose(stub.y_target, -1.0, abs_tol=1e-9)
    assert math.isclose(abs(stub.waypoint_yaw), math.pi, abs_tol=1e-9)
    # A correction updates only the target. The inherited slew limiter advances
    # x_cmd/y_cmd later, so PX4 never receives the correction as a position step.
    assert (stub.x_cmd, stub.y_cmd) == original_command


def test_corrected_arrival_uses_world_pose_not_uncorrected_local_pose() -> None:
    stub = corrected_stub(
        map_target=(3.4, 4.0),
        map_pose=(3.30, 4.0, 0.0),
        x_target=10.0,
        y_target=10.0,
        x_cmd=10.01,
        y_cmd=10.0,
        auto_arm=True,
    )
    # Local PX4 position is nowhere near (10, 10), but corrected world feedback
    # is within corner_tol and the smoothed command has converged.
    assert OffboardSquare.at_corner(stub)

    stub.map_pose = (3.20, 4.0, 0.0)
    assert not OffboardSquare.at_corner(stub)


def test_loop_correction_callbacks_reject_bad_inputs() -> None:
    stub = corrected_stub()
    OffboardSquare.on_correction(stub, correction_message(x=0.40))
    assert stub.correction_valid

    OffboardSquare.on_correction(stub, correction_message(x=0.60))
    assert not stub.correction_valid
    assert "max_correction_m" in stub.correction_reason

    OffboardSquare.on_correction(
        stub, correction_message(yaw=math.radians(16.0))
    )
    assert not stub.correction_valid
    assert "max_correction_yaw_deg" in stub.correction_reason

    invalid = correction_message()
    invalid.pose.orientation.w = 2.0
    OffboardSquare.on_correction(stub, invalid)
    assert not stub.correction_valid
    assert "non-finite" in stub.correction_reason


def test_corrected_map_pose_requires_world_and_unit_quaternion() -> None:
    stub = corrected_stub()
    OffboardSquare.on_map_pose(stub, map_pose_message(yaw=0.25))
    assert stub.map_pose_valid
    assert math.isclose(stub.map_pose[2], 0.25)

    OffboardSquare.on_map_pose(stub, map_pose_message(frame="map"))
    assert not stub.map_pose_valid
    assert "frame" in stub.map_pose_reason

    invalid = map_pose_message()
    invalid.pose.orientation.w = 2.0
    OffboardSquare.on_map_pose(stub, invalid)
    assert not stub.map_pose_valid
    assert "invalid quaternion" in stub.map_pose_reason


def test_loop_correction_health_fails_closed_on_stale_input() -> None:
    stub = corrected_stub()
    assert OffboardSquare.loop_correction_health_reason(stub) is None

    stub.correction_received = 8.0
    assert "correction stale" in OffboardSquare.loop_correction_health_reason(stub)

    stub.correction_received = 10.0
    stub.map_pose_received = 8.0
    assert "body pose stale" in OffboardSquare.loop_correction_health_reason(stub)


def test_corrected_target_outside_geofence_is_refused_not_clamped() -> None:
    stub = corrected_stub(map_target=(5.0, 4.0), geofence_radius=1.5)
    assert not OffboardSquare.refresh_corrected_target(stub, 0.0)
    assert stub.state == "ABORT"
