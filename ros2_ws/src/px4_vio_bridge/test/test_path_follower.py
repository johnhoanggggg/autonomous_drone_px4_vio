import copy
import math

import pytest

from px4_vio_bridge.path_follower import (
    CorrectionReplanGate,
    Polyline,
    PositionRouteFollower,
)


def test_polyline_projection_and_interpolated_lookahead():
    path = Polyline(((0.0, 0.0), (1.0, 0.0), (1.0, 2.0)))
    projection = path.project((0.4, 0.2))
    assert projection.point == pytest.approx((0.4, 0.0))
    assert projection.along == pytest.approx(0.4)
    assert projection.cross_track == pytest.approx(0.2)
    assert path.point_at(1.5) == pytest.approx((1.0, 0.5))


def test_progress_does_not_move_backward_with_pose_noise():
    follower = PositionRouteFollower(max_carrot_acceleration=10.0)
    follower.set_path(((0.0, 0.0), (4.0, 0.0)), (0.0, 0.0))
    first = follower.update((1.2, 0.0), 0.1)
    second = follower.update((1.0, 0.0), 0.1)
    assert first.progress == pytest.approx(1.2)
    assert second.progress == pytest.approx(1.2)


def test_position_carrot_obeys_speed_and_acceleration_limits():
    follower = PositionRouteFollower(
        lookahead=2.0, max_carrot_speed=1.0, max_carrot_acceleration=0.5
    )
    follower.set_path(((0.0, 0.0), (5.0, 0.0)), (0.0, 0.0))
    first = follower.update((0.0, 0.0), 1.0)
    second = follower.update((0.0, 0.0), 1.0)
    assert first.commanded_displacement == pytest.approx((0.5, 0.0))
    assert second.commanded_displacement == pytest.approx((1.5, 0.0))


def test_replan_does_not_jump_commanded_position_displacement():
    follower = PositionRouteFollower(max_carrot_speed=0.25, max_carrot_acceleration=10.0)
    follower.set_path(((0.0, 0.0), (4.0, 0.0)), (0.0, 0.0))
    before = follower.update((0.0, 0.0), 1.0)
    assert follower.set_path(((0.0, 0.0), (0.0, 2.0), (4.0, 2.0)), (0.0, 0.0))
    after = follower.update((0.0, 0.0), 0.2)
    assert math.dist(before.commanded_displacement, after.commanded_displacement) <= 0.25 * 0.2 + 1e-9


def test_cumulative_progress_does_not_reset_on_replan():
    follower = PositionRouteFollower(max_carrot_acceleration=10.0)
    follower.set_path(((0.0, 0.0), (5.0, 0.0)), (0.0, 0.0))
    before = follower.update((1.0, 0.0), 0.1)
    follower.set_path(((1.0, 0.0), (5.0, 0.0)), (1.0, 0.0))
    reanchored = follower.update((1.0, 0.0), 0.1)
    after = follower.update((1.5, 0.0), 0.1)
    assert reanchored.progress == pytest.approx(before.progress)
    assert after.progress == pytest.approx(before.progress + 0.5)
    assert after.path_progress == pytest.approx(0.5)


def test_new_goal_can_reset_cumulative_progress():
    follower = PositionRouteFollower()
    follower.set_path(((0.0, 0.0), (3.0, 0.0)), (0.0, 0.0))
    assert follower.update((1.0, 0.0), 0.1).progress == pytest.approx(1.0)
    follower.reset_route_progress()
    assert follower.progress == 0.0


def test_identical_republished_path_does_not_create_generation():
    follower = PositionRouteFollower()
    assert follower.set_path(((0.0, 0.0), (2.0, 0.0)), (0.0, 0.0))
    assert not follower.set_path(((0.0, 0.0), (2.0, 0.0)), (0.5, 0.0))
    assert follower.generation == 1


def test_translational_loop_closure_preserves_relative_command():
    follower = PositionRouteFollower(max_carrot_acceleration=10.0)
    follower.set_path(((0.0, 0.0), (4.0, 0.0)), (0.0, 0.0))
    before = follower.update((1.0, 0.0), 1.0)
    baseline = copy.deepcopy(follower)
    expected = baseline.update((1.0, 0.0), 0.1)
    follower.set_path(((2.0, -1.0), (6.0, -1.0)), (3.0, -1.0))
    after = follower.update((3.0, -1.0), 0.1)
    assert after.commanded_displacement == pytest.approx(expected.commanded_displacement)
    assert after.commanded_carrot == pytest.approx((expected.commanded_carrot[0] + 2.0, expected.commanded_carrot[1] - 1.0))
    assert after.progress == pytest.approx(before.progress)


def test_yaw_loop_closure_rate_limits_relative_carrot_rotation():
    follower = PositionRouteFollower(max_carrot_speed=0.25, max_carrot_acceleration=10.0)
    follower.set_path(((0.0, 0.0), (4.0, 0.0)), (1.0, 0.0))
    before = follower.update((1.0, 0.0), 1.0)
    follower.set_path(((0.0, 0.0), (0.0, 4.0)), (0.0, 1.0))
    after = follower.update((0.0, 1.0), 0.1)
    assert math.dist(before.commanded_displacement, after.commanded_displacement) <= 0.25 * 0.1 + 1e-9
    assert after.progress == pytest.approx(before.progress)


def test_cross_track_fault_holds_relative_carrot():
    follower = PositionRouteFollower(max_cross_track=0.5)
    follower.set_path(((0.0, 0.0), (3.0, 0.0)), (0.0, 0.0))
    before = follower.commanded_displacement
    result = follower.update((1.0, 0.6), 0.1)
    assert not result.valid
    assert result.status == "CROSS_TRACK_EXCEEDED"
    assert result.commanded_displacement == before


def test_goal_reached_requires_along_track_and_euclidean_arrival():
    follower = PositionRouteFollower(arrival_tolerance=0.15)
    follower.set_path(((0.0, 0.0), (1.0, 0.0)), (0.0, 0.0))
    result = follower.update((0.92, 0.02), 0.1)
    assert result.status == "GOAL_REACHED"


def test_single_point_path_reports_goal_reached():
    follower = PositionRouteFollower(arrival_tolerance=0.15)
    follower.set_path(((1.0, 2.0),), (1.05, 2.0))
    result = follower.update((1.05, 2.0), 0.1)
    assert result.status == "GOAL_REACHED"


def test_correction_gate_ignores_fast_zero_mean_jitter():
    gate = CorrectionReplanGate(
        translation_trigger=0.05,
        yaw_trigger=math.radians(1.5),
        filter_time_constant=0.20,
    )
    assert not gate.observe((0.0, 0.0, 0.0, 0.0), 0.0)
    for index in range(1, 101):
        sign = 1.0 if index % 2 else -1.0
        triggered = gate.observe((0.03 * sign, 0.0, 0.0, math.radians(sign)), index * 0.02)
        assert not triggered
        assert not gate.waiting(index * 0.02)


def test_correction_gate_coalesces_event_and_requires_fresh_path():
    gate = CorrectionReplanGate(
        translation_trigger=0.05,
        yaw_trigger=math.radians(1.5),
        filter_time_constant=0.10,
        material_translation=0.02,
        material_yaw=math.radians(0.5),
        quiet_time=0.20,
        cooldown=1.0,
    )
    gate.observe((0.0, 0.0, 0.0, 0.0), 0.0)
    trigger_count = 0
    for index in range(1, 31):
        now = index * 0.02
        trigger_count += gate.observe((0.12, 0.0, 0.0, math.radians(2.0)), now)
    assert trigger_count == 1
    assert gate.waiting(0.60)
    gate.path_received(0.61)
    assert not gate.waiting(0.70)

    # A second jump during the cooldown is absorbed into the same optimization
    # episode rather than immediately starving the follower again.
    for index in range(46, 76):
        now = index * 0.02
        assert not gate.observe((-0.12, 0.0, 0.0, 0.0), now)
        assert not gate.waiting(now)
