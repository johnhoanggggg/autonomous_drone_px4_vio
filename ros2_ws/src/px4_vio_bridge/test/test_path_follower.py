import copy
import math

import pytest

from px4_vio_bridge.path_follower import (
    CorrectionReplanGate,
    Polyline,
    PositionRouteFollower,
    correction_rejection_reason,
    map_displacement_to_vio,
    requested_goal_reached,
    yaw_from_quaternion,
)


def test_yaw_from_quaternion():
    yaw = math.radians(-115.0)
    quaternion = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    assert yaw_from_quaternion(quaternion) == pytest.approx(yaw)


def test_invalid_correction_quaternion_is_rejected():
    yaw = yaw_from_quaternion((0.0, 0.0, 0.0, 0.0))
    assert correction_rejection_reason((0.0, 0.0, 0.0, yaw)) == (
        "correction contains a non-finite value"
    )
    non_unit_yaw = yaw_from_quaternion((0.5, 0.0, 0.0, 0.0))
    assert correction_rejection_reason((0.0, 0.0, 0.0, non_unit_yaw)) == (
        "correction contains a non-finite value"
    )


def test_native_correction_safety_limits():
    assert correction_rejection_reason((0.2, 0.0, 0.0, 0.0)) is None
    assert "max_correction_m" in correction_rejection_reason(
        (0.9, 0.0, 0.0, 0.0)
    )
    assert "max_correction_yaw_deg" in correction_rejection_reason(
        (0.0, 0.0, 0.0, math.radians(40.0))
    )
    assert correction_rejection_reason((math.nan, 0.0, 0.0, 0.0)) == (
        "correction contains a non-finite value"
    )


def test_map_displacement_is_rotated_back_into_continuous_vio_axes():
    assert map_displacement_to_vio((1.0, 0.0), math.pi / 2.0) == pytest.approx(
        (0.0, -1.0)
    )
    assert map_displacement_to_vio((0.4, -0.2), 0.0) == pytest.approx((0.4, -0.2))


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


def test_update_can_reduce_lookahead_for_a_single_safe_command():
    follower = PositionRouteFollower(
        lookahead=0.60, max_carrot_speed=10.0, max_carrot_acceleration=100.0
    )
    follower.set_path(((0.0, 0.0), (2.0, 0.0)), (0.0, 0.0))
    result = follower.update((0.0, 0.0), 0.1, lookahead=0.20)
    assert result.desired_carrot == pytest.approx((0.20, 0.0))
    assert follower.lookahead == pytest.approx(0.60)


def test_unsafe_command_is_invalid_and_resets_relative_proposal():
    follower = PositionRouteFollower(
        lookahead=0.60, max_carrot_speed=10.0, max_carrot_acceleration=100.0
    )
    follower.set_path(((0.0, 0.0), (2.0, 0.0)), (0.0, 0.0))
    result = follower.update(
        (0.0, 0.0),
        0.1,
        command_validator=lambda carrot: carrot[0] <= 0.10,
    )
    assert result.status == "CLEARANCE_BLOCKED"
    assert not result.valid
    assert result.commanded_displacement == (0.0, 0.0)
    assert result.commanded_carrot == (0.0, 0.0)
    assert result.progress == 0.0


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


def test_cross_track_fault_requires_fresh_path_and_stable_lower_error():
    follower = PositionRouteFollower(
        max_cross_track=0.10,
        cross_track_resume=0.05,
        cross_track_recovery_time=0.30,
    )
    follower.set_path(((0.0, 0.0), (3.0, 0.0)), (0.0, 0.0))
    before = follower.commanded_displacement

    fault = follower.update((1.0, 0.11), 0.10)
    assert fault.status == "CROSS_TRACK_EXCEEDED"
    assert not fault.valid

    # Merely dipping below both thresholds on the same path cannot restart.
    waiting = follower.update((1.0, 0.04), 0.10)
    assert waiting.status == "CROSS_TRACK_HOLD_WAITING_FOR_PATH"
    assert not waiting.valid

    # A replan anchored at the held pose is necessary but not sufficient: the
    # lower error must then remain healthy for the entire recovery interval.
    assert follower.set_path(((1.0, 0.04), (3.0, 0.0)), (1.0, 0.04))
    first = follower.update((1.0, 0.04), 0.10)
    second = follower.update((1.0, 0.04), 0.10)
    assert first.status == second.status == "CROSS_TRACK_RECOVERING"
    assert not first.valid and not second.valid
    assert first.commanded_displacement == second.commanded_displacement == before

    recovered = follower.update((1.0, 0.04), 0.10)
    assert recovered.status == "FOLLOWING"
    assert recovered.valid


def test_cross_track_recovery_is_continuous_and_has_hysteresis():
    follower = PositionRouteFollower(
        max_cross_track=0.10,
        cross_track_resume=0.05,
        cross_track_recovery_time=0.20,
    )
    follower.set_path(((0.0, 0.0), (3.0, 0.0)), (0.0, 0.0))
    follower.update((1.0, 0.11), 0.10)
    follower.set_path(((1.0, 0.0), (3.0, 0.0)), (1.0, 0.04))

    assert follower.update((1.0, 0.04), 0.10).status == "CROSS_TRACK_RECOVERING"
    # Below the 0.10 m trip threshold but above the 0.05 m resume threshold:
    # remain latched and discard the accumulated healthy interval.
    held = follower.update((1.0, 0.06), 0.10)
    assert held.status == "CROSS_TRACK_HOLD"
    assert not held.valid
    assert follower.update((1.0, 0.04), 0.10).status == "CROSS_TRACK_RECOVERING"

    follower.interrupt_cross_track_recovery()
    assert follower.update((1.0, 0.04), 0.10).status == "CROSS_TRACK_RECOVERING"
    assert follower.update((1.0, 0.04), 0.10).valid


def test_new_requested_goal_resets_cross_track_latch():
    follower = PositionRouteFollower(max_cross_track=0.10)
    follower.set_path(((0.0, 0.0), (3.0, 0.0)), (0.0, 0.0))
    assert not follower.update((1.0, 0.11), 0.10).valid
    follower.reset_route_progress()
    follower.clear_path()
    follower.set_path(((1.0, 0.11), (2.0, 1.0)), (1.0, 0.11))
    assert follower.update((1.0, 0.11), 0.10).valid


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


def test_arrival_latches_through_replan_jitter_at_the_threshold():
    # 20260810T105345Z: near the goal the planner replanned ~2 Hz and the path
    # endpoint moved a few cm per generation, so the euclidean term crossed a
    # 0.12m tolerance by 1-2mm and dropped GOAL_REACHED.  Each drop restarted
    # the adapter's 3.0s hold, which took 9.5s to satisfy.
    follower = PositionRouteFollower(
        arrival_tolerance=0.12, arrival_release_tolerance=0.20
    )
    follower.set_path(((0.0, 0.0), (1.0, 0.0)), (0.0, 0.0))
    assert follower.update((0.90, 0.0), 0.1).status == "GOAL_REACHED"

    # The endpoint jitters out to 0.119m, then 0.120m -- inside the release band.
    follower.set_path(((0.0, 0.0), (1.019, 0.0)), (0.90, 0.0))
    assert follower.update((0.90, 0.0), 0.1).status == "GOAL_REACHED"
    follower.set_path(((0.0, 0.0), (1.020, 0.0)), (0.90, 0.0))
    assert follower.update((0.90, 0.0), 0.1).status == "GOAL_REACHED"


def test_arrival_releases_once_past_the_release_tolerance():
    follower = PositionRouteFollower(
        arrival_tolerance=0.12, arrival_release_tolerance=0.20
    )
    follower.set_path(((0.0, 0.0), (1.0, 0.0)), (0.0, 0.0))
    assert follower.update((0.90, 0.0), 0.1).status == "GOAL_REACHED"
    follower.set_path(((0.0, 0.0), (1.25, 0.0)), (0.90, 0.0))
    assert follower.update((0.90, 0.0), 0.1).status == "FOLLOWING"


def test_arrival_still_requires_the_tight_tolerance_to_latch():
    follower = PositionRouteFollower(
        arrival_tolerance=0.12, arrival_release_tolerance=0.20
    )
    follower.set_path(((0.0, 0.0), (1.0, 0.0)), (0.0, 0.0))
    # 0.15m out: inside the release band but never latched, so it must not count.
    assert follower.update((0.85, 0.0), 0.1).status == "FOLLOWING"


def test_arrival_latch_clears_with_the_path_and_the_route():
    for reset in ("clear_path", "reset_route_progress"):
        follower = PositionRouteFollower(
            arrival_tolerance=0.12, arrival_release_tolerance=0.20
        )
        follower.set_path(((0.0, 0.0), (1.0, 0.0)), (0.0, 0.0))
        assert follower.update((0.90, 0.0), 0.1).status == "GOAL_REACHED"
        getattr(follower, reset)()
        follower.set_path(((0.0, 0.0), (1.0, 0.0)), (0.85, 0.0))
        assert follower.update((0.85, 0.0), 0.1).status == "FOLLOWING"


def test_release_tolerance_below_arrival_tolerance_is_rejected():
    with pytest.raises(ValueError):
        PositionRouteFollower(
            arrival_tolerance=0.20, arrival_release_tolerance=0.12
        )


def test_exploration_frontier_is_not_reported_as_requested_goal_reached():
    assert not requested_goal_reached("GOAL_REACHED", goal_terminal=False)
    assert requested_goal_reached("GOAL_REACHED", goal_terminal=True)
    assert not requested_goal_reached("FOLLOWING", goal_terminal=True)


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
