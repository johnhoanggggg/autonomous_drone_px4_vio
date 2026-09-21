import math
import json
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

from px4_vio_bridge.offboard_lateral_stop import OffboardLateralStop, StopProfile, lateral_xy
from px4_vio_bridge.brake_damping import BrakeDamping


@pytest.mark.parametrize("distance", [0.01, 0.35, 0.50])
def test_profile_derivatives_bounds_and_boundary_continuity(distance):
    profile = StopProfile(distance, 0.2, 0.3)
    dt = 1e-5
    for t in np.linspace(dt, profile.duration-dt, 501):
        p, v, a, _ = profile.sample(t)
        before, after = profile.sample(t-dt), profile.sample(t+dt)
        assert 0 <= p <= distance
        assert 0 <= v <= 0.2 + 1e-12
        assert abs(a) <= 0.3 + 1e-12
        assert (after[0]-before[0])/(2*dt) == pytest.approx(v, abs=1e-7)
        assert (after[1]-before[1])/(2*dt) == pytest.approx(a, abs=1e-5)
    for t in [0, profile.ramp, profile.brake_start, profile.duration]:
        for left, right in zip(profile.sample(t-dt)[:3], profile.sample(t+dt)[:3]):
            assert abs(left-right) < 1e-4
    assert profile.sample(profile.duration+1) == (distance, 0.0, 0.0, "HOLD")


@pytest.mark.parametrize("values", [(0, .2, .3), (.35, -1, .3), (.35, .2, math.nan)])
def test_invalid_profiles_are_refused(values):
    with pytest.raises(ValueError):
        StopProfile(*values)


@pytest.mark.parametrize("yaw", [0, math.pi/2, -2.4])
def test_lateral_direction_is_right_relative_to_initial_heading(yaw):
    x, y = lateral_xy(2, -1, yaw, .35)
    assert (x-2)*math.cos(yaw)+(y+1)*math.sin(yaw) == pytest.approx(0)
    assert -(x-2)*math.sin(yaw)+(y+1)*math.cos(yaw) == pytest.approx(.35)


def stub():
    s = SimpleNamespace(x0=2., y0=-1., yaw0=.7, offset=0., velocity=0.,
                        acceleration_sp=0., auto_arm=True, is_armed=True,
                        hover_height=.3, reach_tol=.07, settle_position=.1,
                        settle_speed=.06, settle_hold=3., settle_dwell=1.,
                        settle_timeout=10., climb_timeout=15., geofence_radius=1.,
                        phase_started=None, stable_since=None, last_update=None,
                        phase="INITIAL_HOLD", state="CLIMB_HOLD", leg=-1,
                        endpoints=[.35, 0., -.35, 0.], start_offset=0.,
                        profile=StopProfile(.35, .2, .3), local_received=0.,
                        clock=0., abort_xy=None, messages=[], transitions=[])
    s.brake_damper = BrakeDamping()
    s.pos = SimpleNamespace(x=2., y=-1., z=-.3, heading=.7,
                            vx=0., vy=0., v_xy_valid=True)
    s.hold_point = (2., -1.)
    s.monotonic_time = lambda: s.clock
    s.pos_valid = lambda: True
    s.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    s.trigger_landing = lambda reason: set_state("LAND")
    def set_state(state):
        s.state = state
        s.transitions.append((s.clock, state, s.leg))
    s.set_state = set_state
    for name in ["enter_phase", "fail", "settled", "handle_flight_state"]:
        setattr(s, name, MethodType(getattr(OffboardLateralStop, name), s))
    s.ramp_z = lambda z: (z, 0.)
    s.now_us = lambda: int(s.clock*1e6)
    s.sp_pub = SimpleNamespace(publish=s.messages.append)
    def publish(z):
        s.hold_point = OffboardLateralStop.hold_point.fget(s)
        OffboardLateralStop.publish_setpoint(s, z)
    s.publish_setpoint = publish
    return s


@pytest.mark.parametrize("damping", [False, True])
def test_four_leg_sequence_holds_heading_and_waits_before_each_move(damping):
    s = stub()
    s.brake_damper.enabled = damping
    for _ in range(1000):
        s.clock += .05
        s.local_received = s.clock
        s.handle_flight_state()
        if s.state == "LAND":
            break
        # Ideal following is only a state-machine fixture, not a dynamics model.
        m = s.messages[-1]
        s.pos.x, s.pos.y = m.position[:2]
        s.pos.vx, s.pos.vy = m.velocity[:2]
    assert s.state == "LAND"
    assert s.offset == pytest.approx(0)
    assert s.leg == 4
    assert sum(state == "LATERAL_MOVE" for _, state, _ in s.transitions) == 4
    for before, after in zip(s.transitions, s.transitions[1:]):
        if before[1] == "LATERAL_HOLD":
            assert after[0]-before[0] >= 3.-1e-8
    for m in s.messages:
        assert m.yaw == pytest.approx(.7)
        assert math.hypot(m.velocity[0], m.velocity[1]) <= .2+1e-7
        assert math.hypot(m.acceleration[0], m.acceleration[1]) <= .3+1e-7
        assert math.hypot(m.position[0]-2, m.position[1]+1) <= .35+1e-6


def test_unsettled_motion_blocks_next_leg_then_lands():
    s = stub()
    s.state = "LATERAL_HOLD"
    s.pos.vx = .1
    for _ in range(203):
        s.clock += .05
        s.local_received = s.clock
        s.handle_flight_state()
    assert s.state == "LAND"
    assert s.leg == -1


@pytest.mark.parametrize("fault", ["stale", "geofence", "callback_gap", "velocity_invalid", "nan"])
def test_faults_land_without_advancing_profile(fault):
    s = stub()
    s.clock = 1.
    s.local_received = 1.
    if fault == "stale":
        s.local_received = 0.
    elif fault == "geofence":
        s.pos.x += 1.1
    elif fault == "velocity_invalid":
        s.pos.v_xy_valid = False
    elif fault == "nan":
        s.pos.vx = math.nan
    else:
        s.last_update = .5
    s.handle_flight_state()
    assert s.state == "LAND"
    assert s.leg == -1
    assert s.velocity == 0


def test_settling_dwell_resets_when_vehicle_moves_again():
    s = stub()
    for _ in range(55):
        s.clock += .05
        s.local_received = s.clock
        s.handle_flight_state()
    s.pos.vy = .1
    s.clock += .05
    s.local_received = s.clock
    s.handle_flight_state()
    assert s.stable_since is None
    s.pos.vy = 0
    for _ in range(15):
        s.clock += .05
        s.local_received = s.clock
        s.handle_flight_state()
    assert s.state == "CLIMB_HOLD"


@pytest.mark.parametrize("kwargs", [dict(gain=-1), dict(gain=math.nan),
    dict(limit=0), dict(enabled=True, limit=.31), dict(time_constant=0),
    dict(time_constant=math.inf), dict(acceleration_limit=math.nan)])
def test_damping_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        BrakeDamping(**kwargs)


@pytest.mark.parametrize("kwargs", [dict(enabled=False), dict(enabled=True, gain=0)])
def test_disabled_damping_preserves_baseline_exactly(kwargs):
    d = BrakeDamping(**kwargs)
    for phase in (False, True):
        for a in [(0., 0.), (.12, -.27), (-.3, 0.)]:
            result = d.update((.1, -.1), a, (.8, -.6), 1., active=True, braking=phase)
            assert result.acceleration == a
            assert result.applied_correction == (0., 0.)


def test_disabled_damping_accepts_existing_lower_acceleration_profiles():
    d = BrakeDamping(enabled=False, acceleration_limit=.05)
    r = d.update((0., 0.), (.03, .04), (.1, 0.), 1., active=True, braking=True)
    assert r.acceleration == (.03, .04)


@pytest.mark.parametrize("yaw", [0., .7, math.pi/2, -2.4])
@pytest.mark.parametrize("sign", [-1, 1])
def test_hold_damping_opposes_motion_in_rotated_ned_frame(yaw, sign):
    d = BrakeDamping(enabled=True)
    direction = (-math.sin(yaw), math.cos(yaw))
    velocity = tuple(sign*.1*x for x in direction)
    for i in range(40):
        result = d.update((0., 0.), (0., 0.), velocity, i*.05,
                          active=True, braking=True)
    assert result.acceleration == pytest.approx(tuple(-.5*x for x in velocity), abs=1e-8)
    assert sum(a*v for a,v in zip(result.acceleration, velocity)) < 0


def test_damping_vector_caps_and_phase_release():
    d = BrakeDamping(enabled=True)
    for i in range(30):
        r = d.update((0., 0.), (-.18, -.24), (.8, .6), i*.05,
                     active=True, braking=True)
        assert math.hypot(*r.filtered_correction) <= .1 + 1e-12
        assert math.hypot(*r.acceleration) <= .3 + 1e-12
        assert math.hypot(*r.applied_correction) <= .1 + 1e-12
    previous = math.hypot(*r.filtered_correction)
    for i in range(30, 60):
        r = d.update((.2, 0.), (0., 0.), (.8, .6), i*.05,
                     active=True, braking=False)
        magnitude = math.hypot(*r.filtered_correction)
        assert magnitude == pytest.approx(previous*math.exp(-.05/.1))
        previous = magnitude
    assert previous < 1e-7


@pytest.mark.parametrize("fault", ["missing", "nan", "inactive", "gap", "backward_time"])
def test_damping_clears_and_restarts_smoothly_after_invalid_input(fault):
    d = BrakeDamping(enabled=True)
    for i in range(10):
        r = d.update((0., 0.), (0., 0.), (.1, 0.), i*.05, active=True, braking=True)
    assert abs(r.acceleration[0]) > .04
    now = .8 if fault == "gap" else (.4 if fault == "backward_time" else .5)
    measured = None if fault == "missing" else ((math.nan, 0.) if fault == "nan" else (.1, 0.))
    r = d.update((0., 0.), (0., 0.), measured, now,
                 active=fault != "inactive", braking=True)
    assert r.acceleration == (0., 0.)
    assert d.previous_time is None
    first = d.update((0., 0.), (0., 0.), (.1, 0.), now+.05, active=True, braking=True)
    second = d.update((0., 0.), (0., 0.), (.1, 0.), now+.10, active=True, braking=True)
    assert first.acceleration == (0., 0.)
    assert -.05 < second.acceleration[0] < 0.


def test_published_damping_keeps_targets_and_exposes_applied_correction():
    base, trial = stub(), stub()
    trial.brake_damper.enabled = True
    rows = []
    trial.config = {}
    trial.debug_pub = SimpleNamespace(publish=lambda m: rows.append(json.loads(m.data)))
    for i in range(12):
        for s in (base, trial):
            s.state = "LATERAL_HOLD"
            s.phase = "HOLD"
            s.clock = i*.05
            s.local_received = s.clock
            s.offset = .35
            s.pos.vx, s.pos.vy = .1, -.1
            s.publish_setpoint(.3)
        a, b = base.messages[-1], trial.messages[-1]
        assert list(a.position) == list(b.position)
        assert list(a.velocity) == list(b.velocity)
        assert a.yaw == b.yaw
        assert math.isnan(a.acceleration[2]) and math.isnan(b.acceleration[2])
    assert trial.messages[-1].acceleration[0] < 0
    assert trial.messages[-1].acceleration[1] > 0
    debug = rows[-1]['damping']
    assert debug['reason'] == 'braking_or_hold'
    assert debug['published_acceleration_ned'] == list(trial.messages[-1].acceleration[:2])
    assert debug['applied_correction_ned'] == pytest.approx(debug['published_acceleration_ned'])


@pytest.mark.parametrize("fault", ["stale", "invalid", "nan", "land", "abort", "climb"])
def test_published_damping_never_leaks_into_invalid_or_inactive_states(fault):
    s = stub()
    s.brake_damper.enabled = True
    s.state, s.phase = "LATERAL_HOLD", "HOLD"
    s.pos.vx = .1
    for i in range(10):
        s.clock = s.local_received = i*.05
        s.publish_setpoint(.3)
    assert s.messages[-1].acceleration[0] < -.04
    s.clock += .05
    s.local_received = s.clock
    if fault == "stale":
        s.local_received -= .11
    elif fault == "invalid":
        s.pos.v_xy_valid = False
    elif fault == "nan":
        s.pos.vy = math.nan
    else:
        s.state = {"land": "LAND", "abort": "ABORT", "climb": "CLIMB_HOLD"}[fault]
    s.publish_setpoint(.3)
    assert list(s.messages[-1].acceleration[:2]) == [0., 0.]
    assert s.brake_damper.previous_time is None
