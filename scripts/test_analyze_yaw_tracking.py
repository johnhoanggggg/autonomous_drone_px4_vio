"""Offline regression checks for frame conventions and telemetry matching."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from analyze_yaw_tracking import (
    RecentSamples, analyse_data, angle_stats, enu_yaw_to_ned, yaw_of,
)


def test_enu_to_ned_cardinal_headings():
    # East, north, west, south in both angle conventions.
    for enu, ned in [(0, 90), (90, 0), (180, -90), (-90, -180)]:
        assert enu_yaw_to_ned(math.radians(enu)) == pytest.approx(math.radians(ned))


def test_pairing_does_not_extrapolate_or_look_into_future():
    stream = RecentSamples([(1_000_000_000, "old"), (3_000_000_000, "new")])
    assert stream.at(900_000_000) is None
    assert stream.at(1_300_000_000) == "old"
    assert stream.at(2_900_000_000) is None
    assert stream.at(3_000_000_000) == "new"


def test_angle_statistics_wrap_and_exclude_nan():
    stats = angle_stats(np.radians([179, -179, math.nan]))
    assert stats["samples"] == 2
    assert abs(stats["mean_deg"]) == pytest.approx(180)
    assert stats["sd_deg"] == pytest.approx(1, abs=.01)


def test_invalid_quaternion_is_not_a_heading():
    pose = NS(pose=NS(orientation=NS(x=0, y=0, z=0, w=0)))
    assert math.isnan(yaw_of(pose))


def test_aligned_ned_pose_and_divergent_wire_commands_are_distinguished():
    pose = NS(x=0, y=0, vx=.2, vy=0, heading=0, xy_valid=True, v_xy_valid=True)
    body = NS(pose=NS(orientation=NS(x=0, y=0, z=math.sqrt(.5), w=math.sqrt(.5))))
    samples = [(int(t * 1e9), pose) for t in [1, 2, 3, 4]]
    data = {
        "/fmu/out/vehicle_local_position_v1": samples,
        "/planner/flight/status": [
            (1_000_000_000, NS(data="CLIMB")),
            (2_000_000_000, NS(data="ROUTE yaw_target=0deg")),
            (3_000_000_000, NS(data="LAND")),
        ],
        "/rtabmap/body_vio_pose": [(2_000_000_000, body)],
        "/fmu/in/trajectory_setpoint": [
            (2_000_000_000, NS(position=[.1, 0, 0], velocity=[0, .2, 0], yaw=0))],
        "/planner/follower/vio_displacement": [
            (2_000_000_000, NS(vector=NS(x=0, y=.2)))],
    }
    result = analyse_data(data, 1_000_000_000)
    assert result["sample_counts"]["moving_route"] == 1
    angles = result["angles"]
    assert angles["body_vio_ned_minus_heading"]["median_abs_deg"] == 0
    assert angles["heading_minus_wire_yaw"]["median_abs_deg"] == 0
    assert angles["follower_minus_feedforward"]["median_abs_deg"] == 90
    assert angles["wire_yaw_minus_feedforward"]["median_abs_deg"] == 90
