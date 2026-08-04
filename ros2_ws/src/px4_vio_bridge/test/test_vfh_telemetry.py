"""Tests for the Foxglove telemetry geometry.

The markers are what the operator will believe, so the frame conversions behind
them are worth the same scrutiny as the planner: an arrow drawn on the wrong
side of the drone is a worse failure than no arrow at all.
"""
import math

from px4_vio_bridge.vfh2d import Vfh2D, VfhConfig, sector_center, sector_index
from px4_vio_bridge.vfh_telemetry import (
    enu_heading,
    ned_heading_deg,
    opening_width,
    polar_to_enu,
    sector_rays,
)


def wall(bearing_deg, distance, spread_deg=20.0, points=120):
    half = math.radians(spread_deg) / 2.0
    center = math.radians(bearing_deg)
    return [
        (distance, center - half + 2.0 * half * i / max(1, points - 1))
        for i in range(points)
    ]


def test_a_bearing_to_the_right_is_drawn_to_the_right() -> None:
    """Facing east (ENU yaw 0), a +90 deg body bearing points south."""
    x, y, z = polar_to_enu((0.0, 0.0, 0.3), 0.0, math.radians(90.0), 2.0)

    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, -2.0, abs_tol=1e-9)
    assert math.isclose(z, 0.3)


def test_straight_ahead_is_drawn_along_the_heading() -> None:
    x, y, _ = polar_to_enu((1.0, -2.0, 0.0), math.radians(90.0), 0.0, 3.0)

    assert math.isclose(x, 1.0, abs_tol=1e-9)      # ENU yaw 90 deg = north
    assert math.isclose(y, 1.0, abs_tol=1e-9)


def test_enu_heading_is_the_inverse_of_the_bearing_convention() -> None:
    yaw_enu = math.radians(33.0)
    for bearing_deg in (-150.0, -35.0, 0.0, 35.0, 150.0):
        bearing = math.radians(bearing_deg)
        heading = enu_heading(yaw_enu, bearing)
        # Re-deriving the bearing from the drawn heading must give it back.
        assert math.isclose(
            math.atan2(math.sin(yaw_enu - heading), math.cos(yaw_enu - heading)),
            bearing,
            abs_tol=1e-12,
        )


def test_headings_are_published_in_px4_ned_degrees() -> None:
    # ENU yaw 0 = facing east = NED heading 90.
    assert math.isclose(ned_heading_deg(0.0), 90.0, abs_tol=1e-9)
    # ENU yaw 90 = facing north = NED heading 0.
    assert math.isclose(ned_heading_deg(math.radians(90.0)), 0.0, abs_tol=1e-9)
    # A body bearing adds on in NED: 35 deg right of north is heading 35.
    assert math.isclose(
        ned_heading_deg(math.radians(90.0), math.radians(35.0)), 35.0, abs_tol=1e-9
    )


def test_sector_rays_are_drawn_at_the_range_each_sector_measured() -> None:
    """A ray ends on the obstacle, not at max_range, or the fan is a lie."""
    vfh = Vfh2D(VfhConfig())
    result = vfh.update(wall(0.0, 1.6, spread_deg=20.0))
    origin = (0.0, 0.0, 0.3)

    rays = sector_rays(result, origin, 0.0, 3.0)
    ahead = rays[sector_index(0.0, 72)]
    behind = rays[sector_index(math.pi, 72)]

    assert math.isclose(math.hypot(*ahead[1][:2]), 1.6, abs_tol=0.02)
    assert ahead[2] == 1                              # and drawn as blocked
    assert math.isclose(math.hypot(*behind[1][:2]), 3.0, abs_tol=1e-9)
    # Outside the steering FOV is non-flyable, but it is not falsely drawn as
    # a detected obstacle.
    assert behind[2] == 0


def test_sector_rays_outside_display_fov_are_omitted() -> None:
    cfg = VfhConfig(max_steer=math.radians(35.0))
    result = Vfh2D(cfg).update([])

    rays = sector_rays(
        result, (0.0, 0.0, 0.3), 0.0, cfg.max_range, cfg.max_steer
    )

    expected = sum(
        abs(sector_center(i, cfg.sectors)) <= cfg.max_steer + 1e-9
        for i in range(cfg.sectors)
    )
    assert len(rays) == expected
    assert all(abs(bearing) <= cfg.max_steer + 1e-9 for bearing, _, _ in rays)


def test_display_can_be_wider_than_steering_without_false_red_rays() -> None:
    cfg = VfhConfig(max_steer=math.radians(35.0))
    result = Vfh2D(cfg).update([])

    rays = sector_rays(
        result, (0.0, 0.0, 0.3), 0.0, cfg.max_range, math.radians(90.0)
    )

    assert all(abs(bearing) <= math.radians(90.0) + 1e-9 for bearing, _, _ in rays)
    assert any(abs(bearing) > cfg.max_steer for bearing, _, _ in rays)
    assert all(not obstacle_blocked for _, _, obstacle_blocked in rays)


def test_opening_width_measures_the_gap_actually_chosen() -> None:
    binary = [0] * 72
    for i in range(40, 50):
        binary[i] = 1

    # Sector 39 is the last free one before that wall: the free run wraps all
    # the way round from 50 back to 39, i.e. 62 sectors.
    assert math.isclose(
        opening_width(binary, math.radians(-180.0 + 39.5 * 5.0)),
        math.radians(62 * 5.0),
        abs_tol=1e-9,
    )


def test_opening_width_is_zero_when_the_direction_is_blocked() -> None:
    binary = [0] * 72
    binary[36] = 1

    assert opening_width(binary, 0.0) == 0.0
    assert opening_width([0] * 72, None) == 0.0


def test_opening_width_of_a_completely_free_histogram_is_the_full_circle() -> None:
    assert math.isclose(opening_width([0] * 72, 0.0), 2.0 * math.pi, abs_tol=1e-9)
