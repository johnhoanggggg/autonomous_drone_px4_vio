"""Unit tests for the VFH2D algorithm and the cloud -> samples conversion.

Everything here runs without ROS running, without a camera and without a
vehicle: the algorithm is a pure function of range data by construction, which
is the only reason it can be trusted before it is ever armed.
"""
import math
import struct

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField

from px4_vio_bridge.vfh2d import (
    Vfh2D,
    VfhConfig,
    angle_between,
    histogram_bar,
    relative_bearing_enu,
    relative_bearing_ned,
    sector_center,
    sector_index,
    wrap_pi,
)
from px4_vio_bridge.vfh_obstacles import (
    ObstacleField,
    WorldObstacleMemory,
    cloud_to_samples,
)


def config(**overrides):
    defaults = dict(
        sectors=72,
        min_range=0.25,
        max_range=3.0,
        min_points=3,
        tau_high=2.0,
        tau_low=1.0,
        smoothing=3,
        robot_radius=0.25,
        safety_margin=0.25,
        max_steer=math.radians(35.0),
        wide_valley=math.radians(40.0),
    )
    defaults.update(overrides)
    return VfhConfig(**defaults)


def wall(bearing_deg, distance, spread_deg=10.0, points=120):
    """A dense patch of returns centred on a bearing, as the stereo cloud gives."""
    half = math.radians(spread_deg) / 2.0
    center = math.radians(bearing_deg)
    return [
        (distance, center - half + 2.0 * half * i / max(1, points - 1))
        for i in range(points)
    ]


# --- geometry conventions ---------------------------------------------------
def test_sector_index_and_center_round_trip() -> None:
    cfg = config()
    for bearing_deg in (-179.0, -90.0, -2.5, 0.0, 2.5, 37.0, 179.0):
        bearing = math.radians(bearing_deg)
        i = sector_index(bearing, cfg.sectors)
        assert 0 <= i < cfg.sectors
        assert angle_between(sector_center(i, cfg.sectors), bearing) <= (
            cfg.sector_width / 2.0 + 1e-9
        )


def test_enu_bearing_puts_left_of_the_nose_negative() -> None:
    # Facing east (ENU yaw 0). A point due north is to the vehicle's LEFT.
    assert math.isclose(relative_bearing_enu(0.0, 0.0, 1.0), math.radians(-90.0))
    # Straight ahead is zero, and to the south is to the right.
    assert math.isclose(relative_bearing_enu(0.0, 1.0, 0.0), 0.0)
    assert math.isclose(relative_bearing_enu(0.0, 0.0, -1.0), math.radians(90.0))


def test_ned_bearing_does_not_mirror_the_enu_one() -> None:
    # Heading north (NED heading 0). A point due east is to the RIGHT.
    assert math.isclose(relative_bearing_ned(0.0, 0.0, 1.0), math.radians(90.0))
    assert math.isclose(relative_bearing_ned(0.0, 1.0, 0.0), 0.0)
    # And an absolute heading is recovered by adding the bearing back on.
    heading = math.radians(37.0)
    bearing = relative_bearing_ned(heading, 1.0, 1.0)
    assert math.isclose(
        wrap_pi(heading + bearing), math.atan2(1.0, 1.0), abs_tol=1e-12
    )


# --- histogram and thresholding --------------------------------------------
def test_open_room_steers_at_the_target() -> None:
    vfh = Vfh2D(config())
    result = vfh.update([], target_bearing=math.radians(20.0))

    assert not result.blocked
    assert math.isclose(result.direction, math.radians(20.0), abs_tol=1e-9)


def test_target_beyond_the_steer_limit_is_clamped_not_refused() -> None:
    """A goal behind the vehicle must turn it, not report it as boxed in."""
    vfh = Vfh2D(config())
    result = vfh.update([], target_bearing=math.radians(150.0))

    assert not result.blocked
    assert math.isclose(result.direction, math.radians(35.0), abs_tol=1e-9)


def test_wall_seen_early_is_steered_around() -> None:
    """At 2.5 m the vehicle's half-width subtends ~12 deg, so a gap remains."""
    vfh = Vfh2D(config())
    result = vfh.update(wall(0.0, 2.5, spread_deg=20.0), target_bearing=0.0)

    assert not result.blocked
    assert abs(result.direction) >= math.radians(20.0)
    assert result.binary[sector_index(0.0, 72)] == 1


def test_wall_seen_late_blocks_the_whole_field_of_view() -> None:
    """The same wall at 1.0 m cannot be dodged inside a 70 deg camera.

    This is the honest answer, not a tuning failure: half a metre of required
    clearance at 1 m subtends 30 deg, which added to the wall's own width
    covers everything within max_steer. The flight node's response is to hold
    position, not to squeeze past.
    """
    vfh = Vfh2D(config())
    result = vfh.update(wall(0.0, 1.0, spread_deg=20.0), target_bearing=0.0)

    assert result.blocked
    assert result.direction is None


def test_obstacle_beyond_a_short_goal_does_not_block_the_path() -> None:
    """An obstacle beyond the endpoint cannot block an infinite continuation."""
    cfg = config()
    samples = wall(0.0, 1.5, spread_deg=20.0)

    infinite = Vfh2D(cfg).update(samples, target_bearing=0.0)
    finite = Vfh2D(cfg).update(
        samples, target_bearing=0.0, target_distance=0.8
    )

    assert not math.isclose(infinite.direction, 0.0, abs_tol=1e-9)
    assert not finite.blocked
    assert math.isclose(finite.direction, 0.0, abs_tol=1e-9)


def test_obstacle_inside_endpoint_safety_radius_still_blocks_that_heading() -> None:
    cfg = config()
    result = Vfh2D(cfg).update(
        wall(0.0, 1.2, spread_deg=10.0),
        target_bearing=0.0,
        target_distance=0.8,
    )

    assert result.binary[sector_index(0.0, cfg.sectors)] == 1
    assert not math.isclose(result.direction, 0.0, abs_tol=1e-9)


def test_lateral_obstacle_beyond_goal_uses_endpoint_clearance() -> None:
    """Regression for the live scene: 1.37 m at +23 deg, goal 0.88 m ahead."""
    cfg = config()
    result = Vfh2D(cfg).update(
        wall(23.0, 1.37, spread_deg=5.0),
        target_bearing=0.0,
        target_distance=0.88,
    )

    assert not result.blocked
    assert math.isclose(result.direction, 0.0, abs_tol=1e-9)


def test_obstacle_on_the_right_steers_left() -> None:
    vfh = Vfh2D(config())
    result = vfh.update(wall(15.0, 1.0, spread_deg=25.0), target_bearing=0.0)

    assert not result.blocked
    assert result.direction < 0.0


def test_a_few_stray_points_do_not_block_a_sector() -> None:
    """Two returns are VIO speckle, not a wall — min_points exists for this."""
    vfh = Vfh2D(config(min_points=3))
    result = vfh.update([(0.5, 0.0), (0.5, 0.001)], target_bearing=0.0)

    assert not result.blocked
    assert math.isclose(result.direction, 0.0, abs_tol=1e-9)
    assert result.binary[sector_index(0.0, 72)] == 0


def test_sectors_outside_camera_fov_are_not_flyable() -> None:
    cfg = config(max_steer=math.radians(35.0))
    result = Vfh2D(cfg).update([])

    for i, blocked in enumerate(result.binary):
        if abs(sector_center(i, cfg.sectors)) > cfg.max_steer + 1e-9:
            assert blocked
        else:
            assert not blocked


def test_fov_mask_is_not_reported_as_a_physical_obstacle() -> None:
    cfg = config(max_steer=math.radians(35.0))
    result = Vfh2D(cfg).update([])

    assert not any(result.obstacle_binary)
    assert any(result.binary)


def test_clear_opening_is_limited_to_camera_fov() -> None:
    cfg = config(max_steer=math.radians(35.0))
    result = Vfh2D(cfg).update([])

    free_sectors = sum(not blocked for blocked in result.binary)
    assert free_sectors == 14
    assert math.isclose(
        free_sectors * cfg.sector_width, 2.0 * cfg.max_steer, abs_tol=1e-9
    )


def test_enlargement_widens_a_single_close_obstacle() -> None:
    """One point at 0.7 m must block far more than its own 5 deg sector."""
    cfg = config(robot_radius=0.25, safety_margin=0.25)
    vfh = Vfh2D(cfg)
    result = vfh.update(wall(0.0, 0.7, spread_deg=1.0, points=10))

    expected = math.asin(cfg.enlargement_radius / 0.7)
    blocked = [i for i, b in enumerate(result.binary) if b]
    span = len(blocked) * cfg.sector_width
    assert span >= 2.0 * expected
    assert result.blocked or abs(result.direction) > expected


def test_returns_closer_than_the_vehicle_radius_block_a_full_half_plane() -> None:
    vfh = Vfh2D(config())
    result = vfh.update(wall(0.0, 0.3, spread_deg=1.0, points=10))

    # asin() saturates: nothing within +/-90 deg of that return is flyable.
    assert result.blocked
    assert result.direction is None


def test_out_of_range_samples_are_ignored() -> None:
    vfh = Vfh2D(config(min_range=0.25, max_range=3.0))
    result = vfh.update(
        wall(0.0, 0.1, points=50) + wall(0.0, 5.0, points=50) + [(float("nan"), 0.0)]
    )

    assert result.sample_count == 0
    assert not result.blocked


def test_hysteresis_keeps_a_marginal_sector_blocked() -> None:
    """Density between tau_low and tau_high must not flip state every cycle."""
    cfg = config(tau_high=2.0, tau_low=1.0, smoothing=1, min_points=1)
    vfh = Vfh2D(cfg)
    heavy = [(1.5, 0.0)] * 5     # 5 * (1 - 1.5/3) = 2.5 > tau_high
    light = [(1.5, 0.0)] * 3     # 1.5: between the thresholds

    assert vfh.update(heavy).binary[sector_index(0.0, cfg.sectors)] == 1
    assert vfh.update(light).binary[sector_index(0.0, cfg.sectors)] == 1
    # Only a clear drop below tau_low releases it.
    assert vfh.update([(1.5, 0.0)]).binary[sector_index(0.0, cfg.sectors)] == 0


def test_reset_clears_the_hysteresis_state() -> None:
    cfg = config(tau_high=2.0, tau_low=1.0, smoothing=1, min_points=1)
    vfh = Vfh2D(cfg)
    vfh.update([(1.5, 0.0)] * 5)
    vfh.reset()

    assert vfh.update([(1.5, 0.0)] * 3).binary[sector_index(0.0, cfg.sectors)] == 0


# --- valley selection -------------------------------------------------------
def test_a_gap_narrower_than_the_vehicle_is_rejected() -> None:
    """0.3 m of clearance at 1.2 m is not a doorway for a 1.0 m-wide envelope."""
    cfg = config(robot_radius=0.25, safety_margin=0.25)
    vfh = Vfh2D(cfg)
    gap_half = math.degrees(math.atan2(0.15, 1.2))
    samples = (
        wall(gap_half + 8.0, 1.2, spread_deg=16.0)
        + wall(-gap_half - 8.0, 1.2, spread_deg=16.0)
    )
    result = vfh.update(samples, target_bearing=0.0)

    assert result.blocked or abs(result.direction) > math.radians(gap_half)


def test_a_gap_wider_than_the_vehicle_is_flown_through() -> None:
    cfg = config(robot_radius=0.25, safety_margin=0.25)
    vfh = Vfh2D(cfg)
    samples = wall(50.0, 1.5, spread_deg=30.0) + wall(-50.0, 1.5, spread_deg=30.0)
    result = vfh.update(samples, target_bearing=0.0)

    assert not result.blocked
    assert abs(result.direction) < math.radians(15.0)


def test_fully_enclosed_reports_blocked_with_no_direction() -> None:
    vfh = Vfh2D(config())
    samples = []
    for bearing_deg in range(-180, 180, 5):
        samples.extend(wall(bearing_deg, 0.8, spread_deg=5.0, points=10))
    result = vfh.update(samples, target_bearing=0.0)

    assert result.blocked
    assert result.direction is None
    assert "no free direction" in result.reason


def test_no_direction_ever_leaves_the_camera_field_of_view() -> None:
    cfg = config(max_steer=math.radians(35.0))
    vfh = Vfh2D(cfg)
    for target_deg in range(-180, 180, 15):
        result = vfh.update(
            wall(0.0, 2.5, spread_deg=20.0), target_bearing=math.radians(target_deg)
        )
        if result.direction is not None:
            assert abs(result.direction) < cfg.max_steer
            assert result.binary[sector_index(result.direction, cfg.sectors)] == 0
            assert all(abs(candidate) < cfg.max_steer for candidate in result.candidates)


def test_non_aligned_fov_limit_is_respected_exactly() -> None:
    cfg = config(max_steer=math.radians(33.0))
    result = Vfh2D(cfg).update([], target_bearing=math.pi / 2.0)

    assert math.isclose(result.direction, cfg.max_steer, abs_tol=1e-12)
    assert result.direction < cfg.max_steer
    assert result.binary[sector_index(result.direction, cfg.sectors)] == 0


def test_previous_direction_breaks_a_symmetric_tie() -> None:
    """Without this term a wall dead ahead makes the vehicle weave left/right."""
    cfg = config(mu_previous=2.0)
    samples = wall(0.0, 2.5, spread_deg=20.0)

    left = Vfh2D(cfg).update(samples, 0.0, previous_direction=math.radians(-30.0))
    right = Vfh2D(cfg).update(samples, 0.0, previous_direction=math.radians(30.0))

    assert left.direction < 0.0
    assert right.direction > 0.0


def test_min_range_in_cone_ignores_obstacles_off_to_the_side() -> None:
    vfh = Vfh2D(config())
    result = vfh.update(wall(90.0, 0.6, spread_deg=10.0) + wall(0.0, 2.0, spread_deg=10.0))

    assert math.isclose(result.nearest_range, 0.6, abs_tol=1e-6)
    assert math.isclose(
        result.min_range_in_cone(math.radians(35.0)), 2.0, abs_tol=1e-6
    )


def test_histogram_bar_marks_the_chosen_direction() -> None:
    vfh = Vfh2D(config())
    bar = histogram_bar(vfh.update(wall(0.0, 2.5, spread_deg=20.0)))

    assert len(bar) == 72
    assert "#" in bar and "^" in bar


def test_invalid_configuration_is_rejected_at_construction() -> None:
    with pytest.raises(ValueError):
        VfhConfig(smoothing=2)
    with pytest.raises(ValueError):
        VfhConfig(tau_high=1.0, tau_low=2.0)
    with pytest.raises(ValueError):
        VfhConfig(min_range=3.0, max_range=1.0)


def test_an_isolated_narrow_obstacle_is_not_smoothed_away() -> None:
    """Regression: a flat smoothing window erased this.

    36 stereo returns off a wall edge at 2.5 m, in two sectors with empty
    neighbours. Averaging them flat over three sectors put the density under
    tau and reported a solid obstacle as free; the triangular kernel does not.
    """
    vfh = Vfh2D(config())
    result = vfh.update(wall(30.0, 2.5, spread_deg=5.0, points=36))

    assert result.binary[sector_index(math.radians(30.0), 72)] == 1


def test_a_gap_the_vehicle_fits_through_is_not_closed_by_sector_rounding() -> None:
    """Regression: rounding the enlargement up to whole sectors closed this.

    A 1.4 m gap at 2.2 m needs 12.5 deg of clearance either side of centre and
    has 17.6 deg, so it is flyable; inflating each edge to the next whole
    sector took it away.
    """
    cfg = config()
    vfh = Vfh2D(cfg)
    edge = math.degrees(math.atan2(0.7, 2.2))
    samples = (
        wall(edge + 10.0, 2.2, spread_deg=20.0)
        + wall(-edge - 10.0, 2.2, spread_deg=20.0)
    )
    result = vfh.update(samples, target_bearing=0.0)

    assert not result.blocked
    assert abs(result.direction) < math.radians(5.0)


# --- cloud -> samples -------------------------------------------------------
def test_cloud_to_samples_matches_the_scalar_bearing_convention() -> None:
    rng = np.random.default_rng(7)
    origin = (1.0, -2.0, 0.4)
    yaw_enu = math.radians(33.0)
    points = np.column_stack(
        (
            rng.uniform(-2.0, 2.0, 40) + origin[0],
            rng.uniform(-2.0, 2.0, 40) + origin[1],
            np.full(40, origin[2]),
        )
    )

    samples, _, _, _ = cloud_to_samples(
        points, origin, yaw_enu, min_range=0.0, max_range=100.0,
        z_below=1.0, z_above=1.0,
    )

    assert len(samples) == 40
    for (r, bearing), point in zip(samples, points):
        dx, dy = point[0] - origin[0], point[1] - origin[1]
        assert math.isclose(r, math.hypot(dx, dy), rel_tol=1e-9)
        assert math.isclose(
            bearing, relative_bearing_enu(yaw_enu, dx, dy), abs_tol=1e-9
        )


def test_cloud_to_samples_applies_the_height_slab_and_range_band() -> None:
    origin = (0.0, 0.0, 0.5)
    points = np.array(
        [
            [1.0, 0.0, 0.5],    # kept
            [1.0, 0.0, -0.5],   # 1.0 m below: floor, dropped
            [1.0, 0.0, 2.0],    # ceiling, dropped
            [0.1, 0.0, 0.5],    # inside min_range, dropped
            [9.0, 0.0, 0.5],    # beyond max_range, dropped
        ]
    )

    samples, nearest, _, kept = cloud_to_samples(
        points, origin, 0.0, min_range=0.25, max_range=3.0,
        z_below=0.35, z_above=0.60,
    )

    assert kept == 1
    assert len(samples) == 1
    assert math.isclose(nearest, 1.0, abs_tol=1e-9)


def test_cloud_to_samples_decimates_but_keeps_the_exact_nearest_range() -> None:
    """Safety numbers must not depend on which points survived decimation."""
    origin = (0.0, 0.0, 0.0)
    far = np.column_stack(
        (np.full(5000, 2.0), np.linspace(-1.0, 1.0, 5000), np.zeros(5000))
    )
    near = np.array([[0.4, 0.0, 0.0]])
    points = np.vstack((far, near))

    samples, nearest, nearest_bearing, kept = cloud_to_samples(
        points, origin, 0.0, min_range=0.25, max_range=3.0,
        z_below=1.0, z_above=1.0, max_samples=500,
    )

    assert kept == 5001
    assert len(samples) <= 500
    assert math.isclose(nearest, 0.4, abs_tol=1e-9)
    assert math.isclose(nearest_bearing, 0.0, abs_tol=1e-9)


def test_empty_cloud_is_not_an_error() -> None:
    samples, nearest, bearing, kept = cloud_to_samples(
        np.zeros((0, 3)), (0.0, 0.0, 0.0), 0.0,
        min_range=0.25, max_range=3.0, z_below=0.3, z_above=0.3,
    )

    assert samples == []
    assert kept == 0
    assert nearest == math.inf
    assert bearing is None


# --- world-frame obstacle memory -------------------------------------------
def test_obstacle_memory_survives_yawing_the_camera_away() -> None:
    now = [0.0]
    memory = WorldObstacleMemory(
        duration=30.0, voxel_size=0.10, max_points=100, clock=lambda: now[0]
    )
    memory.update(np.array([[1.0, 0.0, 0.0]]))

    # No obstacle arrives in the next camera cloud after a 90-degree yaw, but
    # the remembered world point still becomes a body-relative VFH sample.
    now[0] = 1.0
    memory.update(np.empty((0, 3)))
    samples, nearest, bearing, kept = cloud_to_samples(
        memory.snapshot(), (0.0, 0.0, 0.0), math.radians(90.0),
        min_range=0.25, max_range=2.0, z_below=0.15, z_above=0.60,
    )

    assert kept == 1
    assert len(samples) == 1
    assert math.isclose(nearest, 1.0, abs_tol=1e-9)
    assert math.isclose(bearing, math.radians(90.0), abs_tol=1e-9)


def test_obstacle_memory_voxel_replaces_instead_of_accumulating() -> None:
    now = [0.0]
    memory = WorldObstacleMemory(
        duration=30.0, voxel_size=0.10, max_points=100, clock=lambda: now[0]
    )
    memory.update(np.array([[1.01, 0.01, 0.01]]))
    now[0] = 1.0
    memory.update(np.array([[1.08, 0.08, 0.08]]))

    points = memory.snapshot()
    assert len(points) == 1
    assert np.allclose(points[0], [1.08, 0.08, 0.08])


def test_obstacle_memory_preserves_latest_cloud_density_inside_a_voxel() -> None:
    now = [0.0]
    memory = WorldObstacleMemory(
        duration=30.0, voxel_size=0.10, max_points=100, clock=lambda: now[0]
    )
    first = np.array(
        [[1.01, 0.01, z] for z in (0.01, 0.02, 0.03, 0.04)], dtype=float
    )
    memory.update(first)
    assert len(memory.snapshot()) == 4

    # Seeing the same voxel again replaces its batch. It preserves the newest
    # frame's three returns rather than collapsing to one or accumulating seven.
    now[0] = 1.0
    second = np.array(
        [[1.02, 0.02, z] for z in (0.05, 0.06, 0.07)], dtype=float
    )
    memory.update(second)
    assert len(memory.snapshot()) == 3


def test_obstacle_memory_reuses_snapshot_until_the_map_changes() -> None:
    memory = WorldObstacleMemory(
        duration=30.0, voxel_size=0.10, max_points=100, clock=lambda: 0.0
    )
    memory.update(np.array([[1.0, 0.0, 0.0], [1.2, 0.0, 0.0]]))
    first = memory.snapshot()
    assert memory.snapshot() is first

    memory.update(np.array([[1.4, 0.0, 0.0]]))
    assert memory.snapshot() is not first


def test_obstacle_memory_expires_and_is_bounded() -> None:
    now = [0.0]
    memory = WorldObstacleMemory(
        duration=2.0, voxel_size=0.10, max_points=2, clock=lambda: now[0]
    )
    memory.update(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]))
    assert len(memory.snapshot()) == 2

    now[0] = 2.01
    assert len(memory.snapshot()) == 0


def test_zero_duration_disables_obstacle_memory() -> None:
    memory = WorldObstacleMemory(duration=0.0, voxel_size=0.10, max_points=10)
    memory.update(np.array([[1.0, 0.0, 0.0]]))
    assert not memory.enabled
    assert len(memory.snapshot()) == 0


def test_remembered_points_do_not_defeat_the_cloud_staleness_watchdog() -> None:
    now = [0.0]

    class FakeNode:
        class Logger:
            def warn(self, *args, **kwargs):
                pass

        def create_subscription(self, *args, **kwargs):
            return None

        def monotonic_time(self):
            return now[0]

        def get_logger(self):
            return self.Logger()

    cloud = PointCloud2()
    cloud.height = 1
    cloud.width = 1
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = 12
    cloud.data = struct.pack("<fff", 1.0, 0.0, 0.0)

    field = ObstacleField(
        FakeNode(), memory_duration=30.0, memory_voxel_size=0.10,
        memory_max_points=100,
    )
    field.origin = (0.0, 0.0, 0.0)
    field.yaw_enu = 0.0
    field.pose_time = now[0]
    field.on_cloud(cloud)
    assert field.stale_reason(1.0) is None
    assert field.snapshot().memory_point_count == 1

    now[0] = 2.0
    assert "obstacle cloud stale" in field.stale_reason(1.0)
    # The point remains available for visualization/planning state, but the
    # node's existing stale-data path will hold instead of using it to fly.
    assert field.snapshot().memory_point_count == 1


def test_small_map_correction_jitter_does_not_rebuild_obstacle_memory() -> None:
    now = [0.0]

    class FakeNode:
        class Logger:
            def warn(self, *args, **kwargs):
                pass

        def create_subscription(self, *args, **kwargs):
            return None

        def monotonic_time(self):
            return now[0]

        def get_logger(self):
            return self.Logger()

    field = ObstacleField(
        FakeNode(), memory_duration=30.0,
        memory_reset_correction_m=0.05,
        memory_reset_correction_deg=2.0,
    )
    field.memory.update(np.array([[1.0, 0.0, 0.0]]))
    field.cloud_time = now[0]

    initial = PoseStamped()
    initial.pose.orientation.w = 1.0
    field.on_memory_correction(initial)

    jitter = PoseStamped()
    jitter.pose.position.x = 0.01
    jitter.pose.orientation.z = math.sin(math.radians(0.5) / 2.0)
    jitter.pose.orientation.w = math.cos(math.radians(0.5) / 2.0)
    field.on_memory_correction(jitter)
    assert len(field.memory.snapshot()) == 1
    assert field.cloud_time == 0.0

    shifted = PoseStamped()
    shifted.pose.position.x = 0.06
    shifted.pose.orientation.w = 1.0
    field.on_memory_correction(shifted)
    assert len(field.memory.snapshot()) == 0
    assert field.cloud_time is None
