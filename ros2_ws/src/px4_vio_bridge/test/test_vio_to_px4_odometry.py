import math
from types import SimpleNamespace

from nav_msgs.msg import Odometry
from px4_vio_bridge.vio_to_px4_odometry import raw_odometry_rejection_reason, VioToPx4Odometry

from px4_vio_bridge.vio_to_px4_odometry import pose_rejection_reason
from px4_vio_bridge.vio_to_px4_odometry import (
    EvLeveler, gravity_down_in_odom_enu, leveling_matrix, matmul, matrix_to_quaternion,
    quaternion_to_matrix, transform_vector, yaw_matrix)


def raw_message():
    msg = Odometry()
    msg.header.frame_id = 'rtabmap_odom'
    msg.child_frame_id = 'oak'
    msg.header.stamp.sec = 10
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_raw_odometry_requires_continuous_frame_and_body():
    msg = raw_message()
    assert raw_odometry_rejection_reason(msg, 10_100_000_000) is None
    msg.header.frame_id = 'world'
    assert raw_odometry_rejection_reason(msg, 10_100_000_000)
    msg.header.frame_id = 'rtabmap_odom'
    msg.child_frame_id = 'optical'
    assert raw_odometry_rejection_reason(msg, 10_100_000_000)


def test_raw_odometry_rejects_stale_future_reordered_and_null():
    msg = raw_message()
    assert raw_odometry_rejection_reason(msg, 10_500_000_001)
    assert raw_odometry_rejection_reason(msg, 10_300_000_001, max_age_ns=300_000_000)
    assert raw_odometry_rejection_reason(msg, 9_999_999_999)
    assert raw_odometry_rejection_reason(msg, 10_100_000_000, 10_000_000_000)
    msg.pose.pose.orientation.w = 0.0
    assert raw_odometry_rejection_reason(msg, 10_100_000_000)
    msg = raw_message()
    msg.pose.pose.position.x = math.nan
    assert raw_odometry_rejection_reason(msg, 10_100_000_000)


def test_raw_callback_preserves_pose_and_stamp_without_map_transform():
    received = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    fake = SimpleNamespace(last_raw_stamp_ns=None, max_raw_odometry_age_ns=300_000_000,
                           get_clock=lambda: clock,
                           pose_callback=received.append)
    msg = raw_message()
    msg.pose.pose.position.x = 2.0
    VioToPx4Odometry.odometry_callback(fake, msg)
    assert fake.last_raw_stamp_ns == 10_000_000_000
    assert len(received) == 1
    assert received[0].header == msg.header
    assert received[0].pose == msg.pose.pose


def test_raw_callback_rejection_never_forwards_and_clears_velocity():
    received = []
    fake = SimpleNamespace(last_raw_stamp_ns=None, max_raw_odometry_age_ns=300_000_000,
        previous_position=(1,2,3), previous_time_us=1,
        rejected_count=0, get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11_000_000_000)),
        get_logger=lambda: SimpleNamespace(warning=lambda text: None), pose_callback=received.append)
    VioToPx4Odometry.odometry_callback(fake, raw_message())
    assert received == [] and fake.previous_position is None and fake.previous_time_us is None
    assert fake.last_raw_stamp_ns is None


def test_rejects_rtabmap_reset_sentinel() -> None:
    assert pose_rejection_reason(
        (0.0, 0.0, 0.0), (0.5, 0.5, 0.5, 0.5)
    ) == "RTABMap reset sentinel"


def test_raw_input_px4_payload_uses_sensor_time_and_ned_without_ros_graph():
    published = []
    parameters = dict(reset_position_epsilon=1e-6, reset_orientation_epsilon=1e-3,
        vio_yaw_offset_deg=0.0, frame_transform='enu_flu_to_ned_frd',
        position_variance=0.02, orientation_variance=0.01,
        velocity_variance=0.05, quality=100)
    fake = SimpleNamespace(last_raw_stamp_ns=None, max_raw_odometry_age_ns=300_000_000,
        raw_odometry_input=True,
        reset_pose_active=False, reset_counter=0, sent_count=0, leveler=None,
        get_parameter=lambda key: SimpleNamespace(value=parameters[key]),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)),
        get_logger=lambda: SimpleNamespace(info=lambda text: None),
        publish_debug_pose=lambda *args: None,
        publisher=SimpleNamespace(publish=published.append))
    velocity_calls = []
    fake.estimate_velocity = lambda position, stamp: (velocity_calls.append((position, stamp)) or (0., 0., 0.))
    fake.pose_callback = lambda pose: VioToPx4Odometry.pose_callback(fake, pose)
    message = raw_message()
    message.pose.pose.position.x = 1.0
    message.pose.pose.position.y = 2.0
    message.pose.pose.position.z = 3.0
    VioToPx4Odometry.odometry_callback(fake, message)
    assert len(published) == 1
    output = published[0]
    assert output.timestamp == 10_100_000
    assert output.timestamp_sample == 10_000_000
    assert list(output.position) == [2.0, 1.0, -3.0]
    assert output.pose_frame == output.POSE_FRAME_NED
    assert velocity_calls == [((2.0, 1.0, -3.0), 10_000_000)]


def test_rejects_equivalent_negative_reset_quaternion() -> None:
    assert pose_rejection_reason(
        (0.0, 0.0, 0.0), (-0.5, -0.5, -0.5, -0.5)
    ) == "RTABMap reset sentinel"


def test_accepts_legitimate_stationary_origin() -> None:
    assert pose_rejection_reason((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)) is None


def test_rejects_non_finite_pose() -> None:
    assert pose_rejection_reason(
        (math.nan, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    ) == "pose contains a non-finite value"


def test_rejects_zero_norm_quaternion() -> None:
    assert pose_rejection_reason(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 0.0)
    ) == "quaternion has zero norm"


def axis_rotation(axis, angle_deg):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    h = math.radians(angle_deg) / 2.0
    return quaternion_to_matrix((math.cos(h), *(math.sin(h) * v / n for v in (x, y, z))))


def test_leveling_matrix_takes_down_to_vertical_without_adding_yaw():
    down = transform_vector(axis_rotation((1.0, 0.3, 0.0), 8.0), (0.0, 0.0, -1.0))
    level = leveling_matrix(down)
    out = transform_vector(level, down)
    assert max(abs(a - b) for a, b in zip(out, (0.0, 0.0, -1.0))) < 1e-12
    # The tilt axis itself lies in the horizontal plane and must not turn.
    axis = (1.0 / math.hypot(1.0, 0.3), 0.3 / math.hypot(1.0, 0.3), 0.0)
    assert max(abs(a - b) for a, b in zip(transform_vector(level, axis), axis)) < 1e-12
    assert leveling_matrix((0.0, 0.0, -1.0)) == yaw_matrix(0.0)


def test_gravity_down_reports_the_odom_frame_tilt():
    # PX4 level with a 30 deg heading; VIO frame tilted 8 deg about odom x.
    odom_from_world = axis_rotation((1.0, 0.0, 0.0), 8.0)
    world_from_body = yaw_matrix(math.radians(30.0))
    vio_orientation = matrix_to_quaternion(matmul(odom_from_world, world_from_body))
    attitude_ned_frd = (math.cos(math.radians(15.0)), 0.0, 0.0, math.sin(math.radians(15.0)))
    down = gravity_down_in_odom_enu(vio_orientation, attitude_ned_frd)
    expected = transform_vector(odom_from_world, (0.0, 0.0, -1.0))
    assert max(abs(a - b) for a, b in zip(down, expected)) < 1e-9


def test_leveler_holds_true_height_over_a_route_flown_in_a_tilted_frame():
    """09-30 03:51: 8 deg odom tilt made a level 10 m route climb 1.2 m."""
    odom_from_world = axis_rotation((0.2, 1.0, 0.0), 8.0)
    leveler = EvLeveler(time_constant_s=5.0)
    heights = []
    for k in range(401):
        world = (0.02 * k, -0.015 * k, 0.8)  # level flight, 10 m
        raw = transform_vector(odom_from_world, world)
        leveler.update_down(transform_vector(odom_from_world, (0.0, 0.0, -1.0)), 0.1 * k)
        position, _ = leveler.apply(raw, (1.0, 0.0, 0.0, 0.0))
        if k == 0:
            assert position == raw  # enabling it never jumps the EV position
        heights.append(position[2])
    raw_climb = transform_vector(odom_from_world, (8.0, -6.0, 0.8))[2] - heights[0]
    assert abs(raw_climb) > 1.0
    assert max(abs(h - heights[0]) for h in heights) < 1e-9


def test_leveler_estimate_changes_never_move_past_positions():
    leveler = EvLeveler(time_constant_s=0.0)
    leveler.update_down((0.0, 0.0, -1.0), 0.0)
    leveler.apply((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    first, _ = leveler.apply((10.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    leveler.update_down(transform_vector(axis_rotation((0.0, 1.0, 0.0), 10.0), (0.0, 0.0, -1.0)), 1.0)
    same, _ = leveler.apply((10.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert same == first


def test_leveler_rejects_implausible_tilt_and_restarts_from_raw_after_reset():
    leveler = EvLeveler(max_tilt_deg=25.0)
    assert not leveler.update_down(transform_vector(axis_rotation((1.0, 0.0, 0.0), 40.0), (0.0, 0.0, -1.0)), 0.0)
    assert math.isnan(leveler.tilt_deg())
    assert leveler.update_down(transform_vector(axis_rotation((1.0, 0.0, 0.0), 8.0), (0.0, 0.0, -1.0)), 0.0)
    assert abs(leveler.tilt_deg() - 8.0) < 1e-9
    leveler.apply((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    leveler.apply((5.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    leveler.reset()
    restarted, _ = leveler.apply((0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0))
    assert restarted == (0.1, 0.2, 0.3)
