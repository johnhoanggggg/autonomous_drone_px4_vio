import math
from typing import Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleAttitude, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # w, x, y, z
Matrix3 = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]


ENU_TO_NED: Matrix3 = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)

FRD_TO_FLU: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


def matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def transform_vector(m: Matrix3, v: Vector3) -> Vector3:
    return tuple(sum(m[row][col] * v[col] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def normalize_quaternion(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in q))
    if norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(component / norm for component in q)  # type: ignore[return-value]


def quaternion_to_matrix(q: Quaternion) -> Matrix3:
    w, x, y, z = normalize_quaternion(q)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def matrix_to_quaternion(m: Matrix3) -> Quaternion:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return normalize_quaternion(
            (
                0.25 * s,
                (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s,
            )
        )

    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return normalize_quaternion(
            (
                (m[2][1] - m[1][2]) / s,
                0.25 * s,
                (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s,
            )
        )

    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return normalize_quaternion(
            (
                (m[0][2] - m[2][0]) / s,
                (m[0][1] + m[1][0]) / s,
                0.25 * s,
                (m[1][2] + m[2][1]) / s,
            )
        )

    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return normalize_quaternion(
        (
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
        )
    )


def enu_flu_pose_to_ned_frd(position: Vector3, orientation: Quaternion) -> Tuple[Vector3, Quaternion]:
    ned_position = transform_vector(ENU_TO_NED, position)
    enu_from_flu = quaternion_to_matrix(orientation)
    ned_from_frd = matmul(matmul(ENU_TO_NED, enu_from_flu), FRD_TO_FLU)
    return ned_position, matrix_to_quaternion(ned_from_frd)


def yaw_matrix(yaw_rad: float) -> Matrix3:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return (
        (c, -s, 0.0),
        (s, c, 0.0),
        (0.0, 0.0, 1.0),
    )


def apply_enu_yaw_offset(
    position: Vector3, orientation: Quaternion, yaw_offset_deg: float
) -> Tuple[Vector3, Quaternion]:
    if abs(yaw_offset_deg) < 1.0e-9:
        return position, orientation

    offset = yaw_matrix(math.radians(yaw_offset_deg))
    rotated_position = transform_vector(offset, position)
    rotated_orientation = matrix_to_quaternion(matmul(offset, quaternion_to_matrix(orientation)))
    return rotated_position, rotated_orientation


def transpose(m: Matrix3) -> Matrix3:
    return tuple(tuple(m[col][row] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def gravity_down_in_odom_enu(
    orientation_enu_flu: Quaternion, attitude_ned_frd: Quaternion
) -> Vector3:
    """True gravity-down direction expressed in the VIO odometry frame.

    PX4's attitude (EKF2_EV_CTRL does not fuse EV attitude) knows where down is
    in the body; the VIO orientation carries that body direction into the odom
    frame.  A level odom frame gives (0, 0, -1).  The camera is taken to be
    square to the body, the same assumption camera_to_body_pose makes.
    """
    down_frd = transform_vector(transpose(quaternion_to_matrix(attitude_ned_frd)), (0.0, 0.0, 1.0))
    down_flu = transform_vector(FRD_TO_FLU, down_frd)
    return transform_vector(quaternion_to_matrix(orientation_enu_flu), down_flu)


def leveling_matrix(down_enu: Vector3) -> Matrix3:
    """Smallest rotation taking `down_enu` onto (0, 0, -1); it never adds yaw."""
    norm = math.sqrt(sum(value * value for value in down_enu))
    if norm <= 0.0:
        return yaw_matrix(0.0)
    dx, dy, dz = (value / norm for value in down_enu)
    # axis = d x (0, 0, -1) = (-dy, dx, 0); cos(angle) = -dz (Rodrigues).
    ax, ay = -dy, dx
    s = math.hypot(ax, ay)
    if s <= 1.0e-12:
        return yaw_matrix(0.0)
    ax, ay = ax / s, ay / s
    c = -dz
    v = 1.0 - c
    return (
        (c + ax * ax * v, ax * ay * v, ay * s),
        (ax * ay * v, c + ay * ay * v, -ax * s),
        (-ay * s, ax * s, c),
    )


class EvLeveler:
    """Remove the VIO odometry frame's tilt from the pose PX4 flies on.

    Without IMU gravity, RTAB-Map odometry starts square to the camera and
    wanders 2-11 deg off gravity (09-30 flights).  PX4 holds EV height, so it
    climbed distance x tan(tilt): 1.2 m over a 10 m route.  Each new VIO step is
    rotated by the current (low-passed) leveling and accumulated, so a changing
    tilt estimate never moves positions already flown, and the output starts
    equal to the raw VIO pose so enabling it causes no jump.
    """

    def __init__(self, time_constant_s: float = 5.0, max_tilt_deg: float = 25.0) -> None:
        self.time_constant_s = time_constant_s
        self.max_tilt_cos = math.cos(math.radians(max_tilt_deg))
        self.reset()

    def reset(self) -> None:
        self.down: Optional[Vector3] = None
        self.down_time_s: Optional[float] = None
        self.last_raw: Optional[Vector3] = None
        self.leveled: Optional[Vector3] = None

    def update_down(self, down_enu: Vector3, now_s: float) -> bool:
        """Low-pass a gravity sample into the estimate; False if it is rejected."""
        norm = math.sqrt(sum(value * value for value in down_enu))
        if norm <= 0.0 or not all(math.isfinite(value) for value in down_enu):
            return False
        sample = tuple(value / norm for value in down_enu)
        if -sample[2] < self.max_tilt_cos:
            return False
        if self.down is None or self.down_time_s is None:
            self.down = sample  # type: ignore[assignment]
        else:
            dt = max(0.0, now_s - self.down_time_s)
            alpha = min(1.0, dt / self.time_constant_s) if self.time_constant_s > 0.0 else 1.0
            blended = tuple(d + (x - d) * alpha for d, x in zip(self.down, sample))
            n = math.sqrt(sum(value * value for value in blended))
            self.down = tuple(value / n for value in blended)  # type: ignore[assignment]
        self.down_time_s = now_s
        return True

    def tilt_deg(self) -> float:
        if self.down is None:
            return math.nan
        return math.degrees(math.acos(max(-1.0, min(1.0, -self.down[2]))))

    def apply(self, position: Vector3, orientation: Quaternion) -> Tuple[Vector3, Quaternion]:
        level = leveling_matrix(self.down) if self.down is not None else yaw_matrix(0.0)
        if self.leveled is None or self.last_raw is None:
            self.leveled = position
        else:
            step = transform_vector(level, tuple(p - q for p, q in zip(position, self.last_raw)))  # type: ignore[arg-type]
            self.leveled = tuple(p + d for p, d in zip(self.leveled, step))  # type: ignore[assignment]
        self.last_raw = position
        return self.leveled, matrix_to_quaternion(matmul(level, quaternion_to_matrix(orientation)))


def covariance_triplet(value: float) -> Iterable[float]:
    return (float(value), float(value), float(value))


def pose_rejection_reason(
    position: Vector3,
    orientation: Quaternion,
    reset_position_epsilon: float = 1.0e-6,
    reset_orientation_epsilon: float = 1.0e-3,
) -> Optional[str]:
    """Return why a source pose is unsafe to forward, or None if it is usable.

    RTABMap's DepthAI VIO reports a distinctive all-zero-position/all-0.5
    quaternion while its transform is reset or unavailable.  The quaternion is
    normalized, so generic quaternion validation alone cannot catch it.
    """
    values = (*position, *orientation)
    if not all(math.isfinite(value) for value in values):
        return "pose contains a non-finite value"

    quaternion_norm = math.sqrt(sum(component * component for component in orientation))
    if quaternion_norm <= 1.0e-6:
        return "quaternion has zero norm"

    at_origin = all(abs(value) <= reset_position_epsilon for value in position)
    # q and -q encode the same rotation, so recognize either sign.
    reset_quaternion = min(
        max(abs(value - 0.5) for value in orientation),
        max(abs(value + 0.5) for value in orientation),
    ) <= reset_orientation_epsilon
    if at_origin and reset_quaternion:
        return "RTABMap reset sentinel"

    return None


def raw_odometry_rejection_reason(
    message, now_ns, previous_stamp_ns=None, max_age_ns=500_000_000
):
    """ROS backend only: accept continuous oak-body odometry, never map poses."""
    if message.header.frame_id != 'rtabmap_odom' or message.child_frame_id != 'oak':
        return 'unexpected raw odometry frames'
    stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    if stamp <= 0 or stamp > now_ns or now_ns - stamp > max_age_ns:
        return (
            'raw odometry timestamp is invalid, future or over '
            f'{max_age_ns / 1_000_000:.0f} ms old'
        )
    if previous_stamp_ns is not None and stamp <= previous_stamp_ns:
        return 'non-increasing raw odometry timestamp'
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return pose_rejection_reason((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))


class VioToPx4Odometry(Node):
    def __init__(self) -> None:
        super().__init__("vio_to_px4_odometry")

        self.declare_parameter("input_pose_topic", "/basalt/pose")
        # Empty preserves the established DepthAI PoseStamped path exactly.
        self.declare_parameter("input_odometry_topic", "")
        self.declare_parameter("output_odometry_topic", "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("frame_transform", "enu_flu_to_ned_frd")
        self.declare_parameter("vio_yaw_offset_deg", 0.0)
        self.declare_parameter("debug_pose_topic", "/vio/yaw_offset/pose")
        self.declare_parameter("debug_odometry_topic", "/vio/yaw_offset/odometry")
        self.declare_parameter("debug_path_topic", "/vio/yaw_offset/path")
        self.declare_parameter("debug_path_size", 500)
        self.declare_parameter("debug_path_publish_stride", 5)
        self.declare_parameter("position_variance", 0.02)
        self.declare_parameter("orientation_variance", 0.01)
        self.declare_parameter("velocity_variance", 0.05)
        self.declare_parameter("quality", 100)
        self.declare_parameter("reject_rtabmap_reset_pose", True)
        self.declare_parameter("reset_position_epsilon", 1.0e-6)
        self.declare_parameter("reset_orientation_epsilon", 1.0e-3)
        self.declare_parameter("max_raw_odometry_age", 0.5)
        # Rotate VIO steps onto PX4's gravity before PX4 sees them (EvLeveler).
        self.declare_parameter("level_with_px4_attitude", False)
        self.declare_parameter("attitude_topic", "/fmu/out/vehicle_attitude")
        self.declare_parameter("level_time_constant", 5.0)
        self.declare_parameter("level_max_tilt_deg", 25.0)
        self.declare_parameter("level_attitude_max_age", 0.3)
        self.declare_parameter("level_tilt_topic", "/vio/level/tilt_deg")

        input_topic = self.get_parameter("input_pose_topic").value
        odometry_topic = self.get_parameter("input_odometry_topic").value
        self.raw_odometry_input = bool(odometry_topic)
        self.last_raw_stamp_ns = None
        max_raw_age = float(self.get_parameter("max_raw_odometry_age").value)
        if not math.isfinite(max_raw_age) or max_raw_age <= 0.0:
            raise ValueError("max_raw_odometry_age must be finite and positive")
        self.max_raw_odometry_age_ns = int(max_raw_age * 1_000_000_000)
        output_topic = self.get_parameter("output_odometry_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(VehicleOdometry, output_topic, qos)
        self.subscription = (
            self.create_subscription(Odometry, odometry_topic, self.odometry_callback, qos)
            if self.raw_odometry_input else
            self.create_subscription(PoseStamped, input_topic, self.pose_callback, qos))
        self.debug_pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("debug_pose_topic").value), 10
        )
        self.debug_odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("debug_odometry_topic").value), 10
        )
        self.debug_path_pub = self.create_publisher(
            Path, str(self.get_parameter("debug_path_topic").value), 10
        )
        self.previous_position: Optional[Vector3] = None
        self.previous_time_us: Optional[int] = None
        self.debug_path = Path()
        self.debug_path.header.frame_id = "world"
        self.debug_path_size = max(1, int(self.get_parameter("debug_path_size").value))
        self.debug_path_publish_stride = max(1, int(self.get_parameter("debug_path_publish_stride").value))
        self.sent_count = 0
        self.rejected_count = 0
        self.reset_counter = 0
        self.reset_pose_active = False

        self.leveler: Optional[EvLeveler] = None
        self.attitude: Optional[Quaternion] = None
        self.attitude_time_ns: Optional[int] = None
        if bool(self.get_parameter("level_with_px4_attitude").value):
            if self.get_parameter("frame_transform").value != "enu_flu_to_ned_frd":
                raise ValueError("level_with_px4_attitude needs frame_transform enu_flu_to_ned_frd")
            self.leveler = EvLeveler(
                float(self.get_parameter("level_time_constant").value),
                float(self.get_parameter("level_max_tilt_deg").value))
            self.attitude_max_age_ns = int(
                float(self.get_parameter("level_attitude_max_age").value) * 1_000_000_000)
            self.attitude_sub = self.create_subscription(
                VehicleAttitude, str(self.get_parameter("attitude_topic").value),
                self.attitude_callback, qos)
            self.tilt_pub = self.create_publisher(
                Float32, str(self.get_parameter("level_tilt_topic").value), 10)

        self.get_logger().info(
            f"Bridging {odometry_topic or input_topic} -> {output_topic}"
            + (" (leveled with PX4 attitude)" if self.leveler else ""))

    def attitude_callback(self, attitude: VehicleAttitude) -> None:
        self.attitude = tuple(float(value) for value in attitude.q)  # type: ignore[assignment]
        self.attitude_time_ns = self.get_clock().now().nanoseconds

    def odometry_callback(self, odometry: Odometry) -> None:
        reason = raw_odometry_rejection_reason(
            odometry, self.get_clock().now().nanoseconds, self.last_raw_stamp_ns,
            self.max_raw_odometry_age_ns)
        if reason is not None:
            self.previous_position = None
            self.previous_time_us = None
            self.rejected_count += 1
            if self.rejected_count == 1 or self.rejected_count % 100 == 0:
                self.get_logger().warning(f"Rejected ROS VIO: {reason}")
            return
        self.last_raw_stamp_ns = (odometry.header.stamp.sec * 1_000_000_000 +
                                  odometry.header.stamp.nanosec)
        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.pose_callback(pose)

    def pose_callback(self, pose: PoseStamped) -> None:
        position_enu: Vector3 = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        orientation_enu_flu: Quaternion = (
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
        )
        rejection_reason = pose_rejection_reason(
            position_enu,
            orientation_enu_flu,
            float(self.get_parameter("reset_position_epsilon").value),
            float(self.get_parameter("reset_orientation_epsilon").value),
        )
        if (
            rejection_reason == "RTABMap reset sentinel"
            and not bool(self.get_parameter("reject_rtabmap_reset_pose").value)
        ):
            rejection_reason = None

        if rejection_reason is not None:
            self.rejected_count += 1
            self.previous_position = None
            self.previous_time_us = None
            if rejection_reason == "RTABMap reset sentinel" and not self.reset_pose_active:
                self.reset_counter = (self.reset_counter + 1) % 256
                self.reset_pose_active = True
                if self.leveler is not None:
                    # A restarted VIO has a new frame and a new tilt.
                    self.leveler.reset()
            if self.rejected_count == 1 or self.rejected_count % 100 == 0:
                self.get_logger().warning(
                    f"Rejected VIO pose #{self.rejected_count}: {rejection_reason}"
                )
            return

        self.reset_pose_active = False
        position_enu, orientation_enu_flu = apply_enu_yaw_offset(
            position_enu,
            orientation_enu_flu,
            float(self.get_parameter("vio_yaw_offset_deg").value),
        )
        if self.leveler is not None:
            position_enu, orientation_enu_flu = self.level(position_enu, orientation_enu_flu)
        self.publish_debug_pose(pose, position_enu, orientation_enu_flu)

        transform_mode = self.get_parameter("frame_transform").value
        if transform_mode == "enu_flu_to_ned_frd":
            position, orientation = enu_flu_pose_to_ned_frd(position_enu, orientation_enu_flu)
            pose_frame = VehicleOdometry.POSE_FRAME_NED
            velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        elif transform_mode == "passthrough_ned_frd":
            position = position_enu
            orientation = normalize_quaternion(orientation_enu_flu)
            pose_frame = VehicleOdometry.POSE_FRAME_NED
            velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        else:
            self.get_logger().error(f"Unsupported frame_transform: {transform_mode}")
            return

        now_us = self.get_clock().now().nanoseconds // 1000
        sample_us = (self.last_raw_stamp_ns // 1000) if self.raw_odometry_input else now_us
        velocity = self.estimate_velocity(position, sample_us)

        msg = VehicleOdometry()
        msg.timestamp = int(now_us)
        msg.timestamp_sample = int(sample_us)
        msg.pose_frame = pose_frame
        msg.position = [float(value) for value in position]
        msg.q = [float(value) for value in orientation]
        msg.velocity_frame = velocity_frame
        msg.velocity = [float(value) for value in velocity]
        msg.angular_velocity = [math.nan, math.nan, math.nan]
        msg.position_variance = list(covariance_triplet(self.get_parameter("position_variance").value))
        msg.orientation_variance = list(covariance_triplet(self.get_parameter("orientation_variance").value))
        msg.velocity_variance = list(covariance_triplet(self.get_parameter("velocity_variance").value))
        msg.reset_counter = self.reset_counter
        msg.quality = int(self.get_parameter("quality").value)

        self.publisher.publish(msg)
        self.sent_count += 1
        if self.sent_count == 1 or self.sent_count % 100 == 0:
            self.get_logger().info(
                "Published visual odometry "
                f"#{self.sent_count}: p_ned=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
            )

    def level(self, position_enu: Vector3, orientation_enu_flu: Quaternion) -> Tuple[Vector3, Quaternion]:
        now_ns = self.get_clock().now().nanoseconds
        # A stale or missing attitude freezes the current estimate; it never
        # stops EV from flowing.
        if (self.attitude is not None and self.attitude_time_ns is not None
                and now_ns - self.attitude_time_ns <= self.attitude_max_age_ns):
            down = gravity_down_in_odom_enu(orientation_enu_flu, self.attitude)
            self.leveler.update_down(down, now_ns * 1.0e-9)
        leveled = self.leveler.apply(position_enu, orientation_enu_flu)
        tilt = self.leveler.tilt_deg()
        self.tilt_pub.publish(Float32(data=float(tilt)))
        if self.sent_count % 100 == 0:
            self.get_logger().info(f"VIO frame tilt {tilt:.1f} deg from PX4 gravity")
        return leveled

    def publish_debug_pose(
        self, source_pose: PoseStamped, position_enu: Vector3, orientation_enu_flu: Quaternion
    ) -> None:
        pose = PoseStamped()
        pose.header = source_pose.header
        pose.header.frame_id = source_pose.header.frame_id or "world"
        pose.pose.position.x = position_enu[0]
        pose.pose.position.y = position_enu[1]
        pose.pose.position.z = position_enu[2]
        pose.pose.orientation.w = orientation_enu_flu[0]
        pose.pose.orientation.x = orientation_enu_flu[1]
        pose.pose.orientation.y = orientation_enu_flu[2]
        pose.pose.orientation.z = orientation_enu_flu[3]

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = "vio_yaw_offset_body"
        odom.pose.pose = pose.pose

        self.debug_path.header = pose.header
        self.debug_path.poses.append(pose)
        if len(self.debug_path.poses) > self.debug_path_size:
            self.debug_path.poses = self.debug_path.poses[-self.debug_path_size :]

        self.debug_pose_pub.publish(pose)
        self.debug_odom_pub.publish(odom)
        if self.sent_count % self.debug_path_publish_stride == 0:
            self.debug_path_pub.publish(self.debug_path)

    def estimate_velocity(self, position: Vector3, now_us: int) -> Vector3:
        if self.previous_position is None or self.previous_time_us is None:
            self.previous_position = position
            self.previous_time_us = now_us
            return (math.nan, math.nan, math.nan)

        dt = (now_us - self.previous_time_us) / 1_000_000.0
        previous = self.previous_position
        self.previous_position = position
        self.previous_time_us = now_us

        if dt <= 0.0 or dt > 1.0:
            return (math.nan, math.nan, math.nan)

        return (
            (position[0] - previous[0]) / dt,
            (position[1] - previous[1]) / dt,
            (position[2] - previous[2]) / dt,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VioToPx4Odometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
