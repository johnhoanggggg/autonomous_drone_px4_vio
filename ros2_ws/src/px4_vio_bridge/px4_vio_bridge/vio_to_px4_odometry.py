import math
from typing import Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import VehicleOdometry
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


def covariance_triplet(value: float) -> Iterable[float]:
    return (float(value), float(value), float(value))


class VioToPx4Odometry(Node):
    def __init__(self) -> None:
        super().__init__("vio_to_px4_odometry")

        self.declare_parameter("input_pose_topic", "/basalt/pose")
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

        input_topic = self.get_parameter("input_pose_topic").value
        output_topic = self.get_parameter("output_odometry_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(VehicleOdometry, output_topic, qos)
        self.subscription = self.create_subscription(PoseStamped, input_topic, self.pose_callback, qos)
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

        self.get_logger().info(f"Bridging {input_topic} -> {output_topic}")

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
        position_enu, orientation_enu_flu = apply_enu_yaw_offset(
            position_enu,
            orientation_enu_flu,
            float(self.get_parameter("vio_yaw_offset_deg").value),
        )
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
        velocity = self.estimate_velocity(position, now_us)

        msg = VehicleOdometry()
        msg.timestamp = int(now_us)
        msg.timestamp_sample = int(now_us)
        msg.pose_frame = pose_frame
        msg.position = [float(value) for value in position]
        msg.q = [float(value) for value in orientation]
        msg.velocity_frame = velocity_frame
        msg.velocity = [float(value) for value in velocity]
        msg.angular_velocity = [math.nan, math.nan, math.nan]
        msg.position_variance = list(covariance_triplet(self.get_parameter("position_variance").value))
        msg.orientation_variance = list(covariance_triplet(self.get_parameter("orientation_variance").value))
        msg.velocity_variance = list(covariance_triplet(self.get_parameter("velocity_variance").value))
        msg.reset_counter = 0
        msg.quality = int(self.get_parameter("quality").value)

        self.publisher.publish(msg)
        self.sent_count += 1
        if self.sent_count == 1 or self.sent_count % 100 == 0:
            self.get_logger().info(
                "Published visual odometry "
                f"#{self.sent_count}: p_ned=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
            )

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
