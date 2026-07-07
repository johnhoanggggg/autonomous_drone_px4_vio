import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class Px4LocalPositionToRos(Node):
    def __init__(self):
        super().__init__("px4_local_position_to_ros")

        self.declare_parameter("input_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("output_pose_topic", "/px4/local_position/pose")
        self.declare_parameter("output_odometry_topic", "/px4/local_position/odometry")
        self.declare_parameter("output_path_topic", "/px4/local_position/path")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", "px4_base_link")
        self.declare_parameter("path_size", 300)
        self.declare_parameter("path_publish_stride", 10)

        input_topic = str(self.get_parameter("input_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.path_size = max(1, int(self.get_parameter("path_size").value))
        self.path_publish_stride = max(1, int(self.get_parameter("path_publish_stride").value))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("output_pose_topic").value), output_qos
        )
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_odometry_topic").value), output_qos
        )
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("output_path_topic").value), output_qos
        )
        self.subscription = self.create_subscription(
            VehicleLocalPosition, input_topic, self.local_position_callback, qos
        )

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.published_count = 0

        self.get_logger().info(f"Converting {input_topic} to /px4/local_position/*")

    def local_position_callback(self, msg):
        if not (msg.xy_valid and msg.z_valid):
            return

        stamp = self.get_clock().now().to_msg()

        # PX4 local position is NED. Foxglove standard ROS views expect ENU.
        x_enu = float(msg.y)
        y_enu = float(msg.x)
        z_enu = float(-msg.z)
        yaw_enu = math.pi * 0.5 - float(msg.heading)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_enu)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x_enu
        pose.pose.position.y = y_enu
        pose.pose.position.z = z_enu
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = pose.pose
        if msg.v_xy_valid:
            odom.twist.twist.linear.x = float(msg.vy)
            odom.twist.twist.linear.y = float(msg.vx)
        if msg.v_z_valid:
            odom.twist.twist.linear.z = float(-msg.vz)

        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.path_size:
            self.path.poses = self.path.poses[-self.path_size :]

        self.pose_pub.publish(pose)
        self.odom_pub.publish(odom)

        self.published_count += 1
        if self.published_count % self.path_publish_stride == 0:
            self.path_pub.publish(self.path)

        if self.published_count == 1 or self.published_count % 100 == 0:
            self.get_logger().info(
                "Published PX4 local position "
                f"#{self.published_count}: enu=({x_enu:.2f}, {y_enu:.2f}, {z_enu:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Px4LocalPositionToRos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
