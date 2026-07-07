#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    quat = Quaternion()
    quat.w = math.cos(half_yaw)
    quat.z = math.sin(half_yaw)
    return quat


class BasaltOdometryTest(Node):
    def __init__(self):
        super().__init__("basalt_odometry_test")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "basalt_body")
        self.declare_parameter("path_size", 900)

        self.fps = max(1.0, float(self.get_parameter("fps").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.path_size = max(1, int(self.get_parameter("path_size").value))

        self.pose_pub = self.create_publisher(PoseStamped, "/basalt/pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/basalt/odometry", 10)
        self.path_pub = self.create_publisher(Path, "/basalt/path", 10)

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.fps, self.publish_sample)

        self.get_logger().info(
            f"Publishing /basalt/pose, /basalt/odometry, and /basalt/path at {self.fps:.1f} fps"
        )

    def publish_sample(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        elapsed = (now - self.start_time).nanoseconds * 1.0e-9

        radius = 2.0
        omega = 0.45
        z_base = 1.2
        z_amp = 0.35

        x = radius * math.cos(omega * elapsed)
        y = radius * math.sin(omega * elapsed)
        z = z_base + z_amp * math.sin(omega * elapsed * 0.5)
        yaw = omega * elapsed + math.pi * 0.5

        vx = -radius * omega * math.sin(omega * elapsed)
        vy = radius * omega * math.cos(omega * elapsed)
        vz = z_amp * omega * 0.5 * math.cos(omega * elapsed * 0.5)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = omega

        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.path_size:
            self.path.poses = self.path.poses[-self.path_size :]

        self.pose_pub.publish(pose)
        self.odom_pub.publish(odom)
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = BasaltOdometryTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
