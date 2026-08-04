#!/usr/bin/env python3
"""Publish a synthetic obstacle cloud + SLAM pose so VFH can be tested indoors, cold.

No camera, no drone, no PX4. This fakes exactly the two topics the VFH nodes
consume — `/rtabmap/obstacle_cloud` (PointCloud2, ENU `world`) and
`/rtabmap/pose` (PoseStamped, same frame) — so `vfh_monitor` can be run against
a known scene and its answers checked against arithmetic you can do by hand.

    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42

    # terminal 1: a wall 2.2 m ahead with a 1.4 m gap 0.8 m off to the right
    python3 scripts/vfh_sim_obstacles.py --wall-distance 2.2 --gap-width 1.4 --gap-offset -0.8

    # terminal 2
    ros2 run px4_vio_bridge vfh_monitor

The monitor should report a steer angle pointing at the gap, and the ASCII bar
should show `#` on both sides of a run of `.`. Note that ENU +y is the
vehicle's LEFT when it faces +x, so a gap at y=-0.8 is a positive (right)
steer angle. Narrow the gap below `2 * (robot_radius + safety_margin)` = 1.0 m
and the monitor must refuse it and report BLOCKED — that check is worth doing
once, because it is the difference between avoidance and optimism.

Scene options:

    --scene wall      a wall across the front, optional gap (default)
    --scene pillar    a single 0.3 m pillar dead ahead
    --scene corridor  walls to the left and right, open ahead
    --scene box       walls on three sides: the planner must report BLOCKED

`--approach 0.15` walks the pose forward at 0.15 m/s so the obstacle closes in
and you can watch the stop/abort thresholds fire in the flight node.
"""
import argparse
import math
import struct

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def wall_points(center_x, center_y, half_width, axis, spacing, heights):
    """A vertical planar patch of points, in ENU."""
    points = []
    n = max(2, int(2.0 * half_width / spacing) + 1)
    for i in range(n):
        offset = -half_width + 2.0 * half_width * i / (n - 1)
        x = center_x + (offset if axis == "x" else 0.0)
        y = center_y + (offset if axis == "y" else 0.0)
        for z in heights:
            points.append((x, y, z))
    return points


def build_scene(args, z_levels):
    """Scene points in ENU, with the vehicle at the origin facing +x (east)."""
    if args.scene == "pillar":
        return wall_points(
            args.wall_distance, 0.0, 0.15, "y", args.spacing, z_levels
        )

    if args.scene == "corridor":
        return (
            wall_points(args.wall_distance, 0.9, 2.0, "x", args.spacing, z_levels)
            + wall_points(args.wall_distance, -0.9, 2.0, "x", args.spacing, z_levels)
        )

    if args.scene == "box":
        return (
            wall_points(args.wall_distance, 0.0, 1.5, "y", args.spacing, z_levels)
            + wall_points(0.5, 1.2, 1.5, "x", args.spacing, z_levels)
            + wall_points(0.5, -1.2, 1.5, "x", args.spacing, z_levels)
        )

    # Default: a wall across the front, with an optional gap punched in it.
    points = wall_points(
        args.wall_distance, 0.0, args.wall_half_width, "y", args.spacing, z_levels
    )
    if args.gap_width > 0.0:
        low = args.gap_offset - args.gap_width / 2.0
        high = args.gap_offset + args.gap_width / 2.0
        points = [p for p in points if not (low <= p[1] <= high)]
    return points


def make_cloud(stamp, frame_id, points):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgba", offset=12, datatype=PointField.UINT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.is_dense = True
    msg.data = b"".join(
        struct.pack("<fffI", float(x), float(y), float(z), 0xFFFF0000)
        for x, y, z in points
    )
    return msg


class ObstacleSimulator(Node):
    def __init__(self, args):
        super().__init__("vfh_sim_obstacles")
        self.args = args
        self.cloud_pub = self.create_publisher(PointCloud2, args.cloud_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, args.pose_topic, 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = math.radians(args.yaw_deg)
        z_levels = [args.height + 0.1 * k for k in range(-1, 2)]
        self.points = build_scene(args, z_levels)
        self.timer = self.create_timer(1.0 / args.rate_hz, self.tick)
        self.get_logger().warn(
            f"simulating scene '{args.scene}' with {len(self.points)} points at "
            f"{args.rate_hz:.0f} Hz on {args.cloud_topic}; pose on {args.pose_topic} "
            f"(approach {args.approach:.2f} m/s)"
        )

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        step = self.args.approach / self.args.rate_hz
        self.x += step * math.cos(self.yaw)
        self.y += step * math.sin(self.yaw)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.args.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.args.height
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(pose)
        self.cloud_pub.publish(make_cloud(stamp, self.args.frame_id, self.points))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene", choices=("wall", "pillar", "corridor", "box"), default="wall"
    )
    parser.add_argument("--wall-distance", type=float, default=1.8)
    parser.add_argument("--wall-half-width", type=float, default=1.5)
    parser.add_argument("--gap-width", type=float, default=0.0)
    parser.add_argument("--gap-offset", type=float, default=0.0)
    parser.add_argument("--spacing", type=float, default=0.02)
    parser.add_argument("--height", type=float, default=0.30)
    parser.add_argument("--yaw-deg", type=float, default=0.0,
                        help="vehicle ENU yaw; 0 faces +x (east)")
    parser.add_argument("--approach", type=float, default=0.0,
                        help="m/s the fake vehicle walks forward")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--cloud-topic", default="/rtabmap/obstacle_cloud")
    parser.add_argument("--pose-topic", default="/rtabmap/pose")
    parser.add_argument("--frame-id", default="world")
    args = parser.parse_args()

    rclpy.init()
    node = ObstacleSimulator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
