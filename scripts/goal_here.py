#!/usr/bin/env python3
"""Publish a planner3d goal at the drone's current world position.

Foxglove's publish-point tool always lands on z=0, so it cannot place a goal
above the floor. This reads the planner's own start pose from
/planner3d/status (frame 'world'), adds an optional offset, and publishes the
result on /waypoint/clicked. Use goal_height_mode:=clicked so the z is kept.

Usage: python3 goal_here.py [dx dy dz]
"""
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


def main():
    offset = [float(v) for v in sys.argv[1:4]] if len(sys.argv) >= 4 else [0.0, 0.0, 0.0]

    rclpy.init()
    node = Node('goal_here')
    status = {}
    node.create_subscription(
        String, '/planner3d/status', lambda m: status.update(json.loads(m.data)), 10)
    pub = node.create_publisher(PointStamped, '/waypoint/clicked', 10)

    deadline = time.time() + 5.0
    while time.time() < deadline and 'start' not in status:
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'start' not in status:
        sys.exit(f"no start pose in /planner3d/status (state={status.get('state')})")

    x, y, z = (s + o for s, o in zip(status['start'], offset))
    msg = PointStamped()
    msg.header.frame_id = 'world'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.point.x, msg.point.y, msg.point.z = x, y, z

    # Wait for the planner's subscription so the one-shot message is not dropped.
    deadline = time.time() + 3.0
    while time.time() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.2)
    print(f"planner state was {status.get('state')}; "
          f"goal sent: x={x:.3f} y={y:.3f} z={z:.3f} (world)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
