#!/usr/bin/env python3
"""Live check of how level the RTAB-Map map and the VIO odometry frames are.

Gravity comes from the OAK accelerometer (mean of the last 0.5 s; valid while
the camera is not accelerating, e.g. on the bench or hand-held slowly), read
from /rtabmap/imu_frame_fixed: the raw /oak/imu/data is 180 deg off the
driver's oak_imu_frame TF (see oak_imu_frame_fix_component.cpp). It is
rotated into `world` (map) and `rtabmap_odom` through TF, and the angle from
each frame's +z is the tilt. Also counts gravity links in /rtabmap/mapData.

Usage: python3 scripts/map_level_check.py --seconds 60 [--out report.json]
Publishes nothing.
"""
import argparse
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from rtabmap_msgs.msg import MapData
from tf2_ros import Buffer, TransformListener

GRAVITY_LINK = 9  # rtabmap::Link::kGravity


def rotate(q, v):
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


class LevelCheck(Node):
    def __init__(self, imu_topic):
        super().__init__('map_level_check')
        self.acc = deque(maxlen=400)
        self.frame = None
        self.gravity_links = 0
        self.nodes = 0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(MapData, '/rtabmap/mapData', self.on_map, 10)

    def on_imu(self, msg):
        self.frame = msg.header.frame_id
        a = msg.linear_acceleration
        w = msg.angular_velocity
        self.acc.append((time.monotonic(), a.x, a.y, a.z, math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)))

    def on_map(self, msg):
        self.nodes = max(self.nodes, len(msg.graph.poses))
        # The graph in each message is complete, so keep the largest count.
        self.gravity_links = max(self.gravity_links, sum(1 for link in msg.graph.links if link.type == GRAVITY_LINK))

    def quiet(self):
        """True when the last 0.5 s had near-1 g and little rotation, so the
        accelerometer is a trustworthy gravity reference."""
        now = time.monotonic()
        recent = [a for a in self.acc if now - a[0] <= 0.5]
        if not recent:
            return False
        mean = [sum(a[i] for a in recent) / len(recent) for i in (1, 2, 3)]
        return (abs(math.sqrt(sum(c * c for c in mean)) - 9.81) < 0.3 and
                max(a[4] for a in recent) < 0.3)

    def tilt(self, target):
        now = time.monotonic()
        recent = [a[1:4] for a in self.acc if now - a[0] <= 0.5]
        if not recent or not self.frame:
            return None
        mean = [sum(c) / len(recent) for c in zip(*recent)]
        norm = math.sqrt(sum(c * c for c in mean))
        try:
            t = self.tf.lookup_transform(target, self.frame, rclpy.time.Time())
        except Exception:
            return None
        r = t.transform.rotation
        up = rotate((r.x, r.y, r.z, r.w), [c / norm for c in mean])
        return math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--out')
    parser.add_argument('--imu-topic', default='/rtabmap/imu_frame_fixed')
    args = parser.parse_args()
    rclpy.init()
    node = LevelCheck(args.imu_topic)
    samples = []
    end = time.monotonic() + args.seconds
    next_sample = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() >= next_sample:
            next_sample += 1.0
            row = {'t': round(args.seconds - (end - time.monotonic()), 1),
                   'map_tilt_deg': node.tilt('world'), 'odom_tilt_deg': node.tilt('rtabmap_odom'),
                   'graph_nodes': node.nodes, 'quiet': node.quiet()}
            samples.append(row)
            print(json.dumps(row), flush=True)
    valid = [s['map_tilt_deg'] for s in samples if s['map_tilt_deg'] is not None and s['quiet']]
    report = {'samples': samples, 'map_tilt_max_deg': max(valid) if valid else None,
              'map_tilt_last_deg': valid[-1] if valid else None,
              'gravity_links_seen': node.gravity_links, 'graph_nodes': node.nodes}
    print(json.dumps({k: v for k, v in report.items() if k != 'samples'}))
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(report, f, indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
