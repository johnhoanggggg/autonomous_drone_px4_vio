#!/usr/bin/env python3
"""Bench/carry measurement of the VIO odometry: tilt from gravity, rate, loss, CPU.

Gravity is the OAK accelerometer, taken only in quiet moments (near 1 g, little
rotation). The raw /oak/imu/data is 180 deg (about IMU x) off the driver's TF, so
with --negate-yz the raw samples are corrected here (use it when the
OakImuFrameFix component is not running); otherwise pass /rtabmap/imu_frame_fixed.

Usage: python3 scripts/vio_bench.py --seconds 60 --label still_off [--negate-yz]
       [--imu-topic /oak/imu/data] [--out report.json]
Publishes nothing.
"""
import argparse
import json
import math
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


def rotate(q, v):
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


def container_cpu_seconds():
    total = 0.0
    tick = os.sysconf('SC_CLK_TCK')
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().split(b'\0')[0]
            if b'component_container' not in cmd:
                continue
            fields = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
            total += (int(fields[11]) + int(fields[12])) / tick
        except (OSError, IndexError, ValueError):
            pass
    return total


class Bench(Node):
    def __init__(self, imu_topic, negate):
        super().__init__('vio_bench')
        self.negate = negate
        self.acc = deque(maxlen=400)
        self.frame = None
        self.odom_times = []
        self.info = []
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/rtabmap/vio_odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OdomInfo, '/rtabmap/vio_odom_info_lite', self.on_info, qos_profile_sensor_data)

    def on_imu(self, msg):
        self.frame = msg.header.frame_id
        a, w = msg.linear_acceleration, msg.angular_velocity
        s = -1.0 if self.negate else 1.0
        self.acc.append((time.monotonic(), a.x, s * a.y, s * a.z, math.sqrt(w.x ** 2 + w.y ** 2 + w.z ** 2)))

    def on_odom(self, _msg):
        self.odom_times.append(time.monotonic())

    def on_info(self, msg):
        self.info.append((time.monotonic(), bool(msg.lost), int(msg.inliers)))

    def odom_tilt(self):
        now = time.monotonic()
        recent = [a for a in self.acc if now - a[0] <= 0.5]
        if not recent or not self.frame:
            return None, False
        mean = [sum(a[i] for a in recent) / len(recent) for i in (1, 2, 3)]
        norm = math.sqrt(sum(c * c for c in mean))
        quiet = abs(norm - 9.81) < 0.3 and max(a[4] for a in recent) < 0.3
        try:
            t = self.tf.lookup_transform('rtabmap_odom', self.frame, rclpy.time.Time())
        except Exception:
            return None, quiet
        r = t.transform.rotation
        up = rotate((r.x, r.y, r.z, r.w), [c / norm for c in mean])
        return math.degrees(math.acos(max(-1.0, min(1.0, up[2])))), quiet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--label', default='run')
    parser.add_argument('--imu-topic', default='/rtabmap/imu_frame_fixed')
    parser.add_argument('--negate-yz', action='store_true')
    parser.add_argument('--out')
    args = parser.parse_args()
    rclpy.init()
    node = Bench(args.imu_topic, args.negate_yz)
    start = time.monotonic()
    cpu0 = container_cpu_seconds()
    tilts = []
    next_sample = start + 1.0
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() >= next_sample:
            next_sample += 1.0
            tilt, quiet = node.odom_tilt()
            if tilt is not None and quiet:
                tilts.append((time.monotonic() - start, tilt))
    elapsed = time.monotonic() - start
    cpu = (container_cpu_seconds() - cpu0) / elapsed * 100.0
    info = node.info
    lost = sum(1 for _, l, _ in info if l)
    inliers = sorted(i for _, l, i in info if not l)
    report = {
        'label': args.label, 'seconds': round(elapsed, 1),
        'vio_rate_hz': round(len(node.odom_times) / elapsed, 2),
        'odom_info_samples': len(info), 'lost_samples': lost,
        'inliers_p50': inliers[len(inliers) // 2] if inliers else None,
        'inliers_p5': inliers[len(inliers) // 20] if inliers else None,
        'odom_tilt_deg_quiet': [round(t, 2) for _, t in tilts],
        'odom_tilt_first': round(tilts[0][1], 2) if tilts else None,
        'odom_tilt_last': round(tilts[-1][1], 2) if tilts else None,
        'odom_tilt_max': round(max(t for _, t in tilts), 2) if tilts else None,
        'container_cpu_percent': round(cpu, 1),
    }
    print(json.dumps({k: v for k, v in report.items() if k != 'odom_tilt_deg_quiet'}), flush=True)
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(report, f, indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
