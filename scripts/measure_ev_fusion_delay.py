#!/usr/bin/env python3
"""Estimate EV-to-EKF2 response delay from live PX4 ROS 2 topics."""

import argparse
import math
import time

import numpy as np
import rclpy
from px4_msgs.msg import SensorCombined, VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_from_q(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Capture(Node):
    def __init__(self):
        super().__init__("measure_ev_fusion_delay")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.t0 = time.monotonic()
        self.ev = []
        self.lp = []
        self.imu = []
        self.create_subscription(
            VehicleOdometry,
            "/fmu/in/vehicle_visual_odometry",
            self.on_ev,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.on_lp,
            qos,
        )
        self.create_subscription(SensorCombined, "/fmu/out/sensor_combined", self.on_imu, qos)

    def now(self):
        return time.monotonic() - self.t0

    def on_ev(self, msg):
        self.ev.append(
            (self.now(), msg.position[0], msg.position[1], msg.position[2], yaw_from_q(msg.q))
        )

    def on_lp(self, msg):
        self.lp.append((self.now(), msg.x, msg.y, msg.z, msg.heading))

    def on_imu(self, msg):
        self.imu.append((self.now(), msg.gyro_rad[2]))


def interp(samples, t):
    a = np.asarray(samples, dtype=float)
    return np.column_stack([np.interp(t, a[:, 0], np.unwrap(a[:, i])) for i in range(1, 5)])


def normalized_score(a, b):
    a = a - np.mean(a, axis=0)
    b = b - np.mean(b, axis=0)
    # Weight each usable axis equally, regardless of motion amplitude.
    scores = []
    for i in range(a.shape[1]):
        denom = np.linalg.norm(a[:, i]) * np.linalg.norm(b[:, i])
        if denom > 1.0e-4:
            scores.append(float(np.dot(a[:, i], b[:, i]) / denom))
    return float(np.mean(scores)) if scores else float("nan")


def estimate(ev, lp):
    start = max(ev[0][0], lp[0][0]) + 1.0
    end = min(ev[-1][0], lp[-1][0]) - 1.0
    grid = np.arange(start, end, 0.02)
    if len(grid) < 100:
        raise RuntimeError("not enough overlapping samples")

    # Compare 250 ms displacement vectors. This removes arbitrary EKF/VIO origins
    # and is less sensitive than raw position to slow VIO drift.
    window = 0.25
    delays = np.arange(-0.10, 0.801, 0.005)
    scores = []
    lp_delta = interp(lp, grid) - interp(lp, grid - window)
    for delay in delays:
        ev_delta = interp(ev, grid - delay) - interp(ev, grid - delay - window)
        scores.append(normalized_score(ev_delta, lp_delta))
    scores = np.asarray(scores)
    best = int(np.nanargmax(scores))

    # A practical peak-width interval, not a statistical confidence interval.
    near = delays[scores >= scores[best] - 0.02]
    motion = interp(ev, grid)
    span = np.ptp(motion, axis=0)
    return delays[best], scores[best], near[0], near[-1], span


def estimate_against_gyro(ev, imu):
    start = max(ev[0][0], imu[0][0]) + 1.0
    end = min(ev[-1][0], imu[-1][0]) - 1.0
    grid = np.arange(start, end, 0.01)
    ev_a = np.asarray(ev, dtype=float)
    imu_a = np.asarray(imu, dtype=float)
    yaw = np.unwrap(ev_a[:, 4])
    # A 200 ms centered difference rejects quantization/noise at the 10-15 Hz VIO rate.
    half = 0.10
    ev_rate = (
        np.interp(grid + half, ev_a[:, 0], yaw)
        - np.interp(grid - half, ev_a[:, 0], yaw)
    ) / (2.0 * half)
    delays = np.arange(0.0, 0.601, 0.005)
    scores = []
    for delay in delays:
        gyro = np.interp(grid - delay, imu_a[:, 0], imu_a[:, 1])
        scores.append(normalized_score(ev_rate[:, None], gyro[:, None]))
    scores = np.asarray(scores)
    best = int(np.nanargmax(scores))
    near = delays[scores >= scores[best] - 0.02]
    return delays[best], scores[best], near[0], near[-1], float(np.ptp(yaw))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = Capture()
    print(f"Capturing {args.duration:.0f} s; move the props-off vehicle in x/y and yaw...")
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        f"Captured EV={len(node.ev)} samples, local_position={len(node.lp)} samples, "
        f"gyro={len(node.imu)} samples"
    )
    if len(node.ev) < 30 or len(node.lp) < 100:
        raise SystemExit("Insufficient topic data; verify VIO and PX4 DDS are streaming.")
    delay, score, low, high, span = estimate(node.ev, node.lp)
    print(
        f"EKF-output motion lag diagnostic (not EV delay): {delay * 1000:.0f} ms "
        f"(peak-width {low * 1000:.0f}..{high * 1000:.0f} ms, correlation {score:.3f})"
    )
    print(
        "EV motion span: "
        f"x={span[0]:.3f} m, y={span[1]:.3f} m, z={span[2]:.3f} m, "
        f"yaw={math.degrees(span[3]):.1f} deg"
    )
    if max(span[0], span[1]) < 0.15 and span[3] < math.radians(15.0):
        print("WARNING: motion excitation was too small for a trustworthy delay estimate.")
    if len(node.imu) >= 100:
        delay, score, low, high, yaw_span = estimate_against_gyro(node.ev, node.imu)
        print(
            f"VIO pipeline delay vs PX4 gyro: {delay * 1000:.0f} ms "
            f"(peak-width {low * 1000:.0f}..{high * 1000:.0f} ms, correlation {score:.3f}, "
            f"yaw span {math.degrees(yaw_span):.1f} deg)"
        )


if __name__ == "__main__":
    main()
