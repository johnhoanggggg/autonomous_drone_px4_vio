#!/usr/bin/env python3
"""Repeat the ROS-native RTAB-Map startup gate against the live OAK.

This is a props-off acceptance test.  It owns the camera while it runs, starts
the complete ROS-native launch once per attempt, and requires live image,
usable depth, IMU and non-lost odometry before calling an attempt healthy.

The v3 driver overlay, ROS workspace and ROS_DOMAIN_ID must already be sourced
in the shell.  Example:

    python3 scripts/ros_rtabmap_startup_sweep.py --profile mono --attempts 10

Nothing in the launched graph publishes to /fmu/in/*.
"""

import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image, Imu


BEST_EFFORT = QoSProfile(
    depth=5,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def stamp_seconds(msg):
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class AttemptMonitor(Node):
    def __init__(self, image_topic):
        super().__init__("ros_rtabmap_startup_sweep")
        self.image_topic = image_topic
        self.create_subscription(Image, image_topic, self.image_cb, BEST_EFFORT)
        self.create_subscription(
            Image, "/oak/stereo/image_raw", self.depth_cb, BEST_EFFORT)
        self.create_subscription(Imu, "/oak/imu/data", self.imu_cb, BEST_EFFORT)
        self.create_subscription(
            OdomInfo, "/rtabmap/vio_odom_info", self.odom_cb, BEST_EFFORT)
        self.reset_attempt()

    def reset_attempt(self):
        self.images = 0
        self.depths = 0
        self.imus = 0
        self.odom_samples = 0
        self.non_lost_samples = 0
        self.inliers = []
        self.features = []
        self.depth_nonzero_ratios = []
        self.last_image_stamp = None
        self.last_depth_stamp = None
        self.last_imu_stamp = None
        self._last_depth_check = 0.0

    def image_cb(self, msg):
        self.images += 1
        self.last_image_stamp = stamp_seconds(msg)

    def depth_cb(self, msg):
        self.depths += 1
        self.last_depth_stamp = stamp_seconds(msg)
        now = time.monotonic()
        if now - self._last_depth_check < 0.5 or msg.encoding not in (
                "16UC1", "mono16"):
            return
        self._last_depth_check = now
        # Sample rather than scan the full 640x400 frame.  Whether a uint16 is
        # zero is endian-independent, and the stride keeps this diagnostic
        # cheap enough to run during repeated starts.
        raw = memoryview(msg.data)
        if len(raw) >= 2:
            values = raw.cast("H")[::64]
            valid = sum(1 for value in values if value != 0)
            self.depth_nonzero_ratios.append(valid / max(1, len(values)))

    def imu_cb(self, msg):
        self.imus += 1
        self.last_imu_stamp = stamp_seconds(msg)

    def odom_cb(self, msg):
        self.odom_samples += 1
        if not msg.lost:
            self.non_lost_samples += 1
            self.inliers.append(int(msg.inliers))
        self.features.append(int(msg.features))

    def snapshot(self):
        median_inliers = statistics.median(self.inliers) if self.inliers else 0
        median_depth = (statistics.median(self.depth_nonzero_ratios)
                        if self.depth_nonzero_ratios else 0.0)
        image_depth_skew = None
        if self.last_image_stamp is not None and self.last_depth_stamp is not None:
            image_depth_skew = self.last_depth_stamp - self.last_image_stamp
        imu_image_skew = None
        if self.last_imu_stamp is not None and self.last_image_stamp is not None:
            imu_image_skew = self.last_imu_stamp - self.last_image_stamp
        return {
            "image_messages": self.images,
            "depth_messages": self.depths,
            "imu_messages": self.imus,
            "odom_info_messages": self.odom_samples,
            "non_lost_odom_messages": self.non_lost_samples,
            "median_inliers": median_inliers,
            "min_inliers": min(self.inliers) if self.inliers else 0,
            "median_features": (
                statistics.median(self.features) if self.features else 0),
            "median_depth_nonzero_ratio": round(median_depth, 4),
            "last_depth_minus_image_ms": (
                round(image_depth_skew * 1000.0, 2)
                if image_depth_skew is not None else None),
            "last_imu_minus_image_ms": (
                round(imu_image_skew * 1000.0, 2)
                if imu_image_skew is not None else None),
        }


def healthy(snapshot):
    """Return whether an attempt has crossed every startup gate."""
    return (
        snapshot["image_messages"] >= 5
        and snapshot["depth_messages"] >= 5
        and snapshot["imu_messages"] >= 20
        and snapshot["median_depth_nonzero_ratio"] >= 0.01
        and snapshot["non_lost_odom_messages"] >= 5
        and snapshot["median_inliers"] >= 20
    )


def drain_output(stream, lines):
    for line in iter(stream.readline, ""):
        lines.append(line.rstrip())
    stream.close()


def stop_launch(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        # This only terminates the child launch group created by this script.
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def launch_command(profile):
    launch_file = ("ros_rtabmap_oak_v3_mono.launch.py"
                   if profile == "mono"
                   else "ros_rtabmap_oak_v3.launch.py")
    command = [
        "ros2", "launch", "px4_vio_bridge",
        launch_file, "foxglove:=false",
    ]
    return command


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("mono", "rgb"), default="mono")
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--startup-timeout", type=float, default=40.0)
    parser.add_argument("--release-seconds", type=float, default=5.0)
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    if args.attempts < 1 or args.startup_timeout <= 0:
        parser.error("attempts and startup-timeout must be positive")

    repo = Path(__file__).resolve().parents[1]
    image_topic = ("/oak/left/image_rect" if args.profile == "mono"
                   else "/oak/rgb/image_raw")
    rclpy.init()
    monitor = AttemptMonitor(image_topic)
    results = []
    try:
        for attempt in range(1, args.attempts + 1):
            monitor.reset_attempt()
            lines = deque(maxlen=80)
            started = time.monotonic()
            process = subprocess.Popen(
                launch_command(args.profile),
                cwd=repo / "ros2_ws",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            reader = threading.Thread(
                target=drain_output, args=(process.stdout, lines), daemon=True)
            reader.start()
            passed = False
            decision_elapsed = None
            pre_stop_lines = []
            try:
                deadline = started + args.startup_timeout
                while time.monotonic() < deadline and process.poll() is None:
                    rclpy.spin_once(monitor, timeout_sec=0.05)
                    if healthy(monitor.snapshot()):
                        passed = True
                        break
            finally:
                decision_elapsed = time.monotonic() - started
                # Snapshot before our SIGINT so expected shutdown messages
                # cannot be misreported as startup failures.
                pre_stop_lines = list(lines)
                stop_launch(process)
                reader.join(timeout=2.0)

            snap = monitor.snapshot()
            error_lines = [
                line for line in pre_stop_lines
                if "[ERROR]" in line or "has crashed" in line
                or "X_LINK" in line or "Device" in line and "crash" in line
            ]
            passed = passed and not error_lines
            result = {
                "attempt": attempt,
                "passed": passed,
                "seconds_to_pass_or_timeout": round(
                    decision_elapsed, 2),
                **snap,
                "launch_exit_code": process.returncode,
                "relevant_errors": error_lines[-10:],
            }
            results.append(result)
            print(json.dumps(result, indent=2), flush=True)
            if attempt != args.attempts:
                end = time.monotonic() + args.release_seconds
                while time.monotonic() < end:
                    rclpy.spin_once(monitor, timeout_sec=0.05)

    except KeyboardInterrupt:
        print("\nSweep stopped early.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

    passed = sum(1 for result in results if result["passed"])
    report = {
        "profile": args.profile,
        "attempts_requested": args.attempts,
        "attempts_completed": len(results),
        "passed": passed,
        "pass_fraction": round(passed / len(results), 3) if results else 0.0,
        "flight_gate_10_of_10": len(results) >= 10 and passed == len(results),
        "results": results,
    }
    print(json.dumps(report, indent=2))
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")


if __name__ == "__main__":
    main()
