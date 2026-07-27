#!/usr/bin/env python3
"""Live readout of the map_correction node, bypassing the ROS 2 CLI.

`ros2 topic echo` is unreliable on this machine -- the daemon caches a stale
graph, so topics show up in `ros2 topic list` while echo reports
"Could not determine the type for the passed topic". A direct rclpy subscriber
has no daemon in the path.

Usage (needs the ROS 2 environment):
    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=42 python3 scripts/probe_map_correction.py

One line per second:
    applied  -- the rate-limited correction actually being applied
    target   -- the raw SLAM solution it is ramping toward
    pending  -- how much correction is still to come, and the implied ramp time
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


def yaw_deg(q):
    return math.degrees(
        math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    )


class Probe(Node):
    def __init__(self):
        super().__init__("map_correction_probe")
        self.applied = None
        self.target = None
        self.residual = None
        self.residual_deg = None
        self.counts = {"applied": 0, "target": 0, "residual": 0}
        self.peak_residual = 0.0

        self.create_subscription(
            PoseStamped, "/vio/map_correction", self.on_applied, 10)
        self.create_subscription(
            PoseStamped, "/vio/map_correction_target", self.on_target, 10)
        self.create_subscription(
            Float32, "/vio/map_correction/residual_m", self.on_residual, 10)
        self.create_subscription(
            Float32, "/vio/map_correction/residual_deg", self.on_residual_deg, 10)
        self.create_timer(1.0, self.report)
        self.started = time.monotonic()

    def on_applied(self, m):
        self.applied = m
        self.counts["applied"] += 1

    def on_target(self, m):
        self.target = m
        self.counts["target"] += 1

    def on_residual(self, m):
        self.residual = m.data
        self.counts["residual"] += 1
        self.peak_residual = max(self.peak_residual, m.data)

    def on_residual_deg(self, m):
        self.residual_deg = m.data

    def report(self):
        elapsed = time.monotonic() - self.started
        if self.counts["applied"] == 0:
            print(f"[{elapsed:5.1f}s] no messages yet -- is the map_correction node running? "
                  f"(pgrep -af map_correction)", flush=True)
            return

        a, t = self.applied.pose.position, self.target.pose.position
        # 0.03 m/s is the node default; only a rough guide if it was retuned.
        eta = (self.residual or 0.0) / 0.03
        print(
            f"[{elapsed:5.1f}s] "
            f"applied=({a.x:+.3f},{a.y:+.3f},{a.z:+.3f})m {yaw_deg(self.applied.pose.orientation):+6.2f}deg  "
            f"target=({t.x:+.3f},{t.y:+.3f},{t.z:+.3f})m {yaw_deg(self.target.pose.orientation):+6.2f}deg  "
            f"pending={self.residual * 100.0:5.1f}cm/{self.residual_deg:+5.2f}deg "
            f"(~{eta:4.1f}s)  peak={self.peak_residual * 100.0:.1f}cm",
            flush=True,
        )


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
