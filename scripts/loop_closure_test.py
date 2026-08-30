#!/usr/bin/env python3
"""Hand-carried loop-closure test for the hybrid stack.

Loop closure has never fired in any bench test: stationary, the SLAM correction
stays identity, so the corrected-pose path and cpp_slam_pose_relay's
non-identity branch are unexercised outside unit tests. This records a carried
loop and reports what closure does to the pose PX4 would consume.

Usage, with the hybrid already running:

    ros2 launch px4_vio_bridge rtabmap_slam_px4_3d.launch.py foxglove:=false
    python3 scripts/loop_closure_test.py --seconds 180

Then: hold still ~20 s, carry the drone slowly around a closed loop of a few
metres keeping textured scenery in view, return it to the marked start in the
original orientation, hold still ~20 s. No props.

Note: the "odometry_while_moving" section only appears on the ROS-native stack
(ros_rtabmap_oak_v3.launch.py). The hybrid's odometry is a DepthAI host node,
which publishes no rtabmap_msgs/OdomInfo, so lost/inlier counts are unavailable
there and the section is simply omitted.
"""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info, OdomInfo


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def mean_point(points):
    n = float(len(points))
    return tuple(sum(p[i] for p in points) / n for i in range(3))


class LoopClosureTest(Node):
    def __init__(self, args):
        super().__init__("loop_closure_test")
        self.args = args
        self.raw = []          # (t, x, y, z) from the pose PX4 consumes
        self.corrected = []    # (t, x, y, z) from the SLAM-corrected pose
        self.corrections = []  # (t, |translation| of map->odom)
        self.closures = []     # (t, ref_id, loop_id, |transform|)
        self.odom = []         # (t, lost, inliers, features)

        best = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.BEST_EFFORT)
        rel = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        for qos in (best, rel):
            self.create_subscription(PoseStamped, args.raw_pose_topic,
                                     self._raw_cb, qos)
            self.create_subscription(PoseStamped, args.pose_topic,
                                     self._corrected_cb, qos)
            self.create_subscription(PoseStamped, args.correction_topic,
                                     self._correction_cb, qos)
            self.create_subscription(Info, args.info_topic, self._info_cb, qos)
            self.create_subscription(OdomInfo, args.odom_info_topic,
                                     self._odom_cb, qos)

    def _pt(self, msg):
        p = msg.pose.position
        return (time.monotonic(), p.x, p.y, p.z)

    def _raw_cb(self, msg):
        self.raw.append(self._pt(msg))

    def _corrected_cb(self, msg):
        self.corrected.append(self._pt(msg))

    def _correction_cb(self, msg):
        p = msg.pose.position
        self.corrections.append(
            (time.monotonic(), math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)))

    def _info_cb(self, msg):
        if msg.loop_closure_id != 0 or msg.proximity_detection_id != 0:
            t = msg.loop_closure_transform.translation
            self.closures.append((
                time.monotonic(), int(msg.ref_id),
                int(msg.loop_closure_id or msg.proximity_detection_id),
                math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2),
            ))

    def _odom_cb(self, msg):
        self.odom.append(
            (time.monotonic(), bool(msg.lost), int(msg.inliers),
             int(msg.features)))


def biggest_jumps(series, n=5):
    """Largest consecutive-sample position steps, in mm."""
    steps = [(1000.0 * dist(series[i][1:], series[i + 1][1:]),
              round(series[i + 1][0] - series[0][0], 1))
             for i in range(len(series) - 1)]
    steps.sort(reverse=True)
    return [{"jump_mm": round(s, 2), "at_s": t} for s, t in steps[:n]]


def report(node, hold):
    out = {}
    raw, corr = node.raw, node.corrected

    if len(raw) > 20:
        span = raw[-1][0] - raw[0][0]
        start = [p for p in raw if p[0] - raw[0][0] <= hold]
        end = [p for p in raw if raw[-1][0] - p[0] <= hold]
        travelled = sum(dist(raw[i][1:], raw[i + 1][1:])
                        for i in range(len(raw) - 1))
        out["raw_vio_pose_the_one_px4_uses"] = {
            "samples": len(raw),
            "duration_s": round(span, 1),
            "rate_hz": round((len(raw) - 1) / span, 1) if span else 0.0,
            "path_travelled_m": round(travelled, 2),
            "start_hold_samples": len(start),
            "end_hold_samples": len(end),
            "return_to_origin_error_mm": (
                round(1000.0 * dist(mean_point([p[1:] for p in start]),
                                    mean_point([p[1:] for p in end])), 1)
                if start and end else None),
        }
    else:
        out["raw_vio_pose_the_one_px4_uses"] = {"samples": len(raw),
                                                "error": "insufficient data"}

    out["loop_closures"] = {
        "count": len(node.closures),
        "events": [{"at_s": round(t - (raw[0][0] if raw else t), 1),
                    "ref_id": r, "matched_id": l,
                    "transform_mm": round(1000.0 * m, 1)}
                   for t, r, l, m in node.closures[:20]],
    }

    if node.corrections:
        mags = [m for _, m in node.corrections]
        out["map_to_odom_correction"] = {
            "samples": len(mags),
            "max_mm": round(1000.0 * max(mags), 1),
            "final_mm": round(1000.0 * mags[-1], 1),
            "ever_non_identity": bool(max(mags) > 1e-6),
        }

    if len(corr) > 2:
        out["corrected_pose_jumps"] = {
            "samples": len(corr),
            "largest": biggest_jumps(corr),
            "note": ("PX4 is fed the RAW pose precisely to avoid these steps; "
                     "this quantifies what it avoids."),
        }

    if node.odom:
        lost = sum(1 for _, l, _, _ in node.odom if l)
        inl = [i for _, _, i, _ in node.odom]
        out["odometry_while_moving"] = {
            "samples": len(node.odom),
            "lost_fraction": round(lost / len(node.odom), 3),
            "inliers_mean": round(sum(inl) / len(inl)),
            "inliers_min": min(inl),
        }

    verdict = []
    if not node.closures:
        verdict.append("NO loop closure fired - the test proved nothing about "
                       "closure; repeat with a more textured route.")
    if node.corrections and max(m for _, m in node.corrections) <= 1e-6:
        verdict.append("Correction stayed identity, so the relay's "
                       "non-identity branch is still unexercised.")
    r = out["raw_vio_pose_the_one_px4_uses"].get("return_to_origin_error_mm")
    if r is not None:
        verdict.append(f"Raw VIO returned to within {r} mm - this is the drift "
                       "PX4 would see, and loop closure cannot remove it.")
    out["verdict"] = verdict
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--hold-seconds", type=float, default=20.0,
                    help="Stationary window at each end used for the "
                         "return-to-origin comparison.")
    ap.add_argument("--raw-pose-topic", default="/rtabmap/vio_pose")
    ap.add_argument("--pose-topic", default="/rtabmap/pose")
    ap.add_argument("--correction-topic", default="/rtabmap/odom_correction")
    ap.add_argument("--info-topic", default="/rtabmap3d/info")
    ap.add_argument("--odom-info-topic", default="/rtabmap/vio_odom_info")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    rclpy.init()
    node = LoopClosureTest(args)
    print(f"Recording {args.seconds:.0f} s. Hold still ~{args.hold_seconds:.0f} s, "
          "carry a closed loop, return to the marked start, hold still again.")
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("\nstopped early")

    result = report(node, args.hold_seconds)
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text + "\n")
        print(f"written to {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
