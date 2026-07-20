#!/usr/bin/env python3
"""Estimate PX4 EKF2_EV_POS_X/Y/Z by rocking a props-off vehicle by hand.

The external-vision position is modelled as

    p_ev_ned = p_fc_ned + R_ned_from_frd * r_ev_frd

where r_ev_frd is the EV sensor position relative to the flight-controller/vehicle
origin. Short-window affine trends are removed from p_ev_ned before fitting, so
slow hand translation is tolerated. Rotation about the FC is still much better
than freely translating the vehicle.

This tool is read-only: it prints suggested parameters and never writes to PX4.
"""

import argparse
import math
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleControlMode, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def quaternion_to_matrix(q):
    """Return the body-FRD to world-NED rotation for PX4 quaternion [w,x,y,z]."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1.0e-8:
        raise ValueError("invalid quaternion")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion_to_euler(q):
    r = quaternion_to_matrix(q)
    return np.array(
        [
            math.atan2(r[2, 1], r[2, 2]),
            math.asin(float(np.clip(-r[2, 0], -1.0, 1.0))),
            math.atan2(r[1, 0], r[0, 0]),
        ]
    )


def match_attitudes(ev, attitudes, delay_s, max_gap_s=0.08):
    """Match each received EV sample to the FC attitude at its measurement time."""
    att_t = np.asarray([sample[0] for sample in attitudes])
    matched_t, positions, rotations, quaternions = [], [], [], []
    for sample in ev:
        target = sample[0] - delay_s
        index = int(np.searchsorted(att_t, target))
        candidates = [i for i in (index - 1, index) if 0 <= i < len(attitudes)]
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(att_t[i] - target))
        if abs(att_t[best] - target) > max_gap_s:
            continue
        try:
            rotation = quaternion_to_matrix(attitudes[best][1])
        except ValueError:
            continue
        matched_t.append(sample[0])
        positions.append(sample[1])
        rotations.append(rotation)
        quaternions.append(attitudes[best][1])
    return (
        np.asarray(matched_t),
        np.asarray(positions),
        np.asarray(rotations),
        np.asarray(quaternions),
    )


def detrended_system(times, positions, rotations, window_s, min_window_angle_deg=4.0):
    """Remove position intercept/rate per window and return A r = b."""
    window_ids = np.floor((times - times[0]) / window_s).astype(int)
    a_blocks, b_blocks = [], []
    used_windows = 0
    for window_id in np.unique(window_ids):
        selected = np.flatnonzero(window_ids == window_id)
        if len(selected) < 6:
            continue
        window_rotations = rotations[selected]
        relative = window_rotations @ window_rotations[0].T
        angles = np.arccos(
            np.clip((np.trace(relative, axis1=1, axis2=2) - 1.0) / 2.0, -1, 1)
        )
        if np.ptp(angles) < math.radians(min_window_angle_deg):
            continue

        local_t = times[selected] - np.mean(times[selected])
        nuisance = np.column_stack((np.ones(len(selected)), local_t))
        projection = np.eye(len(selected)) - nuisance @ np.linalg.pinv(nuisance)
        p_residual = projection @ positions[selected]

        # Apply the same temporal high-pass projection to every element of R.
        r_residual = (
            projection @ window_rotations.reshape(len(selected), 9)
        ).reshape(-1, 3, 3)
        a_blocks.append(r_residual.reshape(-1, 3))
        b_blocks.append(p_residual.reshape(-1))
        used_windows += 1

    if not a_blocks:
        raise RuntimeError("no windows contained enough rotational motion")
    return np.vstack(a_blocks), np.concatenate(b_blocks), used_windows


def robust_fit(a, b, iterations=8):
    """Block-Huber least squares, keeping xyz residuals for a sample together."""
    weights = np.ones(len(b) // 3)
    estimate = np.zeros(3)
    for _ in range(iterations):
        row_weights = np.repeat(np.sqrt(weights), 3)
        estimate = np.linalg.lstsq(
            a * row_weights[:, None], b * row_weights, rcond=None
        )[0]
        residual_vectors = (b - a @ estimate).reshape(-1, 3)
        norms = np.linalg.norm(residual_vectors, axis=1)
        scale = 1.4826 * np.median(np.abs(norms - np.median(norms))) + 1.0e-6
        cutoff = max(0.003, 1.5 * scale)
        weights = np.minimum(1.0, cutoff / np.maximum(norms, 1.0e-9))

    residual = b - a @ estimate
    singular = np.linalg.svd(
        a * np.repeat(np.sqrt(weights), 3)[:, None], compute_uv=False
    )
    dof = max(1, len(b) - 3)
    sigma2 = float(np.dot(residual, residual) / dof)
    covariance = sigma2 * np.linalg.pinv(a.T @ a)
    return estimate, np.sqrt(np.maximum(0.0, np.diag(covariance))), residual, singular


class Capture(Node):
    def __init__(self):
        super().__init__("calibrate_ev_position_offset")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.start = time.monotonic()
        self.ev = []
        self.attitudes = []
        self.armed_seen = False
        self.reset_counters = set()
        self.create_subscription(
            VehicleOdometry, "/fmu/in/vehicle_visual_odometry", self.on_ev, qos
        )
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_fc_odometry, qos
        )
        self.create_subscription(
            VehicleControlMode,
            "/fmu/out/vehicle_control_mode",
            self.on_control_mode,
            qos,
        )

    def now(self):
        return time.monotonic() - self.start

    def on_ev(self, msg):
        position = np.asarray(msg.position, dtype=float)
        if np.all(np.isfinite(position)):
            self.ev.append((self.now(), position))

    def on_fc_odometry(self, msg):
        q = np.asarray(msg.q, dtype=float)
        if np.all(np.isfinite(q)):
            self.attitudes.append((self.now(), q))
            self.reset_counters.add(int(msg.reset_counter))

    def on_control_mode(self, msg):
        self.armed_seen |= bool(msg.flag_armed)


def motion_spans_deg(quaternions):
    euler = np.unwrap(
        np.asarray([quaternion_to_euler(q) for q in quaternions]), axis=0
    )
    return np.degrees(np.ptp(euler, axis=0))


def main():
    parser = argparse.ArgumentParser(
        description="Estimate PX4 EV sensor lever-arm parameters from hand rotation."
    )
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument(
        "--delay-ms",
        type=float,
        default=260.0,
        help="EV pipeline delay relative to current FC attitude (default: 260)",
    )
    parser.add_argument(
        "--window", type=float, default=1.5, help="detrending window in seconds"
    )
    args = parser.parse_args()
    if args.duration < 10.0 or args.window <= 0.25:
        parser.error("use --duration >= 10 and --window > 0.25")

    rclpy.init()
    node = Capture()
    print("PROPS OFF. Keep the FC near one point and repeatedly rock roll, pitch, and yaw.")
    print("Use quick, smooth rotations with only slow translation; excite every axis.")
    print(f"Capturing for {args.duration:.0f} s (EV delay {args.delay_ms:.0f} ms)...")
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline and not node.armed_seen:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.armed_seen:
        raise SystemExit("ABORTED: PX4 reported armed. This calibration is props-off only.")
    print(f"Captured EV={len(node.ev)}, FC attitude={len(node.attitudes)} samples")
    if len(node.ev) < 100 or len(node.attitudes) < 200:
        raise SystemExit(
            "Insufficient topic data; verify main launch and both vehicle_odometry topics."
        )
    if len(node.reset_counters) > 1:
        raise SystemExit("PX4 odometry reset during capture; repeat after the estimator settles.")

    times, positions, rotations, quaternions = match_attitudes(
        node.ev, node.attitudes, args.delay_ms / 1000.0
    )
    if len(times) < 100:
        raise SystemExit("Too few time-aligned samples; check DDS rates or --delay-ms.")
    spans = motion_spans_deg(quaternions)
    try:
        a, b, windows = detrended_system(
            times, positions, rotations, args.window
        )
    except RuntimeError as error:
        raise SystemExit(f"Insufficient rotational motion: {error}") from error
    estimate, uncertainty, residual, singular = robust_fit(a, b)
    rms = math.sqrt(float(np.mean(residual**2)))
    condition = float(singular[0] / max(singular[-1], 1.0e-12))

    print(f"Matched {len(times)} samples in {windows} useful motion windows")
    print(
        "FC attitude span: "
        f"roll={spans[0]:.1f} deg, pitch={spans[1]:.1f} deg, "
        f"yaw={spans[2]:.1f} deg"
    )
    print(f"Fit residual RMS={rms * 100:.1f} cm, condition={condition:.1f}")
    print("\nSuggested PX4 lever arm (meters, body FRD: +X forward, +Y right, +Z down):")
    labels = ("EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z")
    for label, value, sigma in zip(labels, estimate, uncertainty):
        print(f"  {label} = {value:+.4f}   (fit uncertainty ~{sigma:.4f} m)")

    trustworthy = True
    if np.any(spans < 25.0):
        print("WARNING: one or more axes had <25 deg excitation; rock all three axes more.")
        trustworthy = False
    if condition > 20.0:
        print("WARNING: rotational excitation is poorly conditioned.")
        trustworthy = False
    if rms > 0.03:
        print("WARNING: residual >3 cm; translate less and rotate closer to the FC origin.")
        trustworthy = False
    if np.any(uncertainty > 0.02):
        print("WARNING: estimated uncertainty exceeds 2 cm on at least one axis.")
        trustworthy = False
    if np.any(np.abs(estimate) > 1.0):
        print("WARNING: an offset exceeds PX4's expected physical range; reject this fit.")
        trustworthy = False

    if trustworthy:
        print("\nFit checks passed. Repeat twice; accept only repeatable values, then set via NSH:")
        for label, value in zip(labels, estimate):
            print(f"  param set {label} {value:.4f}")
        print("  param save")
    else:
        print("\nDo not apply this result. Repeat the capture with better rotational motion.")


if __name__ == "__main__":
    main()
