#!/usr/bin/env python3
"""Measure the camera lever arm (EKF2_EV_POS_X/Y) by rotating the vehicle in place.

WHY
    VIO reports where the *camera* is. PX4 needs where the *body origin* (the FC)
    is, and converts with EKF2_EV_POS_X/Y/Z. If that offset is wrong, the error
    rotates with the airframe: invisible at a fixed heading, but during a yaw PX4
    sees a phantom drift and physically flies the vehicle sideways to cancel it.
    The vehicle then orbits instead of pivoting.

METHOD
    Rotate the vehicle in place about the flight controller and watch the camera.
    A pure rotation makes the camera trace a circular arc whose radius IS the
    lever arm:

        p_camera(t) = pivot + R(yaw(t)) . r          (r = lever arm, body FLU)

    Four unknowns (pivot x/y, r x/y), hundreds of samples. Least squares.

PROCEDURE (props off, on the ground)
    1. Start the VIO stack:
         ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py
       Wave the vehicle around until VIO has initialised and features are healthy.
    2. Mark a spot on the floor directly under the FLIGHT CONTROLLER. That spot is
       the pivot, and the answer is measured relative to it. Under the camera or
       under the middle of the airframe gives you the wrong number.
    3. Run this script and follow the prompts. Rotate the vehicle smoothly through
       at least 90 degrees -- more is better -- keeping the FC over its mark and
       the vehicle level. Back-and-forth sweeps are fine and help.
    4. Apply the printed EKF2_EV_POS_X/Y over NSH, then re-fly the yaw test.

    Rotation about a vertical axis says NOTHING about EKF2_EV_POS_Z. Leave Z as
    measured with a ruler.

USAGE
    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42

    # live capture (default):
    python3 scripts/measure_ev_position_offset.py

    # re-analyse a recording instead:
    python3 scripts/measure_ev_position_offset.py --bag ros2_ws/flight_logs/<bag>
"""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np

TOPIC = "/rtabmap/vio_pose"
# Current values, for the "change" column only. Verify with:
#   .venv-mavlink/bin/python scripts/nsh.py "param show EKF2_EV_POS_X"
CONFIGURED_X, CONFIGURED_Y = 0.100, -0.036


def yaw_of(qx, qy, qz, qw):
    """Yaw about the vertical, from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


# --------------------------------------------------------------------------- fit

def fit(x, y, psi, drift_t=None):
    """Least-squares solve of  p = c [+ v*t] + R(psi).r.

    The VIO pose is ENU position / FLU orientation (that is what
    vio_to_px4_odometry.enu_flu_pose_to_ned_frd assumes), so r comes out in FLU:
    +x forward, +y LEFT. The caller flips y to get body FRD.
    """
    n = len(x)
    ncol = 6 if drift_t is not None else 4
    A = np.zeros((2 * n, ncol))
    b = np.empty(2 * n)
    A[0::2, 0] = 1.0
    A[1::2, 1] = 1.0
    A[0::2, 2] = np.cos(psi)
    A[0::2, 3] = -np.sin(psi)
    A[1::2, 2] = np.sin(psi)
    A[1::2, 3] = np.cos(psi)
    if drift_t is not None:
        A[0::2, 4] = drift_t
        A[1::2, 5] = drift_t
    b[0::2] = x
    b[1::2] = y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    res = (b - A @ sol).reshape(-1, 2)
    return sol, np.sqrt((res ** 2).sum(axis=1).mean())


def block_bootstrap(x, y, psi, t, iters=200, block_s=1.0, seed=0):
    """Uncertainty on r. Resamples contiguous blocks, because consecutive VIO
    samples are correlated and a plain per-sample bootstrap would flatter us."""
    rng = np.random.default_rng(seed)
    n = len(x)
    dt = np.median(np.diff(t)) if n > 1 else 0.1
    bs = max(2, int(round(block_s / max(dt, 1e-3))))
    nb = max(1, n // bs)
    out = []
    for _ in range(iters):
        idx = np.concatenate([np.arange(s, min(s + bs, n))
                              for s in rng.integers(0, max(1, n - bs), nb)])
        if len(idx) < 8:
            continue
        try:
            sol, _ = fit(x[idx], y[idx], psi[idx])
        except np.linalg.LinAlgError:
            continue
        out.append(sol[2:4])
    return np.array(out)


def report(t, x, y, psi, min_yaw_deg, drift):
    n = len(x)
    if n < 30:
        sys.exit(f"only {n} usable samples -- need a longer capture")

    psi = np.unwrap(psi)
    sweep = math.degrees(psi.max() - psi.min())
    dur = t[-1] - t[0]

    print(f"\n  samples      {n}  over {dur:.1f} s ({n/max(dur,1e-9):.1f} Hz)")
    print(f"  yaw swept    {sweep:.1f} deg")

    sol, rms = fit(x, y, psi, drift_t=(t - t[0]) if drift else None)
    rx, ry = sol[2], sol[3]
    # FLU -> body FRD: forward is the same axis, right = -left
    ev_x, ev_y = rx, -ry
    r_mag = math.hypot(rx, ry)

    scatter = math.sqrt(((x - x.mean()) ** 2 + (y - y.mean()) ** 2).mean())
    explained = 100.0 * (1.0 - rms / scatter) if scatter > 1e-9 else 0.0

    boot = block_bootstrap(x, y, psi, t)
    if len(boot) > 20:
        sx, sy = boot[:, 0].std(), boot[:, 1].std()
        ci = f"  +-{sx*100:.1f} / {sy*100:.1f} cm (1 sigma)"
    else:
        sx = sy = float("nan")
        ci = "  (bootstrap failed)"

    print(f"  camera track scatter {scatter*100:.1f} cm"
          f" -> residual after fit {rms*100:.1f} cm  ({explained:.0f}% explained)")

    print("\n" + "=" * 62)
    print("  MEASURED LEVER ARM (body FRD, +X forward, +Y right)")
    print("=" * 62)
    print(f"    EKF2_EV_POS_X = {ev_x:+.4f} m      (currently {CONFIGURED_X:+.4f},"
          f" change {ev_x-CONFIGURED_X:+.4f})")
    print(f"    EKF2_EV_POS_Y = {ev_y:+.4f} m      (currently {CONFIGURED_Y:+.4f},"
          f" change {ev_y-CONFIGURED_Y:+.4f})")
    print(f"    |r| = {r_mag:.4f} m{ci}")
    print(f"    EKF2_EV_POS_Z  -- not measurable by yawing; leave it alone.")

    # ---- quality gates
    warn = []
    if sweep < min_yaw_deg:
        warn.append(f"yaw sweep only {sweep:.0f} deg. Under ~{min_yaw_deg:.0f} deg the "
                    f"fit is ill-conditioned and |r| is unreliable -- redo with a "
                    f"bigger rotation.")
    if rms > 0.02:
        warn.append(f"residual {rms*100:.1f} cm is high. The pivot probably moved, the "
                    f"vehicle tilted, or VIO drifted. Redo, keeping the FC over its mark.")
    if explained < 70:
        warn.append(f"only {explained:.0f}% of the camera motion fits a pure rotation. "
                    f"You translated as well as rotated.")
    if max(sx, sy) > 0.03:
        warn.append(f"scatter across bootstrap resamples is large -- treat this as a "
                    f"rough number and repeat the run.")
    if r_mag > 0.35:
        warn.append(f"|r| = {r_mag:.2f} m is implausibly large for this airframe. "
                    f"Check you pivoted about the FC and not a corner of the room.")

    if warn:
        print("\n  WARNINGS")
        for w in warn:
            print(f"    ! {w}")
        print("\n  >>> DO NOT APPLY THESE NUMBERS. Fix the capture and repeat. <<<")
        print()
        return ev_x, ev_y

    print("\n  Fit looks clean.")
    print("\n  Apply with:")
    print(f'    .venv-mavlink/bin/python scripts/nsh.py \\')
    print(f'      "param set EKF2_EV_POS_X {ev_x:.4f}" \\')
    print(f'      "param set EKF2_EV_POS_Y {ev_y:.4f}" \\')
    print(f'      "param show EKF2_EV_POS_X" "param show EKF2_EV_POS_Y"')
    print("\n  Then re-fly the 45 deg yaw test and check the excursion during the turn:")
    print("    python3 scripts/analyze_flight.py ros2_ws/flight_logs/<new_bag>")
    print()
    return ev_x, ev_y


# ------------------------------------------------------------------- collection

def drop_resets(t, x, y, z, psi):
    """A VIO reset republishes an exact (0,0,0) and restarts the odom frame.
    Every sample paired across a reset is meaningless, so keep only the longest
    clean run."""
    sentinel = (np.abs(x) < 1e-9) & (np.abs(y) < 1e-9) & (np.abs(z) < 1e-9)
    if not sentinel.any():
        return t, x, y, psi, 0
    cuts = [-1] + list(np.flatnonzero(sentinel)) + [len(x)]
    best = max(((a + 1, b) for a, b in zip(cuts, cuts[1:])), key=lambda s: s[1] - s[0])
    a, b = best
    return t[a:b], x[a:b], y[a:b], psi[a:b], int(sentinel.sum())


def from_bag(path, min_yaw_deg, drift):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from analyze_flight import load, resolve

    uri = resolve(path)
    print(f"=== {os.path.basename(uri)} ===")
    msgs = load(uri)
    if TOPIC not in msgs:
        sys.exit(f"{TOPIC} not in this bag (has: {', '.join(sorted(msgs))})")

    # This measurement is only meaningful for a bench rotation about a fixed
    # pivot. On a flight bag the vehicle translates as well, and the fit returns
    # the arc it happened to fly -- not the lever arm.
    if any(m.flag_armed for _, m in msgs.get("/fmu/out/vehicle_control_mode", [])):
        print("\n  ! This bag contains an ARMED flight. This script measures a")
        print("    props-off rotation about a fixed pivot; a flight will not give")
        print("    a valid lever arm. Expect the quality gates below to reject it.")
    rows = msgs[TOPIC]
    t = np.array([ts for ts, _ in rows], dtype=float) / 1e9
    x = np.array([m.pose.position.x for _, m in rows])
    y = np.array([m.pose.position.y for _, m in rows])
    z = np.array([m.pose.position.z for _, m in rows])
    psi = np.array([yaw_of(m.pose.orientation.x, m.pose.orientation.y,
                           m.pose.orientation.z, m.pose.orientation.w) for _, m in rows])
    t, x, y, psi, nres = drop_resets(t, x, y, z, psi)
    if nres:
        print(f"  dropped {nres} VIO reset sentinel(s); using the longest clean run")
    report(t - t[0], x, y, psi, min_yaw_deg, drift)


def from_live(duration, min_yaw_deg, drift):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped

    # BEST_EFFORT subscriber matches both a best-effort and a reliable publisher.
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=50,
                     durability=DurabilityPolicy.VOLATILE)

    class Collector(Node):
        def __init__(self):
            super().__init__("measure_ev_position_offset")
            self.rows = []
            self.recording = False
            self.seen = 0
            self.create_subscription(PoseStamped, TOPIC, self.cb, qos)

        def cb(self, m):
            self.seen += 1
            if not self.recording:
                return
            p, o = m.pose.position, m.pose.orientation
            self.rows.append((time.monotonic(), p.x, p.y, p.z,
                              yaw_of(o.x, o.y, o.z, o.w)))

    rclpy.init()
    node = Collector()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print(f"waiting for {TOPIC} ...")
    t_wait = time.monotonic()
    while node.seen == 0:
        if time.monotonic() - t_wait > 20:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(f"no messages on {TOPIC}. Is the VIO stack running, and is "
                     f"ROS_DOMAIN_ID=42 exported?")
        time.sleep(0.2)
    print(f"receiving ({node.seen} msgs so far).\n")

    print("  PROPS OFF. Put the FLIGHT CONTROLLER over a marked spot on the floor.")
    print("  When recording starts, rotate the vehicle smoothly in place through at")
    print(f"  least {min_yaw_deg:.0f} degrees, keeping the FC on its mark and the")
    print("  vehicle level. Back-and-forth sweeps are fine.\n")
    try:
        input("  Press ENTER to start recording... ")
    except EOFError:
        sys.exit("live mode needs an interactive terminal; use --bag instead")

    node.rows.clear()
    node.recording = True
    t0 = time.monotonic()

    stop = threading.Event()

    def wait_enter():
        try:
            input()
        except EOFError:
            pass
        stop.set()

    threading.Thread(target=wait_enter, daemon=True).start()
    print("  RECORDING -- rotate now. Press ENTER when done.\n")

    while not stop.is_set():
        el = time.monotonic() - t0
        if el > duration:
            print("\n  duration reached.")
            break
        rows = list(node.rows)
        if len(rows) > 5:
            psi = np.unwrap([r[4] for r in rows])
            sweep = math.degrees(psi.max() - psi.min())
            bar = "#" * min(40, int(40 * sweep / max(min_yaw_deg, 1)))
            ok = "OK " if sweep >= min_yaw_deg else "..."
            sys.stdout.write(f"\r  {el:5.1f}s  n={len(rows):4d}  swept "
                             f"{sweep:6.1f} deg {ok}|{bar:<40}|")
            sys.stdout.flush()
        time.sleep(0.1)

    node.recording = False
    rows = list(node.rows)
    node.destroy_node()
    rclpy.shutdown()
    print()

    if len(rows) < 30:
        sys.exit(f"only {len(rows)} samples captured -- record for longer")
    a = np.array(rows)
    t, x, y, z, psi = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4]
    t, x, y, psi, nres = drop_resets(t, x, y, z, psi)
    if nres:
        print(f"  dropped {nres} VIO reset sentinel(s); using the longest clean run")
    report(t - t[0], x, y, psi, min_yaw_deg, drift)


def main():
    ap = argparse.ArgumentParser(
        description="Measure EKF2_EV_POS_X/Y from an in-place rotation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--bag", metavar="PATH",
                    help="analyse a recorded bag (dir or .mcap) instead of live capture")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="live capture cap in seconds (default 60)")
    ap.add_argument("--min-yaw", type=float, default=90.0, metavar="DEG",
                    help="yaw sweep considered sufficient (default 90)")
    ap.add_argument("--drift", action="store_true",
                    help="also fit a constant translational drift. Off by default: on "
                         "the bench there should be none, and fitting it can absorb "
                         "real signal. Useful when analysing an actual flight.")
    args = ap.parse_args()

    if args.bag:
        from_bag(args.bag, args.min_yaw, args.drift)
    else:
        from_live(args.duration, args.min_yaw, args.drift)


if __name__ == "__main__":
    main()
