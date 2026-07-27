#!/usr/bin/env python3
"""Analyse an offboard_hold_yaw rosbag (and optionally its PX4 ULog).

Usage (needs the ROS 2 environment, NOT .venv-mavlink):
    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    python3 scripts/analyze_flight.py ros2_ws/flight_logs/<bag_dir>

Point it at the bag directory or straight at the `.mcap` — several bags have an
empty `metadata.yaml`, which makes rosbag2 refuse the directory form, so the
`.mcap` inside is resolved automatically.

Reports the things that have actually mattered on this airframe: yaw tracking
vs. the commanded slew, horizontal drift resolved into BODY axes (left/right is
the recurring failure), roll/pitch by flight phase (level-calibration bias), and
VIO health. Motor/torque data is not in the bag — use scripts/fetch_ulog.py and
analyze_ulog.py for that.

If the bag contains `/waypoint/commanded` it is treated as an offboard_waypoint
flight and a WAYPOINT TRACKING section is added. That matters because the two
flight types need different references: a hold flight is judged on distance from
the takeoff latch, while a waypoint flight is *supposed* to leave it, so its
hold error is measured against the commanded point instead — exactly what
OffboardWaypoint.hold_point feeds the watchdog.
"""
import glob
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

D = math.degrees

# Must match the offboard_waypoint launch defaults; they are node parameters and
# are not recorded in the bag. Update together with offboard_waypoint.launch.py.
WP_HOLD_GATE = 0.35       # max_horizontal_error, applied once settled
WP_TRANSIT_GATE = 0.60    # transit_horizontal_error, applied while translating
WP_SETTLE_TIME = 1.0      # transit_settle_time
WP_ARRIVAL_TOL = 0.12     # arrival_tol
WP_SPEED = 0.25           # waypoint_speed, m/s


def resolve(path):
    if os.path.isdir(path):
        hits = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not hits:
            sys.exit(f"no .mcap under {path}")
        return hits[-1]
    return path


def load(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = {}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        try:
            m = deserialize_message(data, get_message(types[topic]))
        except Exception:
            continue  # px4_msgs version skew on a few telemetry topics; not needed here
        msgs.setdefault(topic, []).append((ts, m))
    # A bag from a killed recorder has no message index, so rosbag2 falls back
    # to file order. That is normally write order, but sort anyway -- every
    # downstream calculation assumes monotonic time.
    for v in msgs.values():
        v.sort(key=lambda p: p[0])
    return msgs


def q2eul(q):
    """PX4 quaternion [w,x,y,z], NED/FRD -> roll, pitch, yaw in degrees."""
    w, x, y, z = q
    return (D(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            D(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
            D(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def wp_ned(m):
    """PoseStamped in the ENU `world` frame -> (x_ned, y_ned).

    px4_local_position_to_ros publishes x_enu=y_ned (east), y_enu=x_ned (north),
    so the pair simply swaps back.
    """
    return m.pose.position.y, m.pose.position.x


def waypoint_section(msgs, T, tl, x, y, arm_t, dis_t):
    """Waypoint tracking: accepted clicks, slew limiter, error vs the COMMANDED point.

    The takeoff-relative excursion printed above is the wrong number for these
    flights -- a waypoint flight leaves the latch on purpose. The watchdog
    measures the vehicle against the point currently being commanded, and that
    is what is reported here, split by the two gates the node actually applies.
    """
    cmd = msgs.get("/waypoint/commanded", [])
    if not cmd:
        return
    tgt = msgs.get("/waypoint/target", [])
    print("\n--- WAYPOINT TRACKING ---")

    tc = T([ts for ts, _ in cmd])
    cx = np.array([wp_ned(m)[0] for _, m in cmd])
    cy = np.array([wp_ned(m)[1] for _, m in cmd])

    # Accepted clicks = the points where the target jumps.
    if tgt:
        tt = T([ts for ts, _ in tgt])
        gx = np.array([wp_ned(m)[0] for _, m in tgt])
        gy = np.array([wp_ned(m)[1] for _, m in tgt])
        jump = np.hypot(np.diff(gx), np.diff(gy)) > 1e-4
        idx = [0] + (np.flatnonzero(jump) + 1).tolist()
        print(f"  accepted waypoints: {len(idx) - 1} (plus the initial hold point)")
        print("    #    t      target NED        travel   converged   settled err")
        for n, i in enumerate(idx):
            # Distance the vehicle had to cover, and when it first got inside
            # arrival_tol of the target.
            j = int(np.argmin(np.abs(tl - tt[i])))
            travel = math.hypot(gx[i] - x[j], gy[i] - y[j])
            after = tl >= tt[i]
            d = np.hypot(x - gx[i], y - gy[i])
            hit = np.flatnonzero(after & (d <= WP_ARRIVAL_TOL))
            conv = f"{tl[hit[0]] - tt[i]:5.1f}s" if hit.size else "   --"
            # Error once it has stopped chasing: the last second before the next
            # waypoint (or before the bag ends).
            end = tt[idx[n + 1]] if n + 1 < len(idx) else min(dis_t, tl[-1])
            w = (tl >= max(tt[i], end - 1.0)) & (tl <= end)
            serr = f"{d[w].mean():.3f} m" if w.sum() else "     --"
            print(f"    {n}  {tt[i]:6.2f}  ({gx[i]:+.3f},{gy[i]:+.3f})  "
                  f"{travel:5.2f} m   {conv}     {serr}")

    # Slew limiter. Measure the per-message STEP, never a speed: bag timestamps
    # are receive times and their jitter (observed 2 ms against a 20 ms median)
    # makes a divided rate look several times over the limit when the step is
    # in fact exact.
    step = np.hypot(np.diff(cx), np.diff(cy))
    nz = step[step > 1e-9]
    lim = WP_SPEED / 50.0  # waypoint_speed * dt at the node's 50 Hz
    print(f"  slew: max step {step.max()*1000:.3f} mm  (limit {lim*1000:.3f} mm"
          f" = {WP_SPEED} m/s at 50 Hz)  violations {(step > lim*1.01).sum()}/{step.size}")
    if nz.size:
        print(f"        moving {nz.size}/{step.size} intervals, median step {np.median(nz)*1000:.3f} mm")

    # Error vs the commanded point, split by which gate was live. Reconstruct
    # settled/transit the way the node does: the clock resets whenever the
    # commanded point moves, and the tight gate applies only after settle_time.
    settled = np.zeros(tc.size, dtype=bool)
    acc = 0.0
    for i in range(tc.size):
        if i and step[i - 1] > 1e-9:
            acc = 0.0
        elif i:
            acc += tc[i] - tc[i - 1]
        settled[i] = acc >= WP_SETTLE_TIME

    j = np.searchsorted(tl, tc).clip(0, tl.size - 1)
    err = np.hypot(x[j] - cx, y[j] - cy)
    fly = (tc >= arm_t) & (tc <= dis_t)
    print(f"  hold error vs COMMANDED point: mean {err[fly].mean():.3f} m"
          f"  max {err[fly].max():.3f} m at t={tc[fly][err[fly].argmax()]:.2f}")
    for lab, sel, gate in (("settled", fly & settled, WP_HOLD_GATE),
                           ("transit", fly & ~settled, WP_TRANSIT_GATE)):
        if not sel.sum():
            continue
        e = err[sel]
        over = (e > gate).sum()
        print(f"    {lab:8s} n={sel.sum():5d}  mean {e.mean():.3f}  max {e.max():.3f}"
              f"  gate {gate:.2f}  over-gate {over}"
              f"  margin used {e.max()/gate*100:.0f}%")
    # The number that would have aborted the flight under the pre-waypoint
    # logic, kept explicit because it is the whole reason hold_point exists.
    hard = (err[fly & settled] > WP_HOLD_GATE).sum() if (fly & settled).sum() else 0
    if hard:
        print(f"    WARNING: {hard} settled samples above the {WP_HOLD_GATE:.2f} m hold gate")


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(2)
    uri = resolve(sys.argv[1])
    print(f"=== {os.path.basename(uri)} ===")
    msgs = load(uri)
    t0 = min(v[0][0] for v in msgs.values())

    def T(ts):
        return (np.asarray(ts) - t0) / 1e9

    # ---- node state machine ----
    print("\n--- NODE STATE MACHINE ---")
    for ts, m in msgs.get("/rosout", []):
        if "offboard" in m.name or "yaw" in m.name or "hover" in m.name:
            print(f"  t={T(ts):6.2f} {m.msg}")

    # ---- arm window ----
    cm = msgs.get("/fmu/out/vehicle_control_mode", [])
    arm_t = next((T(ts) for ts, m in cm if m.flag_armed), None)
    dis_t, seen = None, False
    for ts, m in cm:
        if m.flag_armed:
            seen = True
        elif seen and dis_t is None:
            dis_t = T(ts)
    if arm_t is None:
        sys.exit("vehicle never armed in this bag")

    # ---- local position ----
    lp = msgs["/fmu/out/vehicle_local_position_v1"]
    tl = T([ts for ts, _ in lp])
    x = np.array([m.x for _, m in lp])
    y = np.array([m.y for _, m in lp])
    alt = -np.array([m.z for _, m in lp])
    vx = np.array([m.vx for _, m in lp])
    vy = np.array([m.vy for _, m in lp])
    hd = np.array([D(m.heading) for _, m in lp])

    airborne = alt > 0.15
    if not airborne.any():
        sys.exit("vehicle never exceeded 0.15 m — no airborne segment")
    lo, hi = tl[airborne][0], tl[airborne][-1]
    # A bag recovered from a killed recorder is truncated, so the disarm may
    # never have been written. Fall back to the last sample we do have.
    if dis_t is None:
        dis_t = tl[-1]
        print(f"  NOTE: bag truncated before disarm; using last sample t={dis_t:.2f}")
    steady_lo, steady_hi = lo + 0.6, hi - 0.5
    print(f"\n--- WINDOWS ---\n  armed {arm_t:.2f}..{dis_t:.2f}   airborne {lo:.2f}..{hi:.2f}"
          f"   steady {steady_lo:.2f}..{steady_hi:.2f}")

    i0 = int(np.argmax(tl >= arm_t))
    x0, y0, h0 = x[i0], y[i0], hd[i0]
    dN, dE = x - x0, y - y0
    c, s = math.cos(math.radians(h0)), math.sin(math.radians(h0))
    fwd = dN * c + dE * s
    right = -dN * s + dE * c

    fl = (tl >= arm_t) & (tl <= dis_t)
    st = (tl >= steady_lo) & (tl <= steady_hi)
    rad = np.hypot(dN, dE)
    is_wp = "/waypoint/commanded" in msgs
    print("\n--- HORIZONTAL DRIFT (body axes relative to heading at arm) ---")
    print(f"  heading at arm {h0:.1f} deg")
    # For a waypoint flight this distance is the travel envelope, not an error:
    # the vehicle is meant to leave the latch, so comparing it to the 0.35 m
    # abort would flag every successful flight. The real gate is measured
    # against /waypoint/commanded in the WAYPOINT TRACKING section below.
    note = ("(travel envelope -- NOT the abort threshold; see WAYPOINT TRACKING)"
            if is_wp else "(abort threshold 0.35)")
    print(f"  max excursion {rad[fl].max():.3f} m at t={tl[fl][rad[fl].argmax()]:.2f}"
          f"  {note}")
    print(f"  STEADY HOVER mean: fwd {fwd[st].mean():+.3f} m   right {right[st].mean():+.3f} m"
          f"   ({'LEFT' if right[st].mean() < 0 else 'RIGHT'} {abs(right[st].mean()):.3f} m)")
    print(f"  steady hover range: right {right[st].min():+.3f}..{right[st].max():+.3f}")

    print("\n   t     alt     fwd    right     vN     vE     hdg")
    for tt in np.arange(math.floor(arm_t), dis_t, 0.5):
        i = int(np.argmin(np.abs(tl - tt)))
        print(f"{tl[i]:6.2f} {alt[i]:6.3f} {fwd[i]:+7.3f} {right[i]:+7.3f} "
              f"{vx[i]:+6.2f} {vy[i]:+6.2f} {hd[i]:7.1f}")

    waypoint_section(msgs, T, tl, x, y, arm_t, dis_t)

    # ---- attitude by phase ----
    att = msgs["/fmu/out/vehicle_attitude"]
    ta = T([ts for ts, _ in att])
    eul = np.array([q2eul(m.q) for _, m in att])
    print("\n--- ROLL/PITCH BY PHASE (deg; +roll = right side down) ---")
    for a, b, lab in ((0.0, arm_t, "pre-arm on ground"),
                      (arm_t, lo, "armed, on ground"),
                      (steady_lo, steady_hi, "AIRBORNE STEADY"),
                      (hi, dis_t, "landing")):
        w = (ta >= a) & (ta < b)
        if w.sum():
            print(f"  {lab:20s} n={w.sum():5d}  roll {eul[w,0].mean():+6.2f}"
                  f" [{eul[w,0].min():+6.2f},{eul[w,0].max():+6.2f}]"
                  f"  pitch {eul[w,1].mean():+6.2f}")
    w = (ta >= steady_lo) & (ta <= steady_hi)
    r_m = eul[w, 0].mean()
    print(f"  => steady-hover roll {r_m:+.2f} deg implies {9.81*math.tan(math.radians(r_m)):+.3f} m/s^2"
          f" lateral bias (should be ~0 if levelled correctly)")

    # ---- gyro ----
    sc = msgs["/fmu/out/sensor_combined"]
    tg = T([ts for ts, _ in sc])
    g = np.array([m.gyro_rad for _, m in sc]) * 180 / math.pi
    w = (tg >= steady_lo) & (tg <= steady_hi)
    print("\n--- GYRO, AIRBORNE STEADY (deg/s) ---")
    for nm, i in (("p(roll)", 0), ("q(pitch)", 1), ("r(yaw)", 2)):
        v = g[w, i]
        print(f"  {nm:8s} mean {v.mean():+6.2f}  min {v.min():+7.2f}  max {v.max():+7.2f}"
              f"  rms {np.sqrt((v**2).mean()):5.2f}")
    print(f"  peak |yaw rate| whole flight {np.abs(g[:,2]).max():.1f} (abort threshold 60)")

    # ---- yaw tracking ----
    sp = msgs["/fmu/in/trajectory_setpoint"]
    tsp = T([ts for ts, _ in sp])
    spyaw = np.array([D(m.yaw) for _, m in sp])
    spyawspeed = np.array([m.yawspeed for _, m in sp])
    print("\n--- YAW TRACKING ---")
    print(f"  setpoints n={len(sp)}  yawspeed NaN {np.isnan(spyawspeed).sum()}/{len(sp)}"
          f"  yaw sp {np.nanmin(spyaw):.2f}..{np.nanmax(spyaw):.2f}")
    err = []
    for i, tt in enumerate(tsp):
        if steady_lo <= tt <= steady_hi:
            j = int(np.argmin(np.abs(tl - tt)))
            err.append((hd[j] - spyaw[i] + 180) % 360 - 180)
    if err:
        e = np.array(err)
        print(f"  airborne yaw error: mean {e.mean():+.2f}  min {e.min():+.2f}"
              f"  max {e.max():+.2f}  p2p {e.max()-e.min():.2f} deg")

    # ---- altitude ----
    print("\n--- ALTITUDE ---")
    print(f"  max {alt[fl].max():.3f} m; steady mean {alt[st].mean():.3f}"
          f" ({alt[st].min():.3f}..{alt[st].max():.3f})")

    # ---- battery ----
    bat = msgs.get("/fmu/out/battery_status_v1", [])
    if bat:
        tb = T([ts for ts, _ in bat])
        bv = np.array([m.voltage_v for _, m in bat])
        bc = np.array([m.current_a for _, m in bat])
        so = np.array([m.remaining for _, m in bat])
        w = (tb >= steady_lo) & (tb <= steady_hi)
        if w.sum():
            print(f"\n--- BATTERY (hover) --- V {bv[w].mean():.2f}  I {bc[w].mean():.1f} A"
                  f"  P {(bv[w]*bc[w]).mean():.0f} W  SoC {so[w].mean()*100:.0f}%")

    # ---- VIO ----
    fc = msgs.get("/rtabmap/vio_feature_count", [])
    vp = msgs.get("/rtabmap/vio_pose", [])
    if fc:
        f = np.array([m.data for _, m in fc])
        # 160 is the hover/yaw floor; offboard_waypoint runs 80. Report both so
        # the line is readable whichever launch produced the bag.
        print(f"\n--- VIO --- features median {np.median(f):.0f} min {f.min()}"
              f"  below-160 samples {(f<160).sum()}  below-80 {(f<80).sum()}")
    if vp:
        zero = sum(1 for _, m in vp
                   if abs(m.pose.position.x) < 1e-9 and abs(m.pose.position.y) < 1e-9
                   and abs(m.pose.position.z) < 1e-9)
        print(f"  vio_pose n={len(vp)}  reset sentinels (exact 0,0,0) {zero}")


if __name__ == "__main__":
    main()
