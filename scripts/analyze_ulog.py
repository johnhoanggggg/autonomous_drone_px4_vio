#!/usr/bin/env python3
"""Analyse a PX4 ULog for the things the rosbag cannot show: motor outputs and
control-allocator torque demands.

Usage (uses .venv-mavlink, which has pyulog):
    .venv-mavlink/bin/python scripts/analyze_ulog.py ros2_ws/flight_logs/px4_04_18_33.ulg
    .venv-mavlink/bin/python scripts/analyze_ulog.py a.ulg b.ulg      # compare several

The number that has mattered most on this airframe is the mean of
`vehicle_torque_setpoint.xyz[2]` (yaw) over the airborne steady window. A large
positive mean with the motor pairs running at very different levels means a
mechanical yaw-torque bias; see HANDOFF.md for the 2026-07-25 -> 07-26 history
(mean +0.145 with a 2.1x pair split, fixed to +0.004 / 1.02x by correcting the
prop spin directions).

Rotor index -> position, from the live CA_ROTOR* params:
    0 = front-right (KM +0.05)   1 = rear-left  (KM +0.05)
    2 = front-left  (KM -0.05)   3 = rear-right (KM -0.05)
So (0,1) and (2,3) are the two spin-direction pairs; (0,2) is front and (1,3) is
rear; (0,3) is right and (1,2) is left.
"""
import math
import os
import sys

import numpy as np
from pyulog import ULog


def analyse(path):
    u = ULog(path)
    D = {d.name: d.data for d in u.data_list}
    missing = {"vehicle_torque_setpoint", "actuator_motors", "vehicle_local_position"} - set(D)
    if missing:
        print(f"  !! missing topics: {sorted(missing)}")
        return
    t0 = min(v["timestamp"][0] for v in D.values())

    def T(a):
        return (np.asarray(a) - t0) / 1e6

    lp = D["vehicle_local_position"]
    tlp, alt = T(lp["timestamp"]), -lp["z"]
    air = alt > 0.15
    if not air.any():
        print("  !! never airborne")
        return
    lo, hi = tlp[air][0] + 0.6, tlp[air][-1] - 0.5
    print(f"  airborne {tlp[air][0]:.2f}..{tlp[air][-1]:.2f} s   steady window {lo:.2f}..{hi:.2f}")
    print(f"  max altitude {alt.max():.3f} m")

    ts = D["vehicle_torque_setpoint"]
    tt = T(ts["timestamp"])
    w = (tt >= lo) & (tt <= hi)
    print("  torque setpoint (normalised, +-1 = full authority):")
    for nm, k in (("roll  x", "xyz[0]"), ("pitch y", "xyz[1]"), ("YAW   z", "xyz[2]")):
        v = ts[k][w]
        print(f"    {nm}: mean {v.mean():+8.4f}  median {np.median(v):+8.4f}"
              f"  min {v.min():+8.4f}  max {v.max():+8.4f}  neg {(v<0).mean()*100:5.1f}%")

    if "vehicle_thrust_setpoint" in D:
        th = D["vehicle_thrust_setpoint"]
        wt = (T(th["timestamp"]) >= lo) & (T(th["timestamp"]) <= hi)
        print(f"    thrust z: mean {th['xyz[2]'][wt].mean():+.4f}"
              f"  (note: scales with battery voltage, not a clean efficiency metric)")

    am = D["actuator_motors"]
    tm = T(am["timestamp"])
    wm = (tm >= lo) & (tm <= hi)
    mo = np.array([am[f"control[{i}]"] for i in range(4)])
    names = ["FR", "RL", "FL", "RR"]
    print("  motors (airborne steady):")
    for i in range(4):
        print(f"    rotor {i} {names[i]}: mean {mo[i][wm].mean():.4f}"
              f"  min {mo[i][wm].min():.4f}  max {mo[i][wm].max():.4f}")

    def pair(a, b):
        return (mo[a][wm].mean() + mo[b][wm].mean()) / 2

    spin_a, spin_b = pair(0, 1), pair(2, 3)
    ratio = max(spin_a, spin_b) / max(min(spin_a, spin_b), 1e-9)
    print(f"    spin pairs : (0,1)={spin_a:.4f} vs (2,3)={spin_b:.4f}"
          f"   ratio {ratio:.3f}x   <-- yaw-bias indicator, want ~1.0")
    print(f"    front/rear : (0,2)={pair(0,2):.4f} vs (1,3)={pair(1,3):.4f}"
          f"   diff {pair(0,2)-pair(1,3):+.4f}  (fore/aft CG)")
    print(f"    right/left : (0,3)={pair(0,3):.4f} vs (1,2)={pair(1,2):.4f}"
          f"   diff {pair(0,3)-pair(1,2):+.4f}  (lateral CG)")
    print(f"    peak motor whole flight {mo.max():.4f}  (saturation at 1.0)")

    if "vehicle_attitude" in D:
        at = D["vehicle_attitude"]
        ta = T(at["timestamp"])
        wa = (ta >= lo) & (ta <= hi)
        q = np.array([at[f"q[{i}]"] for i in range(4)])
        roll = np.degrees(np.arctan2(2*(q[0]*q[1] + q[2]*q[3]),
                                     1 - 2*(q[1]**2 + q[2]**2)))
        pitch = np.degrees(np.arcsin(np.clip(2*(q[0]*q[2] - q[3]*q[1]), -1, 1)))
        r_m = roll[wa].mean()
        print(f"  attitude (airborne steady): roll {r_m:+.2f} deg  pitch {pitch[wa].mean():+.2f} deg")
        print(f"    => roll implies {9.81*math.tan(math.radians(r_m)):+.3f} m/s^2 lateral bias"
              f" (want ~0; nonzero = level-calibration error)")


def main():
    if len(sys.argv) < 2:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(2)
    for p in sys.argv[1:]:
        print(f"\n=== {os.path.basename(p)} ===")
        analyse(p)


if __name__ == "__main__":
    main()
