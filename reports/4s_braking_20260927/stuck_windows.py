"""Stuck-hold diagnosis: is the drone really stationary off-target, or is the estimate sticky?

Stuck window = reference stationary (hold), |p - p_ref| > OFF and 0.5 s-smoothed |v| < 0.04 m/s for >= MINLEN.
For each window, compare against the flight's normal (on-target, < 0.04 m) hold samples:
  - EV position innovation (VIO minus EKF prediction): mean along the push direction
  - EKF accel-bias drift across the window (states 13-15)
  - raw VIO displacement across the window vs EKF displacement
  - tilt-implied horizontal acceleration vs PX4 acceleration demand, along the push direction
"""
import sys

import numpy as np

OFF, MINLEN = 0.06, 1.0
from pyulog import ULog


def windows(mask, t, min_len):
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if t[i - 1] - t[start] >= min_len:
                out.append((t[start], t[i - 1]))
            start = None
    if start is not None and t[-1] - t[start] >= min_len:
        out.append((t[start], t[-1]))
    return out


def analyse(path):
    U = ULog(path)
    T = {d.name: d.data for d in U.data_list if d.multi_id == 0}
    s = lambda a: (a - U.start_timestamp) / 1e6
    ts = T["trajectory_setpoint"]; tt = s(ts["timestamp"])
    rp = np.column_stack([ts["position[0]"], ts["position[1]"]])
    rv = np.nan_to_num(np.column_stack([ts["velocity[0]"], ts["velocity[1]"]]))
    ok = np.isfinite(rp).all(1)
    cmd = T["vehicle_command"]
    first_move = tt[np.flatnonzero(np.linalg.norm(rv, axis=1) > 1e-3)[0]]
    land = [s(a) for a, c in zip(cmd["timestamp"], cmd["command"]) if int(c) == 21 and s(a) > first_move]
    t_end = min(land) if land else tt[-1]

    lp = T["vehicle_local_position"]; tl = s(lp["timestamp"])
    pos = np.column_stack([lp["x"], lp["y"]]); vel = np.column_stack([lp["vx"], lp["vy"]])
    ref = np.column_stack([np.interp(tl, tt[ok], rp[ok, k]) for k in range(2)])
    refspeed = np.interp(tl, tt, np.linalg.norm(rv, axis=1))
    err = ref - pos
    hold = (tl > first_move) & (tl < t_end - 0.3) & (refspeed < 1e-3)
    k_s = max(1, int(0.5 / np.median(np.diff(tl))))
    vs = np.column_stack([np.convolve(vel[:, i], np.ones(k_s) / k_s, "same") for i in range(2)])
    stuck = hold & (np.linalg.norm(err, axis=1) > OFF) & (np.linalg.norm(vs, axis=1) < .04)
    normal = hold & (np.linalg.norm(err, axis=1) < .04)

    ev = T["estimator_aid_src_ev_pos"]; te = s(ev["timestamp"])
    innov = np.column_stack([ev["innovation[0]"], ev["innovation[1]"]])
    es = T["estimator_states"]; tes = s(es["timestamp"])
    abias = np.column_stack([es["states[13]"], es["states[14]"]])
    vo = T["vehicle_visual_odometry"]; tv = s(vo["timestamp"])
    vpos = np.column_stack([vo["position[0]"], vo["position[1]"]])
    att = T["vehicle_attitude"]; ta = s(att["timestamp"])
    w, x, y, z = (att[f"q[{k}]"] for k in range(4))
    bz = np.column_stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
    atilt = -9.81 * bz[:, :2] / bz[:, 2:3]
    sp = T["vehicle_local_position_setpoint"]; tsp = s(sp["timestamp"])
    adem = np.column_stack([sp["acceleration[0]"], sp["acceleration[1]"]])

    nm = np.isin(np.searchsorted(tl, te), np.flatnonzero(normal))
    base_innov = np.linalg.norm(innov[nm], axis=1).mean() if nm.any() else np.nan
    rows = []
    for a, b in windows(stuck, tl, MINLEN):
        k = (tl >= a) & (tl <= b)
        u = err[k].mean(0); u /= np.linalg.norm(u)          # push direction (toward target)
        ke = (te >= a) & (te <= b); ks = (tes >= a) & (tes <= b)
        kv = (tv >= a) & (tv <= b); ka = (ta >= a) & (ta <= b); kd = (tsp >= a) & (tsp <= b)
        after = (tl > b) & (tl < b + 1.5)
        rows.append(dict(
            t0=a, dur=b - a, off=float(np.linalg.norm(err[k], axis=1).mean()),
            ekf_move=float((pos[k][-1] - pos[k][0]) @ u),
            vio_move=float((vpos[kv][-1] - vpos[kv][0]) @ u) if kv.sum() > 2 else np.nan,
            innov_along=float((innov[ke] @ u).mean()), innov_mag=float(np.linalg.norm(innov[ke], axis=1).mean()),
            base_innov=float(base_innov),
            abias_drift=float((abias[ks][-1] - abias[ks][0]) @ u) if ks.sum() > 2 else np.nan,
            dem=float((adem[kd] @ u).mean()), tilt=float((atilt[ka] @ u).mean()),
            v_after=float((vel[after] @ u).max()) if after.any() else np.nan,
        ))
    return rows


if __name__ == "__main__":
    for p in sys.argv[1:]:
        rows = analyse(p)
        print(p.split("_", 3)[-1][:-4] if "px4_" in p else p)
        for r in rows:
            print("  t %5.1f dur %.1fs off %.3f | move ekf %+.3f vio %+.3f | EV innov along %+.4f (|%.4f| vs normal %.4f) | "
                  "accel-bias drift %+.4f | demand %+.3f tilt %+.3f | v after %+.3f" % (
                      r["t0"], r["dur"], r["off"], r["ekf_move"], r["vio_move"], r["innov_along"], r["innov_mag"],
                      r["base_innov"], r["abias_drift"], r["dem"], r["tilt"], r["v_after"]))
