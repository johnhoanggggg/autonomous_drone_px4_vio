"""ULog-only route tracking / braking metrics for global-adapter flights."""
import sys

import numpy as np
from pyulog import ULog


def eul(q):
    r = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
    p = np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
    return r, p


def analyse(path, label):
    U = ULog(path)
    T = {d.name: d.data for d in U.data_list if d.multi_id == 0}
    s = lambda a: (a - U.start_timestamp) / 1e6
    ts = T["trajectory_setpoint"]; tt = s(ts["timestamp"])
    rp = np.column_stack([ts["position[0]"], ts["position[1]"]])
    rv = np.column_stack([ts["velocity[0]"], ts["velocity[1]"]])
    rvn = np.linalg.norm(np.nan_to_num(rv), axis=1)
    mv = np.flatnonzero(rvn > 1e-3)
    t0 = tt[mv[0]]
    cmd = T["vehicle_command"]
    land = [s(a) for a, c in zip(cmd["timestamp"], cmd["command"]) if int(c) == 21 and s(a) > t0]
    t1 = min(land) if land else tt[-1]
    lp = T["vehicle_local_position"]; tl = s(lp["timestamp"])
    m = (tl >= t0) & (tl <= t1)
    pos = np.column_stack([lp["x"], lp["y"]]); vel = np.column_stack([lp["vx"], lp["vy"]])
    spd = np.linalg.norm(vel, axis=1)
    fin = np.isfinite(rp).all(1)
    rpx = np.interp(tl, tt[fin], rp[fin, 0]); rpy = np.interp(tl, tt[fin], rp[fin, 1])
    perr = np.hypot(pos[:, 0] - rpx, pos[:, 1] - rpy)
    rvi = np.interp(tl, tt, rvn)
    # stops: ref speed crosses from >=0.12 down to <=0.02
    stops = []
    i = 0
    while i < len(tt) - 1:
        if tt[i] < t0 or tt[i] > t1:
            i += 1; continue
        if rvn[i] >= .12:
            j = i
            while j < len(tt) - 1 and rvn[j] > .02:
                j += 1
            if j < len(tt) - 1 and tt[j] < t1 - .5 and np.isfinite(rp[j]).all():
                d = np.nan_to_num(rv[i]) / rvn[i]
                k = j
                while k < len(tt) - 1 and rvn[k] <= .02 and tt[k] < tt[j] + 2.5:
                    k += 1
                hold_s = tt[k] - tt[j]
                w = (tl >= tt[j]) & (tl <= min(tt[k], t1))
                along = (pos[w] - rp[j]) @ d
                v_along = vel[w] @ d
                stops.append(dict(t=float(tt[j]), v_at_ref_stop=float(np.interp(tt[j], tl, vel @ d)),
                                  overshoot=float(max(0, along.max())), v_max_after=float(v_along.max()),
                                  settle_speed_1s=float(np.interp(tt[j] + 1, tl, spd)), hold_s=float(hold_s)))
            i = j + 1
        else:
            i += 1
    a = T["vehicle_attitude"]; ta = s(a["timestamp"]); r, p = eul(np.column_stack([a[f"q[{k}]"] for k in range(4)]))
    sp = T["vehicle_attitude_setpoint"]; tsp = s(sp["timestamp"]); rs, ps = eul(np.column_stack([sp[f"q_d[{k}]"] for k in range(4)]))
    ma = (ta >= t0) & (ta <= t1)
    terr = np.hypot(r[ma] - np.interp(ta[ma], tsp, rs), p[ma] - np.interp(ta[ma], tsp, ps))
    rt = T["vehicle_rates_setpoint"]; trt = s(rt["timestamp"]); av = T["vehicle_angular_velocity"]; tav = s(av["timestamp"])
    mw = (tav >= t0) & (tav <= t1)
    rerr = np.hypot(av["xyz[0]"][mw] - np.interp(tav[mw], trt, rt["roll"]), av["xyz[1]"][mw] - np.interp(tav[mw], trt, rt["pitch"]))
    va = T["vehicle_acceleration"]; tva = s(va["timestamp"]); mva = (tva >= t0) & (tva <= t1)
    axy = np.hypot(va["xyz[0]"][mva], va["xyz[1]"][mva])
    b = T["battery_status"]; tb = s(b["timestamp"]); mb = (tb >= t0) & (tb <= t1)
    n = U.initial_parameters["BAT1_N_CELLS"]
    mot = T["actuator_motors"]; tm = s(mot["timestamp"]); mm = (tm >= t0) & (tm <= t1)
    M = np.column_stack([mot[f"control[{k}]"] for k in range(4)])[mm]
    moving = m & (rvi > .05)
    return dict(
        label=label, cells=n, route_s=float(t1 - t0), ended_by_land=bool(land),
        v_cell=float(np.median(b["voltage_v"][mb]) / n), collective=float(np.median(-sp["thrust_body[2]"][(tsp >= t0) & (tsp <= t1)])),
        motor_peak=float(M.max()),
        speed_peak=float(spd[m].max()), speed_p95=float(np.percentile(spd[m], 95)),
        speed_over_ref_p95=float(np.percentile((spd - rvi)[moving], 95)),
        perr_p50=float(np.percentile(perr[m], 50)), perr_p95=float(np.percentile(perr[m], 95)), perr_max=float(perr[m].max()),
        tilt_err_rms=float(np.sqrt(np.mean(terr ** 2))),
        rate_err_rms=float(np.degrees(np.sqrt(np.mean(rerr ** 2)))),
        axy_max=float(axy.max()), axy_over5=int((axy > 5).sum()),
        stops=stops,
    )


if __name__ == "__main__":
    args = sys.argv[1:]
    rows = [analyse(args[i], args[i + 1]) for i in range(0, len(args), 2)]
    print("%-16s %2s %5s %5s %5s | %5s %5s %6s | %5s %5s %5s | %5s %5s | %5s | %s" % (
        "flight", "S", "V/c", "coll", "route", "vpk", "vp95", "v-ref95", "e50", "e95", "emax", "tilt", "rate", "axy", "stops: v@stop/overshoot cm"))
    for d in rows:
        print("%-16s %2d %5.2f %5.3f %5.1f | %5.3f %5.3f %6.3f | %5.3f %5.3f %5.3f | %5.2f %5.1f | %5.1f | %s" % (
            d["label"], d["cells"], d["v_cell"], d["collective"], d["route_s"], d["speed_peak"], d["speed_p95"], d["speed_over_ref_p95"],
            d["perr_p50"], d["perr_p95"], d["perr_max"], d["tilt_err_rms"], d["rate_err_rms"], d["axy_max"],
            " ".join("%.2f/%.1f(%.1fs)" % (x["v_at_ref_stop"], 100 * x["overshoot"], x["hold_s"]) for x in d["stops"])))
