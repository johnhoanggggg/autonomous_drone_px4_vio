"""ULog-only lateral-stop leg metrics (onboard reference + onboard estimate)."""
import json
import sys

import numpy as np
from pyulog import ULog


def topic(ulog, name, multi=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi:
            return d.data
    return None


def interp(t, tt, v):
    return np.interp(t, tt, v)


def hold_ok(t, a, b):
    return np.any((t >= a) & (t <= b)) and b - a > 0.2


def analyse(path):
    ulog = ULog(path)
    tsp = topic(ulog, "trajectory_setpoint")
    lp = topic(ulog, "vehicle_local_position")
    att = topic(ulog, "vehicle_attitude")
    atts = topic(ulog, "vehicle_attitude_setpoint")
    rsp = topic(ulog, "vehicle_rates_setpoint")
    angv = topic(ulog, "vehicle_angular_velocity")
    bat = topic(ulog, "battery_status")
    hte = topic(ulog, "hover_thrust_estimate")
    mot = topic(ulog, "actuator_motors")
    cmd = topic(ulog, "vehicle_command")
    T0 = ulog.start_timestamp
    s = lambda a: (a - T0) / 1e6

    tt = s(tsp["timestamp"])
    rp = np.column_stack([tsp["position[0]"], tsp["position[1]"]])
    rv = np.column_stack([tsp["velocity[0]"], tsp["velocity[1]"]])
    ra = np.column_stack([tsp["acceleration[0]"], tsp["acceleration[1]"]])
    ok = np.isfinite(rp).all(1)
    rvn = np.nan_to_num(np.linalg.norm(rv, axis=1))
    moving = ok & (rvn > 1e-4)
    # contiguous move segments
    idx = np.flatnonzero(moving)
    segs = []
    if len(idx):
        start = idx[0]
        for a, b in zip(idx[:-1], idx[1:]):
            if b != a + 1:
                segs.append((start, a)); start = b
        segs.append((start, idx[-1]))
    segs = [(a, b) for a, b in segs if tt[b] - tt[a] > 1.5]
    if not segs:
        return dict(path=path, legs=[])
    origin = rp[segs[0][0] - 1]
    first_end = rp[segs[0][1] + 1]
    u = (first_end - origin) / np.linalg.norm(first_end - origin)
    fwd = np.array([-u[1], u[0]])

    t = s(lp["timestamp"])
    pos = np.column_stack([lp["x"], lp["y"]]) - origin
    vel = np.column_stack([lp["vx"], lp["vy"]])
    lat = pos @ u; lon = pos @ fwd
    vlat = vel @ u
    speed = np.linalg.norm(vel, axis=1)

    land = [s(ts) for ts, c in zip(cmd["timestamp"], cmd["command"]) if int(c) == 21] if cmd is not None else []
    land_t = min(land) if land else t[-1]

    # hover/attitude channels (roll/pitch mapped to lateral via body ~ heading)
    q = np.column_stack([att[f"q[{i}]"] for i in range(4)])
    ta = s(att["timestamp"])
    roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
    qd = np.column_stack([atts[f"q_d[{i}]"] for i in range(4)])
    tas = s(atts["timestamp"])
    roll_sp = np.degrees(np.arctan2(2 * (qd[:, 0] * qd[:, 1] + qd[:, 2] * qd[:, 3]), 1 - 2 * (qd[:, 1] ** 2 + qd[:, 2] ** 2)))
    pitch_sp = np.degrees(np.arcsin(np.clip(2 * (qd[:, 0] * qd[:, 2] - qd[:, 3] * qd[:, 1]), -1, 1)))
    thrust_z = -atts["thrust_body[2]"]
    trs = s(rsp["timestamp"]); tav = s(angv["timestamp"])

    legs = []
    targets = []
    for k, (a, b) in enumerate(segs):
        target = float(((rp[b + 1] if b + 1 < len(rp) and ok[b + 1] else rp[b]) - origin) @ u)
        start_prev = float((rp[a - 1] - origin) @ u)
        sign = 1.0 if target > start_prev else -1.0
        t_start, t_stop = tt[a], tt[b]
        # hold end: next move start or land command
        t_hold_end = tt[segs[k + 1][0]] if k + 1 < len(segs) else land_t
        # ref brake start: last time ref speed is at its max
        seg = slice(a, b + 1)
        vmax = rvn[seg].max()
        brake_i = a + int(np.flatnonzero(rvn[seg] >= vmax - 1e-3)[-1])
        t_brake = tt[brake_i]
        sv = lambda tq: float(interp(tq, t, vlat) * sign)
        move = (t >= t_start) & (t <= t_hold_end)
        after = (t >= t_brake) & (t <= t_hold_end)
        if not hold_ok(t, t_stop, t_hold_end):
            continue
        hold = (t >= t_stop) & (t <= t_hold_end)
        first3 = hold & (t < t_stop + 3)
        # first reversal after ref stop
        sv_after = sign * vlat[(t >= t_brake) & (t <= t_hold_end)]
        ta_after = t[(t >= t_brake) & (t <= t_hold_end)]
        rev = np.flatnonzero(sv_after <= 0)
        t_rev = float(ta_after[rev[0]]) if len(rev) else None
        # hold ref acceleration (damping detector)
        hw = (tt >= t_stop + .1) & (tt <= t_hold_end - .1)
        hold_acc = np.linalg.norm(np.nan_to_num(ra[hw]), axis=1)
        # tilt tracking in the brake+first 3s window
        w0, w1 = t_brake, min(t_hold_end, t_stop + 3)
        wa = (ta >= w0) & (ta <= w1)
        rs_i = interp(ta[wa], tas, roll_sp); ps_i = interp(ta[wa], tas, pitch_sp)
        tilt_err = np.hypot(roll[wa] - rs_i, pitch[wa] - ps_i)
        # rate tracking magnitude ratio in that window
        wr = (tav >= w0) & (tav <= w1)
        rx = interp(tav[wr], trs, rsp["roll"]); ry = interp(tav[wr], trs, rsp["pitch"])
        sp_mag = np.hypot(rx, ry); act_mag = np.hypot(angv["xyz[0]"][wr], angv["xyz[1]"][wr])
        err_mag = np.hypot(angv["xyz[0]"][wr] - rx, angv["xyz[1]"][wr] - ry)
        legs.append(dict(
            leg=k, direction=sign, target=target,
            t_start=float(t_start), t_brake=float(t_brake), t_ref_stop=float(t_stop), t_hold_end=float(t_hold_end),
            ref_peak=float(vmax),
            meas_peak=float((sign * vlat[move]).max()),
            v_at_brake=sv(t_brake), v_at_ref_stop=sv(t_stop), v_1s_after=sv(t_stop + 1),
            overshoot=float(max(0., (sign * (lat[after] - target)).max())),
            stop_travel_after_brake=float((sign * (lat[after] - interp(t_brake, t, lat))).max()),
            reversal_after_brake_s=(t_rev - t_brake) if t_rev else None,
            hold_s=float(t_hold_end - t_stop),
            hold_span=float(np.ptp(lat[hold])) if hold.any() else None,
            first3_span=float(np.ptp(lat[first3])) if first3.any() else None,
            first3_vlat_rms=float(np.sqrt(np.mean(vlat[first3] ** 2))) if first3.any() else None,
            hold_max_poserr=float(np.max(np.hypot(lat[hold] - target, lon[hold]))) if hold.any() else None,
            hold_speed_gate_frac=float(np.mean(speed[hold] > .06)) if hold.any() else None,
            hold_ref_acc_max=float(hold_acc.max()) if len(hold_acc) else 0.,
            tilt_err_rms_deg=float(np.sqrt(np.mean(tilt_err ** 2))),
            rate_sp_rms_dps=float(np.degrees(np.sqrt(np.mean(sp_mag ** 2)))),
            rate_act_rms_dps=float(np.degrees(np.sqrt(np.mean(act_mag ** 2)))),
            rate_err_rms_dps=float(np.degrees(np.sqrt(np.mean(err_mag ** 2)))),
        ))
    # whole-flight channels after takeoff
    tb = s(bat["timestamp"]); flying = (tb > tt[segs[0][0]]) & (tb < land_t)
    th = s(hte["timestamp"]) if hte is not None else None
    tm = s(mot["timestamp"])
    fm = (tm > tt[segs[0][0]]) & (tm < land_t)
    motors = np.column_stack([mot[f"control[{i}]"] for i in range(4)])[fm]
    flying_att = (tas > tt[segs[0][0]]) & (tas < land_t)
    return dict(
        path=path,
        n_cells=ulog.initial_parameters.get("BAT1_N_CELLS"),
        params={k: ulog.initial_parameters.get(k) for k in (
            "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLL_P", "MC_PITCHRATE_P",
            "MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC", "MPC_THR_HOVER",
            "MPC_XY_VEL_MAX", "MPC_ACC_HOR_MAX", "MC_BAT_SCALE_EN", "SENS_BOARD_X_OFF")},
        damping=bool(max((l["hold_ref_acc_max"] for l in legs), default=0) > 1e-4),
        land_t=float(land_t), legs=legs,
        v_loaded_median=float(np.median(bat["voltage_v"][flying])),
        v_cell_loaded_median=float(np.median(bat["voltage_v"][flying]) / ulog.initial_parameters.get("BAT1_N_CELLS")),
        v_min=float(bat["voltage_v"][flying].min()),
        i_median=float(np.median(bat["current_a"][flying])),
        hover_thrust_est=float(np.median(hte["hover_thrust"][(th > tt[segs[0][0]]) & (th < land_t)])) if hte is not None else None,
        collective_median=float(np.median(thrust_z[flying_att])),
        motor_max=float(motors.max()), motor_min=float(motors.min()),
        motor_mean=float(motors.mean()),
    )


if __name__ == "__main__":
    import traceback
    out = []
    for p in sys.argv[1:]:
        try:
            out.append(analyse(p))
        except Exception:
            print(p, traceback.format_exc().splitlines()[-4:], file=sys.stderr)
    print(json.dumps(out, indent=1))
