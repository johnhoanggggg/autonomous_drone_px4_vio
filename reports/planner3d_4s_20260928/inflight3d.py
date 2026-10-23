"""In-flight (route-phase) metrics for 3D planner flights: bag states + ULog control.

Usage: .venv-mavlink/bin/python reports/planner3d_4s_20260928/inflight3d.py <label> <bag.pkl> <ulog> <t0_bag> <t1_bag> [...]
The pickle is written by dump3d.py (flight status + setpoints with PX4 timestamps).
"""
import json
import pickle
import sys
from collections import Counter

import numpy as np
from pyulog import ULog


def eul(q):
    r = np.degrees(np.arctan2(2*(q[:, 0]*q[:, 1]+q[:, 2]*q[:, 3]), 1-2*(q[:, 1]**2+q[:, 2]**2)))
    p = np.degrees(np.arcsin(np.clip(2*(q[:, 0]*q[:, 2]-q[:, 3]*q[:, 1]), -1, 1)))
    y = np.arctan2(2*(q[:, 0]*q[:, 3]+q[:, 1]*q[:, 2]), 1-2*(q[:, 2]**2+q[:, 3]**2))
    return r, p, y


def lag(t, x, y, maxlag=0.6, dt=0.01):
    g = np.arange(t[0], t[-1], dt); a = np.interp(g, t, x); b = np.interp(g, t, y)
    a -= a.mean(); b -= b.mean(); best = (0, -2)
    for k in range(int(maxlag/dt)):
        c = np.corrcoef(a[:len(a)-k], b[k:])[0, 1]
        if c > best[1]: best = (k*dt, c)
    return best


def runs(mask, t, min_s):
    out = []; i = 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]: j += 1
            if t[min(j, len(t)-1)] - t[i] >= min_s: out.append((t[i], t[min(j, len(t)-1)]))
            i = j
        else: i += 1
    return out


def analyse(label, pkl, ulg, b0, b1):
    D = pickle.load(open(pkl, 'rb'))
    U = ULog(ulg); T = {d.name: d.data for d in U.data_list if d.multi_id == 0}
    s = lambda a: (a - U.start_timestamp)/1e6
    sp_b = np.array([(r[0], r[1]) for r in D['/fmu/out/vehicle_local_position_v1']], float)
    off = np.median(sp_b[:, 0] - (sp_b[:, 1] - U.start_timestamp)/1e6)  # bag t = ulog t + off
    t0, t1 = b0-off, b1-off
    # bag states in route window
    st = [(t, v['state'], v.get('fault') or '') for t, v in D['/planner3d/flight/status'] if b0 <= t <= b1]
    dt_st = np.diff([x[0] for x in st] + [b1])
    hold = Counter()
    for (t, s_, f), d in zip(st, dt_st):
        if s_ != 'FOLLOWING': hold[s_ + ('/'+f if f else '')] += d
    ts = T['trajectory_setpoint']; okts = ts['timestamp'] >= U.start_timestamp
    ts = {k: v[okts] for k, v in ts.items()}; tt = s(ts['timestamp'])
    P = np.column_stack([ts[f'position[{k}]'] for k in range(3)]); V = np.column_stack([ts[f'velocity[{k}]'] for k in range(3)])
    lp = T['vehicle_local_position']; tl = s(lp['timestamp']); m = (tl >= t0) & (tl <= t1)
    pos = np.column_stack([lp['x'], lp['y'], lp['z']]); vel = np.column_stack([lp['vx'], lp['vy'], lp['vz']])
    fin = np.isfinite(P).all(1)
    Pi = np.column_stack([np.interp(tl, tt[fin], P[fin, k]) for k in range(3)])
    e = pos - Pi; eh = np.hypot(e[:, 0], e[:, 1])
    # along/cross from the setpoint's own motion (smoothed 0.5 s)
    dP = np.gradient(Pi[:, :2], tl, axis=0); k = 25
    dP = np.column_stack([np.convolve(dP[:, i], np.ones(k)/k, 'same') for i in range(2)])
    sp_spd = np.hypot(dP[:, 0], dP[:, 1]); mv = m & (sp_spd > 0.05)
    u = dP/np.maximum(sp_spd, 1e-6)[:, None]
    along = -(e[:, 0]*u[:, 0] + e[:, 1]*u[:, 1])  # + = drone behind command
    cross = e[:, 0]*(-u[:, 1]) + e[:, 1]*u[:, 0]
    hs = np.hypot(vel[:, 0], vel[:, 1])
    # feedforward usage
    vff = np.isfinite(V[:, 0]) & (tt >= t0) & (tt <= t1)
    # attitude
    a = T['vehicle_attitude']; ta = s(a['timestamp']); r, p, y = eul(np.column_stack([a[f'q[{i}]'] for i in range(4)]))
    sp = T['vehicle_attitude_setpoint']; tsp = s(sp['timestamp']); rs, ps, ys = eul(np.column_stack([sp[f'q_d[{i}]'] for i in range(4)]))
    ma = (ta >= t0) & (ta <= t1); msp = (tsp >= t0) & (tsp <= t1)
    rsi = np.interp(ta, tsp, rs); psi = np.interp(ta, tsp, ps)
    terr = np.hypot(r - rsi, p - psi)[ma]
    rl = lag(ta[ma], rsi[ma], r[ma]); pl = lag(ta[ma], psi[ma], p[ma])
    yerr = np.degrees(np.angle(np.exp(1j*(y - np.interp(ta, tsp, np.unwrap(ys))))))[ma]
    yawsp = ts['yaw']; ym = np.isfinite(yawsp) & (tt >= t0) & (tt <= t1)
    yaw_rate_cmd = np.degrees(np.abs(np.diff(np.unwrap(yawsp[ym])))/np.maximum(np.diff(tt[ym]), 1e-3)) if ym.sum() > 2 else np.array([0])
    av = T['vehicle_angular_velocity']; tav = s(av['timestamp']); mw = (tav >= t0) & (tav <= t1)
    yaw_rate = np.degrees(np.abs(av['xyz[2]'][mw]))
    # demand in body frame (push) from local position setpoint acceleration
    lps = T['vehicle_local_position_setpoint']; tlp = s(lps['timestamp']); ml = (tlp >= t0) & (tlp <= t1)
    ax, ay = lps['acceleration[0]'], lps['acceleration[1]']
    yl = np.interp(tlp, ta, np.unwrap(y))
    fwd = ax*np.cos(yl) + ay*np.sin(yl); right = -ax*np.sin(yl) + ay*np.cos(yl)
    acc_h = np.hypot(ax, ay)
    # stationary under demand (snag signature)
    hs_l = np.interp(tlp, tl, hs)
    snag = runs(ml & (hs_l < 0.03) & (acc_h > 0.2), tlp, 0.7)
    va = T['vehicle_acceleration']; tva = s(va['timestamp']); mva = (tva >= t0) & (tva <= t1)
    axy = np.hypot(va['xyz[0]'][mva], va['xyz[1]'][mva])
    # stop-start: horizontal speed dips < 0.04 for >= 0.3 s while FOLLOWING (bag)
    fol = np.zeros(len(tl), bool)
    for (tb, s_, f), d in zip(st, dt_st):
        if s_ == 'FOLLOWING': fol |= (tl >= tb-off) & (tl < tb-off+d)
    dips = runs(fol & (hs < 0.04), tl, 0.3)
    b = T['battery_status']; tb_ = s(b['timestamp']); mb = (tb_ >= t0) & (tb_ <= t1); n = U.initial_parameters['BAT1_N_CELLS']
    mot = T['actuator_motors']; tm = s(mot['timestamp']); mm = (tm >= t0) & (tm <= t1)
    M = np.column_stack([mot[f'control[{i}]'] for i in range(4)])[mm]
    thr = -T['vehicle_thrust_setpoint']['xyz[2]'][(s(T['vehicle_thrust_setpoint']['timestamp']) >= t0) & (s(T['vehicle_thrust_setpoint']['timestamp']) <= t1)]
    temp = None
    if 'vehicle_imu_status' in T:
        im = T['vehicle_imu_status']; ti = s(im['timestamp']); mi = (ti >= t0) & (ti <= t1)
        temp = [float(im['temperature_accel'][mi].min()), float(im['temperature_accel'][mi].max())]
    info = [v for t, v in D['/rtabmap/vio_odom_info_lite'] if b0 <= t <= b1 and isinstance(v, dict)]
    inl = [v.get('inliers') for v in info if v.get('inliers') is not None]
    q = lambda x, pc: float(np.percentile(x, pc))
    return dict(
        label=label, route_s=round(t1-t0, 1), off=round(off, 2), cells=n,
        v_cell=round(float(np.median(b['voltage_v'][mb]))/n, 3), thrust_med=round(float(np.median(thr)), 3), motor_peak=round(float(M.max()), 3),
        imu_temp=temp,
        hold_s={k: round(v, 1) for k, v in hold.most_common()}, hold_total=round(sum(hold.values()), 1),
        ff_frac=round(float(vff.sum()/max(1, ((tt >= t0) & (tt <= t1)).sum())), 2),
        eh=[q(eh[m], 50), q(eh[m], 95), float(eh[m].max())],
        ez=[q(np.abs(e[m, 2]), 50), q(np.abs(e[m, 2]), 95), float(np.abs(e[m, 2]).max())],
        along_mv=[q(along[mv], 50), q(along[mv], 95)], cross_mv=[q(np.abs(cross[mv]), 50), q(np.abs(cross[mv]), 95), float(np.mean(cross[mv]))],
        spd=[q(hs[m], 50), q(hs[m], 95), float(hs[m].max())], cmd_spd_p95=q(sp_spd[m], 95), vz_max=float(np.abs(vel[m, 2]).max()),
        dips=len(dips), dips_per_m=None,
        path_len=float(np.sum(np.hypot(*np.diff(pos[m, :2], axis=0).T))),
        tilt_rms=float(np.sqrt(np.mean(terr**2))), roll_lag=rl, pitch_lag=pl,
        yaw_err=[q(np.abs(yerr), 50), q(np.abs(yerr), 95)], yaw_rate_cmd_p95=q(yaw_rate_cmd, 95), yaw_rate_p95=q(yaw_rate, 95),
        push_fwd=float(np.mean(fwd[ml])), push_right=float(np.mean(right[ml])), acc_dem_p95=q(acc_h[ml], 95),
        acc_dem_sat=float(np.mean(acc_h[ml] > 0.5)),
        snags=[(round(a_+off, 1), round(b_-a_, 1)) for a_, b_ in snag],
        axy_max=float(axy.max()), axy_over3=int((axy > 3).sum()), axy_over5=int((axy > 5).sum()),
        inliers=[int(np.min(inl)), int(np.median(inl))] if inl else None,
    )


if __name__ == '__main__':
    a = sys.argv[1:]; rows = []
    for i in range(0, len(a), 5):
        d = analyse(a[i], a[i+1], a[i+2], float(a[i+3]), float(a[i+4])); d['dips_per_m'] = round(d['dips']/max(d['path_len'], 0.1), 1); rows.append(d)
        print(json.dumps(d, default=lambda o: round(float(o), 3)))
