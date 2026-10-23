"""Route-phase timeline plot for a 3D flight: speed, command speed, errors, altitude, holds (bag time)."""
import pickle, sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from pyulog import ULog
out = sys.argv[1]; flights = [sys.argv[i:i+5] for i in range(2, len(sys.argv), 5)]
fig, axs = plt.subplots(4, len(flights), figsize=(9*len(flights), 12), squeeze=False)
for c, (label, pkl, ulg, b0, b1) in enumerate(flights):
    b0, b1 = float(b0), float(b1)
    D = pickle.load(open(pkl, 'rb')); U = ULog(ulg); T = {d.name: d.data for d in U.data_list if d.multi_id == 0}
    lpb = np.array([(r[0], r[1]) for r in D['/fmu/out/vehicle_local_position_v1']], float)
    off = np.median(lpb[:, 0] - (lpb[:, 1] - U.start_timestamp)/1e6)
    s = lambda a: (a - U.start_timestamp)/1e6 + off
    ts = T['trajectory_setpoint']; ok = ts['timestamp'] >= U.start_timestamp; tt = s(ts['timestamp'][ok])
    P = np.column_stack([ts[f'position[{k}]'][ok] for k in range(3)]); V = np.column_stack([ts[f'velocity[{k}]'][ok] for k in range(3)])
    lp = T['vehicle_local_position']; tl = s(lp['timestamp']); pos = np.column_stack([lp['x'], lp['y'], lp['z']])
    fin = np.isfinite(P).all(1); Pi = np.column_stack([np.interp(tl, tt[fin], P[fin, k]) for k in range(3)])
    hs = np.hypot(lp['vx'], lp['vy']); ffs = np.hypot(V[:, 0], V[:, 1])
    lps = T['vehicle_local_position_setpoint']; tlp = s(lps['timestamp'])
    ax = axs[0, c]; ax.plot(tl, hs, label='speed'); ax.plot(tt, ffs, label='vel ff (cmd)'); ax.plot(tlp, np.hypot(lps['vx'], lps['vy']), lw=.6, label='PX4 vel sp')
    ax.plot(tl, -lp['vz'], lw=.6, label='climb rate'); ax.axhline(.15, ls=':', c='k'); ax.set_ylabel('m/s'); ax.legend(fontsize=7); ax.set_title(label)
    ax = axs[1, c]; e = pos - Pi; ax.plot(tl, np.hypot(e[:, 0], e[:, 1]), label='|xy err|'); ax.plot(tl, -e[:, 2], label='z err (+ = high)'); ax.set_ylabel('m'); ax.legend(fontsize=7)
    ax = axs[2, c]; ax.plot(tl, -lp['z'], label='alt'); ax.plot(tl, -Pi[:, 2], label='alt sp'); ax.legend(fontsize=7); ax.set_ylabel('m')
    ax = axs[3, c]; ax.plot(tlp, np.hypot(lps['acceleration[0]'], lps['acceleration[1]']), lw=.6, label='|acc sp|')
    if np.isfinite(ts['acceleration[0]']).any():
        ax.plot(tt, np.hypot(ts['acceleration[0]'][ok], ts['acceleration[1]'][ok]), lw=.6, label='acc ff')
    ax.legend(fontsize=7); ax.set_ylabel('m/s2')
    st = D['/planner3d/flight/status']
    for i, (t, v) in enumerate(st[:-1]):
        if v['state'] != 'FOLLOWING' and b0-3 <= t <= b1+2:
            col = {'HOLD': 'r', 'YAW_ALIGN': 'y'}.get(v['state'], 'grey')
            for a in axs[:, c]: a.axvspan(t, st[i+1][0], color=col, alpha=.15, lw=0)
    for a in axs[:, c]: a.set_xlim(b0-2, b1+1); a.grid(alpha=.3)
plt.tight_layout(); plt.savefig(out, dpi=70)
