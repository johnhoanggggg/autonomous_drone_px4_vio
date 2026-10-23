"""Side push along the route: the part of PX4's measured acceleration that its
position/velocity cascade does not explain from the recorded setpoints
(v_sp = ff + Kp (s - p); a = lag(Kv (v_sp - v), tau) + d), projected on the
route's right-hand side, binned by map x. Bag-only (no ULog).
Usage: python3 push.py <pkl> [<pkl> ...]
"""
import math
import pickle
import sys

import numpy as np

KP, KV, TAU = 0.95, 2.2, 0.15
bins = np.arange(0.0, 2.01, 0.1)
rows = {}
for pkl in sys.argv[1:]:
    D = pickle.load(open(pkl, 'rb'))
    st = D['/planner3d/flight/status']
    lp = np.array(D['/fmu/out/vehicle_local_position_v1'], dtype=float)
    sp = np.array(D['/fmu/in/trajectory_setpoint'], dtype=float)
    t0 = next(t for t, v in st if v.get('state') == 'ROUTE_START')
    t1 = max(t for t, v in st if v.get('state') in ('FOLLOWING', 'HOLD', 'YAW_ALIGN'))
    g = np.arange(t0, t1, 0.02)
    P = np.column_stack([np.interp(g, lp[:, 0], lp[:, k]) for k in (2, 3)])
    V = np.column_stack([np.interp(g, lp[:, 0], lp[:, k]) for k in (5, 6)])
    idx = np.searchsorted(sp[:, 0], g, side='right') - 1
    S = sp[idx][:, 2:4]
    FF = np.nan_to_num(sp[idx][:, 5:7])
    psi = np.array([s[-1] if np.isfinite(s[-1]) else np.nan for s in sp[idx]])
    psi = np.where(np.isfinite(psi), psi, np.interp(g, lp[:, 0], lp[:, 8]))
    k = 5
    A = np.gradient(np.column_stack([np.convolve(V[:, i], np.ones(k) / k, 'same') for i in (0, 1)]), 0.02, axis=0)
    am = np.zeros(2); d = np.zeros_like(A)
    for i in range(len(g)):
        vsp = FF[i] + KP * (S[i] - P[i])
        am += (KV * (vsp - V[i]) - am) * 0.02 / TAU
        d[i] = A[i] - am
    right = np.column_stack([-np.sin(psi), np.cos(psi)])
    ds = np.sum(d * right, axis=1)
    es = np.sum((S - P) * right, axis=1)  # + = setpoint to the right of the drone
    bx = np.array([next((v['body_map'][0] for tt, v in st if tt >= x and v.get('body_map')), np.nan) for x in g[::10]])
    bxg = np.interp(g, g[::10], bx)
    name = pkl.split('/')[-1][:6]
    rows[name] = (bxg, ds, es)
    print('%s side push rms %.2f m/s2' % (name, np.sqrt(np.mean(ds ** 2))))
print('\nmap x  | side push (m/s2, + = to the right), then drone left(+)/right(-) of setpoint (cm)')
print('        ' + '  '.join('%-15s' % n for n in rows))
for lo in bins[:-1]:
    cells = []
    for n, (bxg, ds, es) in rows.items():
        m = (bxg >= lo) & (bxg < lo + 0.1)
        cells.append('%+5.2f %+5.1f     ' % (ds[m].mean(), 100 * es[m].mean()) if m.sum() > 5 else '   -          ')
    print('%.1f-%.1f  ' % (lo, lo + 0.1) + '  '.join('%-15s' % c for c in cells))
