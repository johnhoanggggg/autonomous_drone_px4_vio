"""What-if PX4 xy gains under the SAME backed-out push, same setpoints.
Re-simulates v_sp = ff + Kp (s - p); a = lag(Kv (v_sp - v)) + d with the push d(t)
from push.py's model; reports the side offset from the setpoint in the gap
(map x 0.2-0.9) and over the whole route. Baseline reproduces the flight.
Usage: python3 gains_whatif.py <pkl> [...]
"""
import pickle
import sys

import numpy as np

KP0, KV0, TAU = 0.95, 2.2, 0.15


def extract(pkl):
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
    S, FF = sp[idx][:, 2:4], np.nan_to_num(sp[idx][:, 5:7])
    psi = np.array([s[-1] for s in sp[idx]])
    psi = np.where(np.isfinite(psi), psi, np.interp(g, lp[:, 0], lp[:, 8]))
    A = np.gradient(np.column_stack([np.convolve(V[:, i], np.ones(5) / 5, 'same') for i in (0, 1)]), 0.02, axis=0)
    am = np.zeros(2); d = np.zeros_like(A)
    for i in range(len(g)):
        am += (KV0 * (FF[i] + KP0 * (S[i] - P[i]) - V[i]) - am) * 0.02 / TAU
        d[i] = A[i] - am
    bx = [next((v['body_map'][0] for tt, v in st if tt >= x and v.get('body_map')), np.nan) for x in g[::10]]
    return g, P, V, S, FF, d, psi, np.interp(g, g[::10], bx)


def sim(P0, V0, S, FF, d, kp, kv):
    p, v, a = P0.copy(), V0.copy(), np.zeros(2); out = np.zeros_like(S)
    for i in range(len(S)):
        out[i] = p
        a += (kv * (FF[i] + kp * (S[i] - p) - v) - a) * 0.02 / TAU
        v = v + (a + d[i]) * 0.02; p = p + v * 0.02
    return out


cases = [(0.95, 2.2), (1.2, 2.2), (1.5, 2.2), (0.95, 3.0), (1.2, 3.0), (1.5, 3.0)]
res = {c: [] for c in cases}; resall = {c: [] for c in cases}
for pkl in sys.argv[1:]:
    g, P, V, S, FF, d, psi, bx = extract(pkl)
    right = np.column_stack([-np.sin(psi), np.cos(psi)])
    gap = (bx > 0.2) & (bx < 0.9)
    for c in cases:
        Q = sim(P[0], V[0], S, FF, d, *c)
        side = np.abs(np.sum((Q - S) * right, axis=1))
        res[c].append(side[gap]); resall[c].append(side)
print('xy gains (MPC_XY_P, MPC_XY_VEL_P_ACC) | side offset from setpoint: gap p95 / max | whole route p95')
for c in cases:
    a, b = np.concatenate(res[c]), np.concatenate(resall[c])
    print('  P %.2f  VEL_P %.1f  | %5.1f / %5.1f cm | %5.1f cm %s' % (c[0], c[1], 100 * np.percentile(a, 95), 100 * a.max(), 100 * np.percentile(b, 95), ' (as flown)' if c == cases[0] else ''))
