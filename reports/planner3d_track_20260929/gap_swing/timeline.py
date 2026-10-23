"""Per-tick timeline of the gap-entry stretch (body map x in [X0, X1]) of 3D flights.

Usage: python3 timeline.py <pkl> [X0 X1]
Columns: route side of drone/command (+ = left, vs the route 1 s old), drone-command
gap (side), PX4 heading vs yaw setpoint, route heading, map correction yaw jump,
VIO inliers, drone speed.
"""
import math
import pickle
import sys

import numpy as np


def cross(route, p):
    best = None
    for a, b in zip(route[:-1], route[1:]):
        d = b - a
        L2 = float(d @ d)
        t = 0.0 if L2 < 1e-12 else float(np.clip((p - a) @ d / L2, 0, 1))
        c = a + t * d
        e = p - c
        dh = d[:2] / max(np.linalg.norm(d[:2]), 1e-9)
        n = float(np.linalg.norm(e))
        if best is None or n < best[0]:
            best = (n, float(dh[0] * e[1] - dh[1] * e[0]), math.degrees(math.atan2(d[1], d[0])), math.degrees(math.atan2(d[2], np.linalg.norm(d[:2]))))
    return best


def last(rows, t):
    r = None
    for x in rows:
        if x[0] > t:
            break
        r = x
    return r


pkl = sys.argv[1]
X0, X1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0.1, 1.1)
D = pickle.load(open(pkl, 'rb'))
st = D['/planner3d/flight/status']
rp = [(t, np.array(p)) for t, p in D['/planner3d/reference_path'] if len(p) >= 2]
lp, sp, info = D['/fmu/out/vehicle_local_position_v1'], D['/fmu/in/trajectory_setpoint'], D['/rtabmap/vio_odom_info_lite']
print('   t  state      fault                     bx    by    bz  |dside cside  d-c | hdg  yawsp  rte climb | mfo_yaw inl  spd')
prev_q = None
for t, v in st:
    b = v.get('body_map')
    if not b or not (X0 <= b[0] <= X1) or v.get('state') in ('LAND', 'DONE'):
        continue
    old = [p for tp, p in rp if tp <= t - 1.0]
    c = v.get('command_map')
    ds = cross(old[-1], np.array(b)) if old else None
    cs = cross(old[-1], np.array(c)) if old and c else None
    L = last(lp, t)
    S = last(sp, t)
    I = last(info, t)
    q = v.get('map_from_odom')
    myaw = math.degrees(2 * math.atan2(q[5], q[6])) if q else float('nan')
    print('%5.1f %-10s %-24s %5.2f %5.2f %5.2f |%5.1f %5.1f %5.1f |%5.0f %5.0f %5.0f %4.0f | %6.2f %4d %4.2f' % (
        t, v.get('state')[:10], (v.get('fault') or '')[:24], b[0], b[1], b[2],
        100 * ds[1] if ds else float('nan'), 100 * cs[1] if cs else float('nan'),
        100 * (ds[1] - cs[1]) if ds and cs else float('nan'),
        math.degrees(L[8]) if L else float('nan'), math.degrees(S[-1]) if S is not None and np.isfinite(S[-1]) else float('nan'),
        ds[2] if ds else float('nan'), ds[3] if ds else float('nan'), myaw,
        I[1]['inliers'] if I else -1, math.hypot(L[5], L[6]) if L else float('nan')))
