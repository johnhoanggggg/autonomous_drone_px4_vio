"""Route-following decomposition for a 3D-planner flight (bag pickle from dump3d.py).

Offsets are measured against the FIRST planned route in a frame frozen at route start
(PX4 estimate re-expressed in the map frame; no map corrections), split into lateral
(horizontal, + = left of route) and vertical (+ = above route) components.
"""
import pickle
import sys

import numpy as np


def rot(q, v):
    x, y, z, w = q
    t = 2 * np.cross([x, y, z], v)
    return v + w * t + np.cross([x, y, z], t)


def nearest(route, p):
    """Closest point on the 3D polyline (by horizontal distance): (lateral signed, vertical, along, segment dir)."""
    best = None
    acc = 0.0
    for a, b in zip(route[:-1], route[1:]):
        d = b - a
        L2 = float(d[:2] @ d[:2])
        L = float(np.linalg.norm(d))
        t = 0.0 if L2 < 1e-12 else float(np.clip((p[:2] - a[:2]) @ d[:2] / L2, 0, 1))
        c = a + t * d
        h = float(np.hypot(*(p[:2] - c[:2])))
        if best is None or h < best[0]:
            side = np.sign(d[0] * (p[1] - a[1]) - d[1] * (p[0] - a[0])) or 1.0
            best = (h, side * h, p[2] - c[2], acc + t * L, d / max(L, 1e-9))
        acc += L
    return best[1:]


def load(pkl, b0=None, b1=None):
    D = pickle.load(open(pkl, 'rb'))
    st = D['/planner3d/flight/status']
    if b0 is None:
        b0 = next(t for t, v in st if v.get('state') == 'ROUTE_START')
    if b1 is None:
        b1 = max(t for t, v in st if v.get('state') not in ('LAND', 'DONE'))
    route = np.array(next(p for t, p in D['/planner3d/reference_path'] if t >= b0 and len(p) >= 2))
    t0, v0 = next((t, v) for t, v in st if t >= b0 and v.get('command_origin_ned') and v.get('map_from_odom'))
    body0, act0, qc = np.array(v0['body_map']), np.array(v0['command_origin_ned']), v0['map_from_odom'][3:]

    def to_map(n):
        return body0 + rot(qc, np.array([n[1] - act0[1], n[0] - act0[0], -(n[2] - act0[2])]))
    lp = [(r[0], to_map(r[2:5])) for r in D['/fmu/out/vehicle_local_position_v1'] if b0 <= r[0] <= b1]
    sp = [(r[0], to_map(r[2:5])) for r in D['/fmu/in/trajectory_setpoint'] if b0 <= r[0] <= b1 and np.isfinite(r[2])]
    off = lambda rows: np.array([(t, *nearest(route, p)[:3]) for t, p in rows])
    return D, route, off(lp), off(sp), b0, b1


if __name__ == '__main__':
    q = lambda x: 'p50 %5.1f p95 %5.1f max %5.1f cm' % (100 * np.percentile(np.abs(x), 50), 100 * np.percentile(np.abs(x), 95), 100 * np.max(np.abs(x)))
    for pkl in sys.argv[1:]:
        D, route, P, S, b0, b1 = load(pkl)
        print('== %s  route %.1f-%.1f s, first route length %.2f m, rise %.2f m' % (pkl, b0, b1, np.sum(np.linalg.norm(np.diff(route, axis=0), axis=1)), route[:, 2].max() - route[0, 2]))
        print('  drone    lateral', q(P[:, 1]), '| vertical', q(P[:, 2]), '| mean vertical %+.1f cm' % (100 * P[:, 2].mean()))
        print('  setpoint lateral', q(S[:, 1]), '| vertical', q(S[:, 2]), '| mean vertical %+.1f cm' % (100 * S[:, 2].mean()))
