"""Gap-entry swing in PX4's own frame: setpoint vs position (side of the route
heading), VIO/map pose error (body_map vs PX4 position re-expressed in the map),
heading, climb. Usage: python3 px4_side.py <pkl> <t0> <t1>
"""
import math
import pickle
import sys

import numpy as np

pkl, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
D = pickle.load(open(pkl, 'rb'))
st = D['/planner3d/flight/status']
lp = np.array([r for r in D['/fmu/out/vehicle_local_position_v1']], dtype=float)
sp = np.array([r for r in D['/fmu/in/trajectory_setpoint']], dtype=float)


def rot(q, v):
    x, y, z, w = q
    t = 2 * np.cross([x, y, z], v)
    return v + w * t + np.cross([x, y, z], t)


# route heading in NED from the first setpoint motion: use map->NED via status command pairs
print('   t    side_err  along_err  sp_vside  vside  hdg   z_ned  pose_err_side  pose_err_jump')
prev = None
for t, v in st:
    if not (a <= t <= b) or not v.get('command_ned') or not v.get('body_map'):
        continue
    L = lp[lp[:, 0] <= t][-1]
    S = sp[sp[:, 0] <= t][-1]
    # route direction ~ NED heading of travel; use yaw setpoint as route heading
    psi = S[-1] if np.isfinite(S[-1]) else L[8]
    f = np.array([math.cos(psi), math.sin(psi)]); s = np.array([-math.sin(psi), math.cos(psi)])  # right-hand side in NED
    e = S[2:4] - L[2:4] if np.isfinite(S[2]) else np.array([np.nan, np.nan])
    vel = L[5:7]
    spv = S[5:7] if np.isfinite(S[5]) else np.zeros(2)
    # pose error: body_map vs PX4 NED position mapped through the adapter's own pairing
    q = v['map_from_odom'][3:] if v.get('map_from_odom') else [0, 0, 0, 1]
    cn = np.array(v['command_ned']); cm = np.array(v['command_map'] or v['body_map'])
    # map->NED delta of (command - body) vs NED (command - position)
    dm = cm - np.array(v['body_map'])
    dn = cn - L[2:5]
    # express the map delta in NED (ENU->NED swap) ignoring small tilt
    dmn = np.array([dm[1], dm[0], -dm[2]])
    pe = dn - dmn  # nonzero = PX4 position disagrees with the map body by this much (relative to the command pairing)
    print('%5.1f  %7.1f  %8.1f  %8.2f  %5.2f  %4.0f  %6.2f  %8.1f' % (
        t, 100 * (e @ s), 100 * (e @ f), spv @ s, vel @ s, math.degrees(L[8]), -L[4], 100 * (pe[:2] @ s)))
