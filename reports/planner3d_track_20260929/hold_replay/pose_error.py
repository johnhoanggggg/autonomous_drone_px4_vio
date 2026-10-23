"""Recorded map-pose error for hold_sim: body_map minus PX4's position (frozen frame).

Usage: python3 pose_error.py <dump.pkl> <replay_dir>   (dump.pkl from ../dump3d.py)
The adapter builds PX4's target as PX4 position + (command - body_map), so this
error (pose lag, VIO jitter, map-correction jumps) is injected into every command.
"""
import json
import sys

import numpy as np
import pickle


def rot(q, v):
    x, y, z, w = q
    t = 2 * np.cross([x, y, z], v)
    return v + w * t + np.cross([x, y, z], t)


D = pickle.load(open(sys.argv[1], 'rb'))
meta = json.load(open(sys.argv[2] + '/meta.json'))
st = D['/planner3d/flight/status']
route_t = meta['route_t']
t_fix, v0 = next((t, v) for t, v in st if t >= route_t and v.get('command_origin_ned') and v.get('map_from_odom'))
body0, act0, qc = np.array(v0['body_map']), np.array(v0['command_origin_ned']), v0['map_from_odom'][3:]
pos = lambda n: body0 + rot(qc, np.array([n[1] - act0[1], n[0] - act0[0], -(n[2] - act0[2])]))
rows = [(t, *(np.array(v['body_map']) - pos(v['command_origin_ned'])))
        for t, v in st if t >= route_t - 0.5 and v.get('body_map') and v.get('command_origin_ned')]
R = np.array(rows)
np.savetxt(sys.argv[2] + '/pose_error.csv', R, delimiter=',', fmt='%.6f')
hp = R[:, 1:3] - np.array([np.convolve(R[:, k], np.ones(5) / 5, 'same') for k in (1, 2)]).T
print('%s: %d samples, map-pose error xy rms %.1f cm (max %.1f), 10 Hz jitter %.1f cm, z rms %.1f cm' % (
    sys.argv[2], len(R), 100 * np.sqrt(np.mean(np.sum(R[:, 1:3] ** 2, 1))), 100 * np.max(np.hypot(R[:, 1], R[:, 2])),
    100 * np.sqrt(np.mean(np.sum(hp[2:-2] ** 2, 1))), 100 * np.sqrt(np.mean(R[:, 3] ** 2))))
