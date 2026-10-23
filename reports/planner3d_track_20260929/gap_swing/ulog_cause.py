"""Is the gap-entry side push physical or a VIO artefact? ULog + bag pickle.

Per 0.1 m of map x (bag body_map), projected on the route's right-hand side (+ = right):
  push     : bag-model residual push (as push.py)        [m/s2]
  a_true   : EKF acceleration (IMU-based)                 [m/s2]
  a_tilt   : thrust-direction accel from ACTUAL attitude  [m/s2]
  a_tiltsp : same from the attitude SETPOINT              [m/s2]
  f_ext    : body x/y specific force rotated to NED (non-thrust aero force on the frame) [m/s2]
  ev_inn   : EV position innovation (VIO minus EKF prediction) [cm], and max test ratio
  tq_roll  : roll torque setpoint (normalised), rate integrator roll
Usage: python3 ulog_cause.py <pkl> <ulg>
"""
import math
import pickle
import sys

import numpy as np
sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
from pyulog import ULog  # noqa: E402

pkl, ulg = sys.argv[1], sys.argv[2]
D = pickle.load(open(pkl, 'rb'))
U = ULog(ulg)


def top(name, i=0):
    return next(d.data for d in U.data_list if d.name == name and d.multi_id == i)


def rotm(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


st = D['/planner3d/flight/status']
blp = np.array(D['/fmu/out/vehicle_local_position_v1'], dtype=float)  # bag t, px4 us, ...
t0 = next(t for t, v in st if v.get('state') == 'ROUTE_START')
t1 = max(t for t, v in st if v.get('state') in ('FOLLOWING', 'HOLD', 'YAW_ALIGN'))
bag2us = lambda t: np.interp(t, blp[:, 0], blp[:, 1])
g_bag = np.arange(t0, t1, 0.02)
g = bag2us(g_bag)
bx = np.interp(g_bag, [t for t, v in st if v.get('body_map')], [v['body_map'][0] for t, v in st if v.get('body_map')])

lp = top('vehicle_local_position'); ts = top('trajectory_setpoint'); att = top('vehicle_attitude')
asp = top('vehicle_attitude_setpoint'); acc = top('vehicle_acceleration'); ev = top('estimator_aid_src_ev_pos')
tq = top('vehicle_torque_setpoint'); rcs = top('rate_ctrl_status')
I = lambda d, k: np.interp(g, d['timestamp'], d[k])
yaw = np.interp(g, ts['timestamp'], np.nan_to_num(ts['yaw'], nan=0.0))
right = np.column_stack([-np.sin(yaw), np.cos(yaw)])
side = lambda xy: np.sum(xy * right, axis=1)

# bag-model push (same as push.py), on this grid
KP, KV, TAU = 0.95, 2.2, 0.15
P = np.column_stack([I(lp, 'x'), I(lp, 'y')]); V = np.column_stack([I(lp, 'vx'), I(lp, 'vy')])
idx = np.clip(np.searchsorted(ts['timestamp'], g, side='right') - 1, 0, len(ts['timestamp']) - 1)
S = np.column_stack([ts['position[0]'][idx], ts['position[1]'][idx]])
FF = np.nan_to_num(np.column_stack([ts['velocity[0]'][idx], ts['velocity[1]'][idx]]))
A = np.gradient(np.column_stack([np.convolve(V[:, i], np.ones(5) / 5, 'same') for i in (0, 1)]), 0.02, axis=0)
am = np.zeros(2); push = np.zeros_like(A)
for i in range(len(g)):
    am += (KV * (FF[i] + KP * (S[i] - P[i]) - V[i]) - am) * 0.02 / TAU
    push[i] = A[i] - am

a_true = np.column_stack([I(lp, 'ax'), I(lp, 'ay')])
Q = np.column_stack([I(att, 'q[%d]' % k) for k in range(4)])
Qs = np.column_stack([I(asp, 'q_d[%d]' % k) for k in range(4)])
fb = np.column_stack([I(acc, 'xyz[%d]' % k) for k in range(3)])
a_tilt = np.zeros((len(g), 2)); a_tiltsp = np.zeros((len(g), 2)); f_ext = np.zeros((len(g), 2))
for i in range(len(g)):
    R = rotm(Q[i] / np.linalg.norm(Q[i])); Rs = rotm(Qs[i] / np.linalg.norm(Qs[i]))
    fz = fb[i, 2]  # specific force along body z (negative = thrust up)
    a_tilt[i] = (R @ np.array([0, 0, fz]))[:2]
    a_tiltsp[i] = (Rs @ np.array([0, 0, fz]))[:2]
    f_ext[i] = (R @ np.array([fb[i, 0], fb[i, 1], 0]))[:2]
evi = np.column_stack([np.interp(g, ev['timestamp'], ev['innovation[%d]' % k]) for k in (0, 1)])
evr = np.maximum(np.interp(g, ev['timestamp'], ev['test_ratio[0]']), np.interp(g, ev['timestamp'], ev['test_ratio[1]']))
# roll torque about the route axis: body roll torque ~ side when heading ~ route
troll = I(tq, 'xyz[0]'); rint = I(rcs, 'rollspeed_integ')
cols = dict(push=side(push), a_true=side(a_true), a_tilt=side(a_tilt), a_tiltsp=side(a_tiltsp), f_ext=side(f_ext),
            ev_inn=100 * side(evi), ev_tr=evr, tq_roll=troll, rint=rint)
print(ulg.split('/')[-1], 'ROUTE %.1f-%.1f s' % (t0, t1))
print('map x    | push  a_true  a_tilt a_tiltsp  f_ext | ev_inn cm  ev_tr | tq_roll  roll_int')
for lo in np.arange(0.0, 1.6, 0.1):
    m = (bx >= lo) & (bx < lo + 0.1)
    if m.sum() < 10:
        continue
    c = {k: v[m] for k, v in cols.items()}
    print('%.1f-%.1f  | %+5.2f  %+5.2f  %+5.2f  %+5.2f   %+5.2f | %+6.1f   %5.2f | %+6.3f  %+6.3f' % (
        lo, lo + 0.1, c['push'].mean(), c['a_true'].mean(), c['a_tilt'].mean(), c['a_tiltsp'].mean(), c['f_ext'].mean(),
        c['ev_inn'].mean(), c['ev_tr'].max(), c['tq_roll'].mean(), c['rint'].mean()))
