"""Control-signal chain through the gap entry, one flight (bag pickle + ULog).

Stacked panels on one time axis, all sideways components along the route's
right-hand side (+ = right of the direction of travel):
  position (PX4 setpoint vs drone), velocity (PX4 velocity setpoint vs drone),
  acceleration (PX4 command vs drone), the command split into P term and
  velocity integrator, the external push left over, roll (setpoint vs actual),
  roll torque setpoint. Bands mark map x 0.2-0.5 (gap mouth) and 0.5-0.75.
Usage: python3 plot_control.py <pkl> <ulg> <out.png> [x0 x1]
"""
import math
import pickle
import sys

import numpy as np
sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from pyulog import ULog  # noqa: E402

S1, S2 = '#2a78d6', '#eb6834'          # categorical slots 1, 2
INK, INK2, MUTED, GRID = '#0b0b0b', '#52514e', '#8a8984', '#e6e5e0'
SURF = '#fcfcfb'
pkl, ulg, out = sys.argv[1:4]
X0, X1 = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else (-0.1, 1.1)

D = pickle.load(open(pkl, 'rb'))
st = D['/planner3d/flight/status']
blp = np.array(D['/fmu/out/vehicle_local_position_v1'], dtype=float)
U = ULog(ulg)
P_ = U.initial_parameters
get = lambda n: next(d.data for d in U.data_list if d.name == n and d.multi_id == 0)
lp, lps, ts, att, asp, tq = (get(n) for n in ('vehicle_local_position', 'vehicle_local_position_setpoint',
                                              'trajectory_setpoint', 'vehicle_attitude', 'vehicle_attitude_setpoint',
                                              'vehicle_torque_setpoint'))
bt = np.array([t for t, v in st if v.get('body_map')])
bxs = np.array([v['body_map'][0] for t, v in st if v.get('body_map')])
t0r = next(t for t, v in st if v.get('state') == 'ROUTE_START')
t1r = max(t for t, v in st if v.get('state') in ('FOLLOWING', 'HOLD', 'YAW_ALIGN'))
gb = np.arange(t0r, t1r, 0.02)
bx = np.interp(gb, bt, bxs)
win = (bx >= X0) & (bx <= X1)
first = np.argmax(win); last = len(win) - 1 - np.argmax(win[::-1])
gb = gb[first:last + 1]; bx = bx[first:last + 1]
g = np.interp(gb, blp[:, 0], blp[:, 1])          # PX4 microseconds
t = gb - gb[0]
I = lambda d, k: np.interp(g, d['timestamp'], d[k])
idx = np.clip(np.searchsorted(ts['timestamp'], g, side='right') - 1, 0, None)
yaw = np.nan_to_num(ts['yaw'][idx])
R = np.column_stack([-np.sin(yaw), np.cos(yaw)])
side = lambda x, y: x * R[:, 0] + y * R[:, 1]

pos_sp = side(ts['position[0]'][idx], ts['position[1]'][idx]); pos = side(I(lp, 'x'), I(lp, 'y'))
ref = pos_sp.copy()                                # plot relative to the setpoint's own path
vsp = side(I(lps, 'vx'), I(lps, 'vy')); vel = side(I(lp, 'vx'), I(lp, 'vy'))
acc_cmd = side(I(lps, 'acceleration[0]'), I(lps, 'acceleration[1]'))
acc = side(I(lp, 'ax'), I(lp, 'ay'))
p_term = P_['MPC_XY_VEL_P_ACC'] * (vsp - vel)
i_term = acc_cmd - p_term + P_['MPC_XY_VEL_D_ACC'] * acc
lag = np.zeros_like(acc_cmd); lag[0] = acc_cmd[0]
for i in range(1, len(lag)):
    lag[i] = lag[i - 1] + (acc_cmd[i] - lag[i - 1]) * 0.02 / 0.15
push = acc - lag
sm = lambda y: np.convolve(y, np.ones(5) / 5, 'same')  # 0.1 s moving average for the noisy traces
acc, push = sm(acc), sm(push)
eul = lambda q: math.degrees(math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)))
roll = np.array([eul(q) for q in np.column_stack([I(att, 'q[%d]' % k) for k in range(4)])])
roll_sp = np.array([eul(q) for q in np.column_stack([I(asp, 'q_d[%d]' % k) for k in range(4)])])
tq_roll = sm(I(tq, 'xyz[0]'))

# the side offset of setpoint and drone is shown relative to the setpoint's straight-line trend
off_sp = 100 * (pos_sp - ref); off = 100 * (pos - ref)

tx = lambda x: np.interp(x, bx, t) if bx.min() <= x <= bx.max() else None
bands = [(tx(0.2), tx(0.5), 'map x 0.2-0.5\nleft pull', '#e4e3dc'), (tx(0.5), tx(0.75), 'map x 0.5-0.75\npull ends, rebound', '#f1f0eb')]

plt.rcParams.update({'font.size': 9, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.titlecolor': INK, 'axes.titlesize': 9.5, 'axes.titleweight': 'bold',
                     'axes.titlelocation': 'left'})
panels = [
    ('Sideways position vs PX4 setpoint (cm)', [('drone', off, S2)], 'drone offset from setpoint'),
    ('Sideways velocity (m/s)', [('PX4 velocity setpoint', vsp, S1), ('drone', vel, S2)], None),
    ('Sideways acceleration (m/s², drone 0.1 s avg)', [('PX4 command', acc_cmd, S1), ('drone', acc, S2)], None),
    ('PX4 command split (m/s²)', [('P term (velocity error)', p_term, S1), ('velocity integrator', i_term, S2)], None),
    ('External push = drone - lagged PX4 command (m/s², 0.1 s avg)', [('external push', push, S1)], None),
    ('Roll (deg, + = right wing down)', [('roll setpoint', roll_sp, S1), ('roll', roll, S2)], None),
    ('Roll torque setpoint (normalised)', [('roll torque', tq_roll, S1)], None),
]
fig, axes = plt.subplots(len(panels), 1, figsize=(10, 15.5), sharex=True, facecolor=SURF)
for ax, (title, series, _) in zip(axes, panels):
    ax.set_facecolor(SURF)
    for a, b, lab, col in bands:
        if a is not None and b is not None:
            ax.axvspan(a, b, color=col, zorder=0, lw=0)
    ax.axhline(0, color=MUTED, lw=0.8, zorder=1)
    for name, y, c in series:
        ax.plot(t, y, color=c, lw=2, label=name, zorder=3, solid_capstyle='round')
    ax.grid(axis='y', color=GRID, lw=0.6); ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.set_title(title, pad=4)
    if len(series) > 1:
        ax.legend(loc='upper right', frameon=False, fontsize=8, labelcolor=INK2, ncol=len(series))
for a, b, lab, col in bands:
    if a is not None and b is not None:
        axes[0].text(a + 0.08, 0.95, lab, transform=axes[0].get_xaxis_transform(), ha='left', va='top',
                     fontsize=8, color=INK2)
axes[-1].set_xlabel('time since map x = %.1f m (s)' % X0)
mx = axes[-1].secondary_xaxis(-0.42, functions=(lambda tt: np.interp(tt, t, bx), lambda xx: np.interp(xx, bx, t)))
mx.set_xlabel('drone map x (m)', color=INK2); mx.tick_params(colors=INK2)
stamp = pkl.split('/')[-1][:6]
fig.suptitle('Flight %s - control signals through the gap entry (sideways, + = right)\n'
             'MC_ROLLRATE_I %.2f, MPC_XY_P %.2f, MPC_XY_VEL_P_ACC %.1f, MPC_XY_VEL_I_ACC %.1f' % (
                 stamp, P_['MC_ROLLRATE_I'], P_['MPC_XY_P'], P_['MPC_XY_VEL_P_ACC'], P_['MPC_XY_VEL_I_ACC']),
             x=0.06, ha='left', color=INK, fontsize=10.5, fontweight='bold', y=0.995)
fig.tight_layout(rect=(0, 0, 1, 0.975))
fig.savefig(out, dpi=110, facecolor=SURF)
print('wrote', out)
