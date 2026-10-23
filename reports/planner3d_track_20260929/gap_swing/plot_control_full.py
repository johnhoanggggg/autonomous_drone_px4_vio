"""Whole-flight control signals for one 3D flight (bag pickle from dump3d.py + ULog).

Stacked panels on the bag's time axis, from takeoff to the end of the log:
height, forward speed, sideways offset from the PX4 setpoint, sideways
acceleration (PX4 command vs drone), the command split into P term and velocity
integrator, roll and pitch (setpoint vs actual), motor outputs, and horizontal
specific force (cage contacts show as spikes). Sideways/forward are along the
route heading PX4 was given (+ = right / forward). HOLD periods are shaded.
Usage: python3 plot_control_full.py <pkl> <ulg> <out.png>
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

SLOTS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']       # categorical slots 1-4
INK, INK2, MUTED, GRID, SURF = '#0b0b0b', '#52514e', '#8a8984', '#e6e5e0', '#fcfcfb'
HOLD_FILL, CONTACT = '#f6e3c8', '#c0392b'
pkl, ulg, out = sys.argv[1:4]

D = pickle.load(open(pkl, 'rb'))
st = D['/planner3d/flight/status']
blp = np.array(D['/fmu/out/vehicle_local_position_v1'], dtype=float)
U = ULog(ulg)
P_ = U.initial_parameters
get = lambda n: next(d.data for d in U.data_list if d.name == n and d.multi_id == 0)
lp, lps, ts, att, asp, acc_b, mot = (get(n) for n in (
    'vehicle_local_position', 'vehicle_local_position_setpoint', 'trajectory_setpoint',
    'vehicle_attitude', 'vehicle_attitude_setpoint', 'vehicle_acceleration', 'actuator_motors'))

t_climb = next(t for t, v in st if v.get('state') == 'CLIMB')
t_end = min(blp[-1, 0], st[-1][0])
gb = np.arange(t_climb - 0.5, t_end, 0.02)
g = np.interp(gb, blp[:, 0], blp[:, 1])            # PX4 microseconds on the bag time grid
I = lambda d, k: np.interp(g, d['timestamp'], d[k])
idx = np.clip(np.searchsorted(ts['timestamp'], g, side='right') - 1, 0, None)
yaw = np.nan_to_num(ts['yaw'][idx])
fwd = np.column_stack([np.cos(yaw), np.sin(yaw)]); rgt = np.column_stack([-np.sin(yaw), np.cos(yaw)])
along = lambda x, y, e: x * e[:, 0] + y * e[:, 1]
sm = lambda y, n=5: np.convolve(y, np.ones(n) / n, 'same')

ground_z = next(v['ground_ned'][2] for t, v in st if v.get('ground_ned'))
h = ground_z - I(lp, 'z'); h_sp = ground_z - ts['position[2]'][idx]
v_f, vsp_f = along(I(lp, 'vx'), I(lp, 'vy'), fwd), along(I(lps, 'vx'), I(lps, 'vy'), fwd)
side_off = 100 * along(I(lp, 'x') - ts['position[0]'][idx], I(lp, 'y') - ts['position[1]'][idx], rgt)
a_cmd = along(I(lps, 'acceleration[0]'), I(lps, 'acceleration[1]'), rgt)
a_drn = sm(along(I(lp, 'ax'), I(lp, 'ay'), rgt))
vsp_r, v_r = along(I(lps, 'vx'), I(lps, 'vy'), rgt), along(I(lp, 'vx'), I(lp, 'vy'), rgt)
p_term = P_['MPC_XY_VEL_P_ACC'] * (vsp_r - v_r)
i_term = a_cmd - p_term + P_['MPC_XY_VEL_D_ACC'] * along(I(lp, 'ax'), I(lp, 'ay'), rgt)


def euler(qs):
    w, x, y, z = qs.T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


roll, pitch = euler(np.column_stack([I(att, 'q[%d]' % k) for k in range(4)]))
roll_sp, pitch_sp = euler(np.column_stack([I(asp, 'q_d[%d]' % k) for k in range(4)]))
motors = [I(mot, 'control[%d]' % k) for k in range(4)]
# full-rate horizontal specific force for contact spikes (not interpolated away)
ta = np.interp(acc_b['timestamp'], blp[:, 1], blp[:, 0])
fa = np.hypot(acc_b['xyz[0]'], acc_b['xyz[1]'])
keep = (ta >= gb[0]) & (ta <= gb[-1])
ta, fa = ta[keep], fa[keep]
contacts = []
for i in np.where(fa > 3.0)[0]:
    if not contacts or ta[i] - contacts[-1][0] > 0.5:
        contacts.append((ta[i], fa[i]))
    elif fa[i] > contacts[-1][1]:
        contacts[-1] = (contacts[-1][0], fa[i])

holds, cur = [], None
for t, v in st:
    s = v.get('state')
    if s in ('HOLD', 'YAW_ALIGN') and cur is None:
        cur = [t, t, s + (' ' + v.get('fault') if v.get('fault') else '')]
    elif s in ('HOLD', 'YAW_ALIGN'):
        cur[1] = t
    elif cur is not None:
        holds.append(cur); cur = None
if cur:
    holds.append(cur)
events = []
for t, v in st:
    s = v.get('state')
    if s in ('ROUTE_START', 'GOAL_REACHED', 'SAFE_APPROACH_REACHED', 'LAND') and (not events or events[-1][1] != s):
        events.append((t, s))

plt.rcParams.update({'font.size': 9, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.titlecolor': INK, 'axes.titlesize': 9.5, 'axes.titleweight': 'bold',
                     'axes.titlelocation': 'left'})
panels = [
    ('Height above takeoff (m)', [('setpoint', h_sp, SLOTS[0]), ('drone', h, SLOTS[1])]),
    ('Forward speed along the route (m/s)', [('PX4 velocity setpoint', vsp_f, SLOTS[0]), ('drone', v_f, SLOTS[1])]),
    ('Sideways offset from PX4 setpoint (cm, + = right)', [('drone', side_off, SLOTS[1])]),
    ('Sideways acceleration (m/s², drone 0.1 s avg)', [('PX4 command', a_cmd, SLOTS[0]), ('drone', a_drn, SLOTS[1])]),
    ('PX4 sideways command split (m/s²)', [('P term (velocity error)', p_term, SLOTS[0]), ('velocity integrator', i_term, SLOTS[1])]),
    ('Roll (deg, + = right wing down)', [('setpoint', roll_sp, SLOTS[0]), ('roll', roll, SLOTS[1])]),
    ('Pitch (deg, - = nose down)', [('setpoint', pitch_sp, SLOTS[0]), ('pitch', pitch, SLOTS[1])]),
    ('Motor outputs (0-1)', [('motor %d' % (k + 1), motors[k], SLOTS[k]) for k in range(4)]),
]
fig, axes = plt.subplots(len(panels) + 1, 1, figsize=(11, 19), sharex=True, facecolor=SURF)
for ax, (title, series) in zip(axes, panels):
    for y, lw in [(None, None)]:
        pass
    for name, y, c in series:
        ax.plot(gb, y, color=c, lw=1.6 if len(series) > 2 else 2, label=name, zorder=3)
    ax.set_title(title, pad=4)
    if len(series) > 1:
        ax.legend(loc='lower right', bbox_to_anchor=(1.0, 0.98), frameon=False, fontsize=8, labelcolor=INK2, ncol=len(series), borderaxespad=0)
ax = axes[-1]
ax.plot(ta, fa, color=SLOTS[0], lw=1, zorder=3)
ax.axhline(3.0, color=MUTED, lw=0.8, ls='--')
ax.text(gb[-1], 2.9, 'contact threshold 3 m/s²', color=INK2, fontsize=8, va='top', ha='right')
for t, f in contacts:
    ax.annotate('%.1f' % f, (t, f), textcoords='offset points', xytext=(0, 4), ha='center', fontsize=8, color=INK)
ax.set_title('Horizontal specific force, full rate (m/s²): spikes = bumps / cage contacts', pad=4)
for ax in axes:
    ax.set_facecolor(SURF)
    for a, b, lab in holds:
        ax.axvspan(a, max(b, a + 0.1), color=HOLD_FILL, zorder=0, lw=0)
    for t, f in contacts:
        ax.axvline(t, color=CONTACT, lw=0.8, alpha=0.6, zorder=2)
    for t, s in events:
        ax.axvline(t, color=MUTED, lw=0.8, ls=':', zorder=2)
    ax.axhline(0, color=MUTED, lw=0.6, zorder=1) if ax is not axes[0] and ax is not axes[-2] else None
    ax.grid(axis='y', color=GRID, lw=0.6); ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
for t, s in events:
    axes[0].text(t + 0.15, 0.04, s.replace('_', ' ').lower(), transform=axes[0].get_xaxis_transform(), rotation=0,
                 fontsize=8, color=INK2, ha='left', va='bottom')
for a, b, lab in holds:
    axes[2].text(a, 0.97, lab.replace('_', ' ').lower(), transform=axes[2].get_xaxis_transform(), fontsize=7.5,
                 color=INK2, ha='left', va='top')
axes[-1].set_xlabel('bag time (s)')
stamp = pkl.split('/')[-1][:6]
cfg = D['/planner3d/flight/config'][-1][1] if D['/planner3d/flight/config'] else {}
fig.suptitle('Flight %s - control signals, takeoff to end\n'
             'shaded = hold, red lines = contacts > 3 m/s², dotted = route events\n'
             'MPC_XY_VEL_P_ACC %.1f, MPC_XY_VEL_I_ACC %.1f, MC_ROLLRATE_I %.2f | clearance %s m, speed %s m/s, path_carrot %s' % (
                 stamp, P_['MPC_XY_VEL_P_ACC'], P_['MPC_XY_VEL_I_ACC'], P_['MC_ROLLRATE_I'],
                 cfg.get('required_clearance'), cfg.get('command_speed'), cfg.get('path_carrot')),
             x=0.05, ha='left', color=INK, fontsize=10, fontweight='bold', y=0.997)
fig.tight_layout(rect=(0, 0, 1, 0.968), h_pad=1.6)
fig.savefig(out, dpi=100, facecolor=SURF)
print('wrote', out, '| contacts:', ['%.1fs %.1f' % c for c in contacts], '| holds:', [(round(a, 1), round(b, 1), l) for a, b, l in holds])
