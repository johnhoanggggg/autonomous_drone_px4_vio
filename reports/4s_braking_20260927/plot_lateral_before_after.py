"""Before/after control plots for the 4S lateral-stop (sideways hold) test.

Each column is one ULog. Signals are projected on the test axis found the same way as
lat_ulog.py (direction of the first reference leg), and time is aligned to the start of the
first leg. Rows share y-limits across columns so the two tunings compare directly.
Usage: python plot_lateral_before_after.py <before.ulg> <after.ulg> <out.png> "<before label>" "<after label>"
"""
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from pyulog import ULog  # noqa: E402

sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/reports/4s_braking_20260927')
from lat_ulog import analyse  # noqa: E402

SURF, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
MEAS, SP = '#2a78d6', '#eb6834'
MOTORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
HOLD = '#eeeeea'
plt.rcParams.update({
    'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.spines.top': False,
    'axes.spines.right': False, 'font.size': 9, 'axes.titlesize': 9.5, 'axes.titleweight': 'bold',
    'axes.titlecolor': INK, 'axes.titlelocation': 'left', 'legend.frameon': False,
    'legend.fontsize': 8, 'lines.linewidth': 1.3})


def topic(u, name):
    for d in u.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    raise KeyError(name)


def euler_roll(q0, q1, q2, q3):
    return np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))


def lag_s(t, sp, act, fs=100.0, max_lag=0.6):
    tt = np.arange(t[0], t[-1], 1 / fs)
    a = np.interp(tt, t, sp) - np.mean(sp)
    b = np.interp(tt, t, act) - np.mean(act)
    lags = np.arange(0, int(max_lag * fs) + 1)
    c = [np.dot(a[:len(a) - k], b[k:]) for k in lags]
    return lags[int(np.argmax(c))] / fs


def load(path):
    u = ULog(path)
    T0 = u.start_timestamp
    s = lambda a: (a - T0) / 1e6  # noqa: E731
    tsp = topic(u, 'trajectory_setpoint')
    tt = s(tsp['timestamp'])
    rp = np.column_stack([tsp['position[0]'], tsp['position[1]']])
    rv = np.column_stack([tsp['velocity[0]'], tsp['velocity[1]']])
    ok = np.isfinite(rp).all(1)
    moving = ok & (np.nan_to_num(np.linalg.norm(rv, axis=1)) > 1e-4)
    idx = np.flatnonzero(moving)
    a0 = idx[0]
    b0 = a0
    while b0 + 1 < len(moving) and moving[b0 + 1]:
        b0 += 1
    origin, first_end = rp[a0 - 1], rp[b0 + 1]
    uax = (first_end - origin) / np.linalg.norm(first_end - origin)
    t0 = tt[a0]  # first leg start
    res = analyse(path)
    legs = res['legs']
    t_end = max(l_['t_hold_end'] for l_ in legs) + 0.5

    lp = topic(u, 'vehicle_local_position')
    lps = topic(u, 'vehicle_local_position_setpoint')
    att = topic(u, 'vehicle_attitude')
    atts = topic(u, 'vehicle_attitude_setpoint')
    rsp = topic(u, 'vehicle_rates_setpoint')
    angv = topic(u, 'vehicle_angular_velocity')
    mot = topic(u, 'actuator_motors')
    d = {}
    tl = s(lp['timestamp'])
    d['pos'] = (tl - t0, (np.column_stack([lp['x'], lp['y']]) - origin) @ uax)
    d['vel'] = (tl - t0, np.column_stack([lp['vx'], lp['vy']]) @ uax)
    acc = np.column_stack([lp['ax'], lp['ay']]) @ uax
    k = 10  # 0.1 s moving average on the 100 Hz measured acceleration
    d['acc'] = (tl - t0, np.convolve(acc, np.ones(k) / k, mode='same'))
    d['ref_pos'] = (tt - t0, np.where(ok, (rp - origin) @ uax, np.nan))
    d['ref_vel'] = (tt - t0, np.nan_to_num(rv) @ uax)
    ts = s(lps['timestamp'])
    d['acc_sp'] = (ts - t0, np.column_stack([lps['acceleration[0]'], lps['acceleration[1]']]) @ uax)
    ta, tas = s(att['timestamp']), s(atts['timestamp'])
    roll = euler_roll(*(att[f'q[{i}]'] for i in range(4)))
    roll_sp = euler_roll(*(atts[f'q_d[{i}]'] for i in range(4)))
    d['roll'] = (ta - t0, roll)
    d['roll_sp'] = (tas - t0, roll_sp)
    d['rate'] = (s(angv['timestamp']) - t0, np.degrees(angv['xyz[0]']))
    d['rate_sp'] = (s(rsp['timestamp']) - t0, np.degrees(rsp['roll']))
    d['motors'] = (s(mot['timestamp']) - t0, [mot[f'control[{i}]'] for i in range(4)])
    d['holds'] = [(l_['t_ref_stop'] - t0, l_['t_hold_end'] - t0) for l_ in legs]
    d['t_end'] = t_end - t0
    for key, val in list(d.items()):
        if key in ('holds', 't_end'):
            continue
        t_, y_ = val
        keep = t_ <= d['t_end']
        d[key] = (t_[keep], [m_[keep] for m_ in y_] if key == 'motors' else y_[keep])
    win = (ta > 0) & (ta < t_end - t0 - 0.5)
    rs_i = np.interp(ta[win], tas, roll_sp)
    d['metrics'] = dict(
        holds=[l_['hold_s'] for l_ in legs],
        overshoot_cm=[100 * l_['overshoot'] for l_ in legs],
        hold_err_cm=max(100 * l_['hold_max_poserr'] for l_ in legs),
        roll_lag=lag_s(ta[win], rs_i, roll[win]),
        roll_err_rms=float(np.sqrt(np.mean((roll[win] - rs_i) ** 2))),
        params=res.get('params', {}) if 'params' in res else {},
    )
    return d


before, after = load(sys.argv[1]), load(sys.argv[2])
out, lab_b, lab_a = sys.argv[3], sys.argv[4], sys.argv[5]
TITLE = sys.argv[6] if len(sys.argv) > 6 else '4S lateral stop-and-hold test: before vs after PID tuning'
x_end = max(before['t_end'], after['t_end'])
rows = [('Sideways position (m)', 'pos', 'ref_pos', 'reference (Pi)', 'measured (EKF2)'),
        ('Sideways velocity (m/s)', 'vel', 'ref_vel', 'reference (Pi)', 'measured'),
        ('Sideways acceleration (m/s²)', 'acc', 'acc_sp', 'PX4 demand', 'measured (0.1 s avg)'),
        ('Roll angle (deg)', 'roll', 'roll_sp', 'setpoint', 'measured'),
        ('Roll rate (deg/s)', 'rate', 'rate_sp', 'setpoint', 'measured'),
        ('Motor commands (0–1)', 'motors', None, None, None)]
fig, axes = plt.subplots(len(rows), 2, figsize=(14, 15), sharex=True, sharey='row', constrained_layout=True)
fig.get_layout_engine().set(rect=(0, 0, 1, 0.945))
for col, (d, label) in enumerate(((before, lab_b), (after, lab_a))):
    m = d['metrics']
    for r, (title, key, spkey, splab, mlab) in enumerate(rows):
        ax = axes[r, col]
        for h0, h1 in d['holds']:
            ax.axvspan(h0, h1, color=HOLD, zorder=0, lw=0)
        if key == 'motors':
            t, ms = d['motors']
            for i, mv in enumerate(ms):
                ax.plot(t, mv, color=MOTORS[i], lw=0.7, label=f'motor {i + 1}')
            if col == 1:
                ax.legend(loc='upper right', ncol=4)
        else:
            ax.plot(*d[spkey], color=SP, ls='--', lw=1.3, label=splab)
            ax.plot(*d[key], color=MEAS, lw=1.1, label=mlab)
            if col == 1:
                ax.legend(loc='upper right', ncol=2)
        if col == 0:
            ax.set_ylabel(title)
        if r == 0:
            ax.set_title(label.replace(' | ', '\n') + '\n'
                         f'holds {", ".join(f"{h:.1f}" for h in m["holds"])} s  ·  '
                         f'overshoot {", ".join(f"{o:.1f}" for o in m["overshoot_cm"])} cm\n'
                         f'worst hold error {m["hold_err_cm"]:.1f} cm  ·  '
                         f'roll lag {m["roll_lag"]:.2f} s  ·  '
                         f'roll tracking error {m["roll_err_rms"]:.2f}° rms', fontsize=9.5)
        ax.set_xlim(-3, x_end)
for ax in axes[-1]:
    ax.set_xlabel('time from first leg start (s)   ·   grey = hold after each stop')
axes[2, 0].set_ylim(-0.45, 0.45)
axes[4, 0].set_ylim(-15, 15)
axes[5, 0].set_ylim(0.2, 0.85)
fig.suptitle(TITLE, x=0.01, ha='left',
             fontsize=13, fontweight='bold', color=INK)
fig.text(0.01, 0.966, 'Each leg moves 0.35 m sideways at 0.20 m/s and stops; the next leg starts once the drone has held at least 3 s and settled.\n'
         'Signals are projected on the test axis; setpoints dashed, measured solid. Same y-scale per row.',
         fontsize=9, color=INK2, va='top')
fig.savefig(out, dpi=150)
print('wrote', out)
for name, d in (('before', before), ('after', after)):
    m = d['metrics']
    print(name, {k: (np.round(v, 3).tolist() if isinstance(v, (list, float)) else v) for k, v in m.items() if k != 'params'})
