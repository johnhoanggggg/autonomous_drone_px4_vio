"""Report figures for one global-planner flight: Pi resources, battery power, control signals.

Usage (venv python, has pyulog + matplotlib):
  .venv-mavlink/bin/python plot_flight_report.py <bag_resources.pkl> <flight.ulg> <out_dir> <name>

<bag_resources.pkl> holds {'arm', 'dis', 'perf', 'batt'} extracted from the rosbag
(/perf/processes JSON per second, bag time in s). All plots use time since ARMING,
taken from each log's own arming event, so the Pi (bag) and PX4 (ULog) clocks never
have to agree. Writes <name>_resources.png, <name>_control_outer.png,
<name>_control_inner.png and prints the summary numbers.
"""
import math
import pickle
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from pyulog import ULog  # noqa: E402

pkl, ulg, out_dir, name = sys.argv[1:5]

# Reference palette (dataviz skill, light mode), fixed categorical order.
SURF, INK, INK2, MUTED, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e4e3df'
CAT = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#4a3aa7', '#e34948']
OTHER = '#b9b8b2'
MEAS, SP = CAT[0], CAT[1]

plt.rcParams.update({
    'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.spines.top': False,
    'axes.spines.right': False, 'font.size': 9, 'axes.titlesize': 10, 'axes.titleweight': 'bold',
    'axes.titlecolor': INK, 'axes.titlelocation': 'left', 'legend.frameon': False,
    'legend.fontsize': 8, 'lines.linewidth': 1.4,
})

# ------------------------------------------------------------------ ULog
u = ULog(ulg)


def topic(n, i=0):
    for d in u.data_list:
        if d.name == n and d.multi_id == i:
            return d.data
    raise KeyError(n)


vs = topic('vehicle_status')
armed = vs['arming_state'] == 2
t_arm = vs['timestamp'][np.argmax(armed)]
t_dis = vs['timestamp'][len(armed) - 1 - np.argmax(armed[::-1])]
takeoff_us = vs['takeoff_time'][armed].max()


def T(d):
    return (d['timestamp'] - t_arm) / 1e6


flight_end = (t_dis - t_arm) / 1e6
takeoff = (takeoff_us - t_arm) / 1e6 if takeoff_us > 0 else float('nan')
XL = (-1.0, flight_end + 1.0)


def euler(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))))
    return roll, pitch, yaw


def mark_phases(ax):
    for x in (0.0, takeoff, flight_end):
        if np.isfinite(x):
            ax.axvline(x, color=MUTED, lw=0.8, ls=':', zorder=0)
    ax.set_xlim(*XL)


def finish(fig, axes, path, title, note):
    for ax in axes:
        mark_phases(ax)
    axes[-1].set_xlabel('time since arming (s)   ·   dotted lines: arm, takeoff, disarm')
    fig.suptitle(title, x=0.01, ha='left', fontsize=12, fontweight='bold', color=INK)
    fig.text(0.01, 0.955, note, ha='left', fontsize=8.5, color=INK2)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print('wrote', path)


def pair(ax, t_m, y_m, t_s, y_s, unit, title):
    ax.plot(t_s, y_s, color=SP, ls='--', lw=1.4, label='setpoint')
    ax.plot(t_m, y_m, color=MEAS, lw=1.2, label='measured')
    ax.set_ylabel(unit)
    ax.set_title(title)
    ax.legend(loc='best', ncol=2)


summary = {}
in_air = None  # mask helper per stream


def air(t):
    return (t > takeoff + 2.0) & (t < flight_end - 3.0)


# ------------------------------------------------------------------ outer loop: position / velocity
lp, lps = topic('vehicle_local_position'), topic('vehicle_local_position_setpoint')
tl, ts = T(lp), T(lps)
fig, axes = plt.subplots(6, 1, figsize=(11, 13), sharex=True, constrained_layout=True)
fig.get_layout_engine().set(rect=(0, 0, 1, 0.94))
rows = [('x', 'x', 'm', 'Position x (north, PX4 local frame)'),
        ('y', 'y', 'm', 'Position y (east)'),
        ('z', 'z', 'm', 'Height (up = −z)'),
        ('vx', 'vx', 'm/s', 'Velocity x'),
        ('vy', 'vy', 'm/s', 'Velocity y'),
        ('vz', 'vz', 'm/s', 'Vertical velocity (up = −vz)')]
for ax, (m, s, unit, title) in zip(axes, rows):
    sign = -1.0 if m in ('z', 'vz') else 1.0
    pair(ax, tl, sign * lp[m], ts, sign * lps[s], unit, title)
    if m == 'vz':
        # PX4's takeoff/landing setpoints reach 2.4 m/s and flatten the cruise.
        ax.set_ylim(-0.8, 0.6)
        ax.set_title(title + '   (axis clipped to ±0.8 m/s; takeoff/landing setpoints exceed it)')
for key in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
    sp = np.interp(tl, ts, lps[key])
    mask = air(tl) & np.isfinite(sp)
    summary[f'track_rms_{key}'] = float(np.sqrt(np.mean((lp[key][mask] - sp[mask]) ** 2)))
finish(fig, list(axes), f'{out_dir}/{name}_control_outer.png',
       f'{name}: position and velocity control (PX4 ULog, 100 Hz / setpoints 10 Hz)',
       'Setpoint = vehicle_local_position_setpoint (what PX4\'s position controller was asked for); '
       'measured = EKF2 estimate fusing the OAK-D VIO.')

# ------------------------------------------------------------------ inner loop: attitude, rates, thrust, motors
att, atts = topic('vehicle_attitude'), topic('vehicle_attitude_setpoint')
ta, tas = T(att), T(atts)
r_m, p_m, y_m = euler(*(att[f'q[{i}]'] for i in range(4)))
r_s, p_s, y_s = euler(*(atts[f'q_d[{i}]'] for i in range(4)))
# Align the setpoint yaw branch with the measured one.
y_s = y_s + 360.0 * np.round((np.interp(tas, ta, y_m) - y_s) / 360.0)
av, rs = topic('vehicle_angular_velocity'), topic('vehicle_rates_setpoint')
tv, tr = T(av), T(rs)
thr, tq = topic('vehicle_thrust_setpoint'), topic('vehicle_torque_setpoint')
mot = topic('actuator_motors')
tm = T(mot)

fig, axes = plt.subplots(8, 1, figsize=(11, 17), sharex=True, constrained_layout=True)
fig.get_layout_engine().set(rect=(0, 0, 1, 0.955))
pair(axes[0], ta, r_m, tas, r_s, 'deg', 'Roll angle')
pair(axes[1], ta, p_m, tas, p_s, 'deg', 'Pitch angle')
pair(axes[2], ta, y_m, tas, y_s, 'deg', 'Yaw angle (unwrapped)')
for ax, i, key, title in ((axes[3], 0, 'roll', 'Roll rate'), (axes[4], 1, 'pitch', 'Pitch rate'),
                          (axes[5], 2, 'yaw', 'Yaw rate')):
    pair(ax, tv, np.degrees(av[f'xyz[{i}]']), tr, np.degrees(rs[key]), 'deg/s', title)
ax = axes[6]
ax.plot(T(thr), -thr['xyz[2]'], color=CAT[0], label='collective thrust (normalised)')
ax.set_ylabel('0–1')
ax.set_title('Collective thrust setpoint')
ax.legend(loc='upper right')
ax = axes[7]
for i in range(4):
    ax.plot(tm, mot[f'control[{i}]'], color=CAT[i], lw=0.9, label=f'motor {i + 1}')
ax.set_ylabel('0–1')
ax.set_ylim(0, 1)
ax.set_title('Motor commands (actuator_motors)')
ax.legend(loc='upper right', ncol=4)
finish(fig, list(axes), f'{out_dir}/{name}_control_inner.png',
       f'{name}: attitude, rate, thrust and motor signals (PX4 ULog, 100–200 Hz)',
       'Cascade: position → velocity → attitude → body rate → torque/thrust → motors. '
       'Setpoints dashed, measured solid.')

for key, tm_, m_, ts_, s_ in (('roll', ta, r_m, tas, r_s), ('pitch', ta, p_m, tas, p_s),
                              ('yaw', ta, y_m, tas, y_s)):
    mask = air(tm_)
    summary[f'att_rms_{key}_deg'] = float(np.sqrt(np.mean((m_[mask] - np.interp(tm_[mask], ts_, s_)) ** 2)))
for i, key in enumerate(('roll', 'pitch', 'yaw')):
    mask = air(tv)
    err = np.degrees(av[f'xyz[{i}]'][mask] - np.interp(tv[mask], tr, rs[key]))
    summary[f'rate_rms_{key}_dps'] = float(np.sqrt(np.mean(err ** 2)))
mask = air(tm)
motors = np.stack([mot[f'control[{i}]'][mask] for i in range(4)])
summary['motor_mean'] = [round(float(m), 3) for m in motors.mean(axis=1)]
summary['motor_max'] = [round(float(m), 3) for m in motors.max(axis=1)]
summary['motor_saturated_pct'] = round(float((motors > 0.95).mean() * 100), 2)
summary['thrust_mean'] = float(-thr['xyz[2]'][air(T(thr))].mean())
summary['max_tilt_deg'] = float(np.max(np.hypot(r_m[air(ta)], p_m[air(ta)])))

# ------------------------------------------------------------------ resources + power
R = pickle.load(open(pkl, 'rb'))
perf = [(t - R['arm'], d) for t, d in R['perf']]
tp = np.array([t for t, _ in perf])
procs = sorted({k for _, d in perf for k, v in d['processes'].items() if v.get('running')})
cpu = {k: np.array([d['processes'].get(k, {}).get('cpu_percent_of_machine', 0.0) or 0.0 for _, d in perf])
       for k in procs}
mem = {k: np.array([d['processes'].get(k, {}).get('mem_mb', 0.0) or 0.0 for _, d in perf]) for k in procs}
top = sorted(procs, key=lambda k: -cpu[k].mean())[:6]
rest = [k for k in procs if k not in top]
machine = np.array([d['cpu_percent'] for _, d in perf])
memtot = np.array([d['mem_percent'] for _, d in perf])
temp = np.array([d['cpu_temp_c'] for _, d in perf])
LABEL = {'slam': 'SLAM + VIO (OAK-D)', 'bag_record': 'bag recorder', 'vio_bridge': 'VIO → PX4 bridge',
         'px4_pos': 'PX4 pose relay', 'planner_cpp': 'flight adapter (C++)', 'xrce_agent': 'XRCE agent',
         'astar_cpp': 'A* planner (C++)', 'follower_cpp': 'route follower (C++)', 'foxglove': 'Foxglove bridge'}

bat = topic('battery_status')
tb = T(bat)
V, I = bat['voltage_v'], bat['current_a']
P = V * I
fl = (tb >= 0) & (tb <= flight_end)
energy_wh = float(np.trapezoid(P[fl], tb[fl]) / 3600.0)
hover = air(tb)

fig, axes = plt.subplots(4, 1, figsize=(11, 10.5), sharex=True, constrained_layout=True)
fig.get_layout_engine().set(rect=(0, 0, 1, 0.93))
# CPU and memory are summarised per process in the printed table instead.
ax = axes[0]
ax.plot(tp, temp, color=CAT[1])
ax.set_ylabel('°C')
ax.set_title('Pi CPU temperature')
ax = axes[1]
ax.plot(tb, V, color=CAT[0])
ax.set_ylabel('V')
ax.set_title('Battery voltage (4S LiPo)')
ax = axes[2]
ax.plot(tb, I, color=CAT[0])
ax.set_ylabel('A')
ax.set_title('Battery current (whole drone; sensor reads 0 A while disarmed)')
ax = axes[3]
ax.plot(tb, P, color=CAT[0])
ax.axhline(P[hover].mean(), color=MUTED, ls='--', lw=0.9)
ax.text(XL[0] + 0.5, P[hover].mean() * 1.03, f'cruise mean {P[hover].mean():.0f} W', color=INK2, fontsize=8)
ax.set_ylabel('W')
ax.set_title(f'Electrical power = V × I   ·   {energy_wh:.2f} Wh used while armed')
finish(fig, list(axes), f'{out_dir}/{name}_resources.png',
       f'{name}: Pi temperature and battery power',
       'Pi data: /perf/processes from the flight bag (1 Hz). Battery: PX4 battery_status (10 Hz). '
       'Both aligned to their own arming event.')

ap = air(tp)
summary.update(
    flight_s=round(flight_end, 1), takeoff_s=round(takeoff, 2),
    cpu_machine_mean=round(float(machine[ap].mean()), 1), cpu_machine_max=round(float(machine[ap].max()), 1),
    cpu_by_process_mean={LABEL.get(k, k): round(float(cpu[k][ap].mean()), 1) for k in top},
    cpu_by_process_core_pct={LABEL.get(k, k): round(float(cpu[k][ap].mean() * 4), 0) for k in top},
    mem_used_mb=round(float((memtot[ap] / 100 * 7937).mean()), 0), mem_pct=round(float(memtot[ap].mean()), 1),
    mem_by_process_mb={LABEL.get(k, k): round(float(mem[k][ap].mean()), 0) for k in top},
    temp_c=(round(float(temp[ap].min()), 1), round(float(temp[ap].max()), 1)),
    batt_v=(round(float(V[fl].max()), 2), round(float(V[fl].min()), 2)),
    current_cruise_mean=round(float(I[hover].mean()), 1), current_max=round(float(I[fl].max()), 1),
    power_cruise_mean=round(float(P[hover].mean()), 0), power_max=round(float(P[fl].max()), 0),
    energy_wh=round(energy_wh, 2),
    discharged_mah=round(float(bat['discharged_mah'][fl].max() - bat['discharged_mah'][fl].min()), 0),
)
for k, v in summary.items():
    print(f'{k}: {v:.4g}' if isinstance(v, float) else f'{k}: {v}')
