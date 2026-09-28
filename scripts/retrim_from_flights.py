#!/usr/bin/env python3
"""Re-trim the level reference (SENS_BOARD_X_OFF / Y_OFF) from recent hover flights.

The FC sits on soft mounts and its level reference creeps between sessions; the
drone also does not rest level, so a ground level-cal bakes in its leg tilt. This
fits the trim from the air instead: the steady horizontal acceleration PX4 demands
while holding position (the "push") is the level error, 1 deg ~= 0.17 m/s2.

    .venv-mavlink/bin/python3 scripts/retrim_from_flights.py                 # latest session, report only
    .venv-mavlink/bin/python3 scripts/retrim_from_flights.py --apply         # write + save over USB/NSH
    .venv-mavlink/bin/python3 scripts/retrim_from_flights.py a.ulg b.ulg     # specific ULogs

Default flights: the newest ULogs in ros2_ws/flight_logs that flew >= 10 s in the air
with the SAME offsets as the newest one (up to --last). The FC must be on USB,
Disarmed, and no offboard flight node may be running for --apply.
"""
import argparse
import glob
import math
import os
import re
import subprocess
import sys

import numpy as np
from pyulog import ULog

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# Measured on this airframe (2026-08-28 .. 09-28): reported attitude moves 1.10-1.17 deg
# per deg of SENS_BOARD_*_OFF, opposite sign.
GAIN = 1.14
G = 9.81


def flight_push(path):
    """Mean body-frame push (side, fwd) in m/s2, hover roll/pitch (deg) and offsets, or None."""
    U = ULog(path, ['vehicle_local_position', 'vehicle_local_position_setpoint', 'vehicle_attitude'])
    T = {d.name: d.data for d in U.data_list if d.multi_id == 0}
    P = U.initial_parameters
    if not {'vehicle_local_position', 'vehicle_local_position_setpoint', 'vehicle_attitude'} <= T.keys():
        return None
    s = lambda a: (a - U.start_timestamp) / 1e6
    lp = T['vehicle_local_position']; tl = s(lp['timestamp'])
    air = np.flatnonzero(-lp['z'] > .25)
    if len(air) < 50:
        return None
    # skip the takeoff transient: the velocity integrator needs a few s to take up the lean
    t0, t1 = tl[air[0]] + 5, tl[air[-1]] - 1
    if t1 - t0 < 10:
        return None
    sp = T['vehicle_local_position_setpoint']; ts = s(sp['timestamp']); m = (ts > t0) & (ts < t1)
    h = np.median(lp['heading'])
    ax, ay = sp['acceleration[0]'][m], sp['acceleration[1]'][m]
    a = T['vehicle_attitude']; ta = s(a['timestamp']); ma = (ta > t0) & (ta < t1)
    q = np.column_stack([a[f'q[{i}]'] for i in range(4)])[ma]
    roll = np.degrees(np.arctan2(2*(q[:, 0]*q[:, 1]+q[:, 2]*q[:, 3]), 1-2*(q[:, 1]**2+q[:, 2]**2)))
    pitch = np.degrees(np.arcsin(np.clip(2*(q[:, 0]*q[:, 2]-q[:, 3]*q[:, 1]), -1, 1)))
    return dict(name=os.path.basename(path),
                side=float(np.nanmean(-math.sin(h)*ax + math.cos(h)*ay)),
                fwd=float(np.nanmean(math.cos(h)*ax + math.sin(h)*ay)),
                roll=float(roll.mean()), pitch=float(pitch.mean()),
                x_off=round(float(P['SENS_BOARD_X_OFF']), 3), y_off=round(float(P['SENS_BOARD_Y_OFF']), 3))


def default_flights(last):
    out = []
    for p in sorted(glob.glob(os.path.join(ROOT, 'ros2_ws/flight_logs/px4_20[0-9][0-9][01][0-9][0-3][0-9]_*.ulg')), reverse=True):
        f = flight_push(p)
        if f is None:
            continue
        if out and (f['x_off'], f['y_off']) != (out[0]['x_off'], out[0]['y_off']):
            break
        out.append(f)
        if len(out) == last:
            break
    return out[::-1]


def nsh(*cmds, timeout=90):
    r = subprocess.run([sys.executable, os.path.join(HERE, 'nsh.py'), *cmds],
                       capture_output=True, text=True, timeout=timeout, cwd=ROOT)
    return r.stdout + r.stderr


def offsets():
    text = nsh('param show SENS_BOARD_X_OFF', 'param show SENS_BOARD_Y_OFF')
    get = lambda n: float(re.search(rf'{n} \[\d+,\d+\] : (-?[\d.]+)', text).group(1))
    return get('SENS_BOARD_X_OFF'), get('SENS_BOARD_Y_OFF')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('ulogs', nargs='*')
    p.add_argument('--last', type=int, default=6, help='max flights from the latest session (default 6)')
    p.add_argument('--deadband', type=float, default=0.3, help='leave an axis alone below this many deg')
    p.add_argument('--apply', action='store_true', help='write and save the new offsets over USB')
    a = p.parse_args()

    F = [f for f in map(flight_push, a.ulogs) if f] if a.ulogs else default_flights(a.last)
    if not F:
        sys.exit('no usable hover flights')
    if len({(f['x_off'], f['y_off']) for f in F}) > 1:
        sys.exit('flights were flown with different offsets; pass ULogs from one trim only')
    print('flight                          push side/fwd m/s2   hover roll/pitch deg')
    for f in F:
        print(f"  {f['name']:<30} {f['side']:+.3f} / {f['fwd']:+.3f}      {f['roll']:+.2f} / {f['pitch']:+.2f}")
    side, fwd = np.mean([f['side'] for f in F]), np.mean([f['fwd'] for f in F])
    lean_roll, lean_pitch = math.degrees(math.atan2(side, G)), math.degrees(math.atan2(-fwd, G))
    x0, y0 = F[0]['x_off'], F[0]['y_off']
    # lean_* has the sign of the hover attitude it causes (+roll right-down, +pitch nose-up);
    # raising an offset lowers that reported attitude (09-28 02:40 trim: roll +0.53 -> X_OFF +0.5,
    # pitch +0.55 -> Y_OFF +0.35), so both axes move by +lean / GAIN
    dx = lean_roll / GAIN if abs(lean_roll) >= a.deadband else 0.0
    dy = lean_pitch / GAIN if abs(lean_pitch) >= a.deadband else 0.0
    x1, y1 = round(x0 + dx, 2), round(y0 + dy, 2)
    print(f'mean push side {side:+.3f} fwd {fwd:+.3f} m/s2 -> level error roll {lean_roll:+.2f} pitch {lean_pitch:+.2f} deg '
          f'(spread roll {np.ptp([f["roll"] for f in F]):.2f} deg across flights)')
    print(f'SENS_BOARD_X_OFF {x0:.2f} -> {x1:.2f}   SENS_BOARD_Y_OFF {y0:.2f} -> {y1:.2f}'
          + ('   (no change: within deadband)' if (dx, dy) == (0, 0) else ''))
    if not a.apply or (dx, dy) == (0, 0):
        return 0

    if not os.path.exists('/dev/ttyACM0'):
        sys.exit('FC not on USB')
    if subprocess.run(['pgrep', '-f', 'lib/px4_vio_bridge/[o]ffboard_'], capture_output=True).returncode == 0:
        sys.exit('an offboard flight node is running; refusing')
    if 'INFO  [commander] Disarmed' not in nsh('commander status'):
        sys.exit('FC not Disarmed; refusing')
    live = offsets()
    if abs(live[0] - x0) > .01 or abs(live[1] - y0) > .01:
        sys.exit(f'FC offsets {live} differ from the flights ({x0}, {y0}); refusing')
    nsh(f'param set SENS_BOARD_X_OFF {x1:.4f}', f'param set SENS_BOARD_Y_OFF {y1:.4f}', 'param save')
    got = offsets()
    print(f'FC now: SENS_BOARD_X_OFF {got[0]:.2f}  SENS_BOARD_Y_OFF {got[1]:.2f}')
    print(f"restore: scripts/nsh.py 'param set SENS_BOARD_X_OFF {x0:.4f}' 'param set SENS_BOARD_Y_OFF {y0:.4f}' 'param save'")
    return 0 if abs(got[0] - x1) < .01 and abs(got[1] - y1) < .01 else 1


if __name__ == '__main__':
    sys.exit(main())
