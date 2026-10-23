"""Is the in-session level change accel thermal drift or the board moving?

Both IMUs sit on the same board. In steady hover the airframe is level (no net horizontal
force), so each IMU's raw mean accel y/x = its own bias + the board's tilt relative to the
thrust axis. A board/mount tilt moves BOTH IMUs equally; chip thermal drift does not.
Uses vehicle_imu_status.mean_accel (1 s raw means, before calibration and thermal comp).

Usage: .venv-mavlink/bin/python reports/4s_braking_20260927/imu_pair.py [ulogs...]
"""
import glob
import sys

import numpy as np
from pyulog import ULog


def flight(p):
    U = ULog(p, ['vehicle_imu_status', 'vehicle_local_position', 'vehicle_local_position_setpoint'])
    T = {(d.name, d.multi_id): d.data for d in U.data_list}
    if ('vehicle_imu_status', 1) not in T:
        return None
    s = lambda a: (a - U.start_timestamp)/1e6
    lp = T[('vehicle_local_position', 0)]; tl = s(lp['timestamp']); air = np.flatnonzero(-lp['z'] > .25)
    if len(air) < 50 or tl[air[-1]] - tl[air[0]] < 15:
        return None
    a, b = tl[air[0]] + 5, tl[air[-1]] - 1
    r = dict(name=p.split('px4_')[1][:-4])
    for i in (0, 1):
        d = T[('vehicle_imu_status', i)]; m = (s(d['timestamp']) > a) & (s(d['timestamp']) < b)
        r[f'T{i}'] = d['temperature_accel'][m].mean()
        r[f'x{i}'] = d['mean_accel[0]'][m].mean(); r[f'y{i}'] = d['mean_accel[1]'][m].mean()
    sp = T[('vehicle_local_position_setpoint', 0)]; ts = s(sp['timestamp']); m = (ts > a) & (ts < b); h = np.median(lp['heading'])
    r['push'] = np.nanmean(-np.sin(h)*sp['acceleration[0]'][m] + np.cos(h)*sp['acceleration[1]'][m])
    r['pushf'] = np.nanmean(np.cos(h)*sp['acceleration[0]'][m] + np.sin(h)*sp['acceleration[1]'][m])
    return r


def main():
    paths = sys.argv[1:] or sorted(glob.glob('ros2_ws/flight_logs/px4_20260928_0[2-6]_*.ulg'))
    R = [r for r in map(flight, paths) if r]
    print('hover means, m/s2 (raw, pre-cal/TC)      IMU0 T  IMU1 T |   y0      y1    y0-y1 |   x0      x1    x0-x1 | push side  fwd')
    for r in R:
        print('  %-36s %5.1f %5.1f | %+.3f %+.3f %+.3f | %+.3f %+.3f %+.3f | %+.3f %+.3f' % (
            r['name'], r['T0'], r['T1'], r['y0'], r['y1'], r['y0']-r['y1'], r['x0'], r['x1'], r['x0']-r['x1'], r['push'], r['pushf']))
    A = {k: np.array([r[k] for r in R]) for k in R[0] if k != 'name'}
    def fit(y, *xs):
        X = np.column_stack(xs + (np.ones(len(y)),)); c, *_ = np.linalg.lstsq(X, y, rcond=None)
        return c, np.corrcoef(X[:, :-1] @ c[:-1], y)[0, 1] if len(xs) else np.nan
    print('\nacross %d flights:' % len(R))
    for k in ('y0', 'y1', 'y0-y1', 'x0', 'x1'):
        y = A['y0'] - A['y1'] if k == 'y0-y1' else A[k]
        c, _ = fit(y, A['T0']); print('  %-6s spread %.3f  slope vs IMU0 temp %+.4f /C  corr with side push %+.2f' % (k, np.ptp(y), c[0], np.corrcoef(y, A['push'])[0, 1]))
    c, rr = fit(A['push'], A['T0']); print('  side push vs IMU0 temp: %+.4f m/s2 per C (r=%.2f)' % (c[0], rr))


if __name__ == '__main__':
    main()
