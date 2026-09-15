#!/usr/bin/env python3
"""Offline VIO timing against integrated PX4 gyro, never against EKF position.

Run with .venv-mavlink/bin/python3; writes JSON and optionally a score plot.
Positive delay means VIO represents motion older than its timestamp_sample.
The peak-width range is sensitivity, not a statistical confidence interval.
This measures rotational pipeline timing, not a certified translation delay or
an EKF replay. No parameters are changed by this script.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def score_delays(ev_time, ev_yaw, gyro_time, gyro_yaw, *, start, end,
                 window=0.25, excluded=()):
    """Compare equal-duration yaw increments, rejecting gaps and reset windows."""
    ev_time, ev_yaw, gyro_time, gyro_yaw = map(
        lambda a: np.asarray(a, dtype=float),
        (ev_time, ev_yaw, gyro_time, gyro_yaw))
    for time, values in [(ev_time, ev_yaw), (gyro_time, gyro_yaw)]:
        if len(time) < 3 or len(time) != len(values):
            raise ValueError('insufficient samples')
        if not np.all(np.isfinite(time)) or not np.all(np.isfinite(values)):
            raise ValueError('nonfinite input')
        if np.any(np.diff(time) <= 0):
            raise ValueError('timestamps must increase strictly')
    delays = np.arange(0.0, 0.501, 0.005)
    lower = max(start, ev_time[0]+window, gyro_time[0]+window+delays[-1])
    upper = min(end, ev_time[-1], gyro_time[-1])
    grid = np.arange(lower, upper, 0.02)
    # Use the same samples for every lag, including all integration intervals.
    keep = np.ones(len(grid), dtype=bool)
    for times, max_gap, extra in [(ev_time, .20, 0), (gyro_time, .03, delays[-1])]:
        for i in np.flatnonzero(np.diff(times) > max_gap):
            keep &= ~((grid >= times[i]) & (grid <= times[i+1]+window+extra))
    for lo, hi in excluded:
        keep &= ~((grid >= lo) & (grid <= hi+window+delays[-1]))
    grid = grid[keep]
    if len(grid) < 100:
        raise ValueError('less than two seconds of usable overlap')
    ev_delta = np.interp(grid, ev_time, ev_yaw) - np.interp(grid-window, ev_time, ev_yaw)
    if np.std(ev_delta) < np.radians(.15):
        raise ValueError('insufficient yaw excitation')
    scores = []
    for delay in delays:
        gyro_delta = (np.interp(grid-delay, gyro_time, gyro_yaw) -
                      np.interp(grid-delay-window, gyro_time, gyro_yaw))
        if np.std(gyro_delta) < np.radians(.15):
            raise ValueError('insufficient gyro excitation')
        scores.append(float(np.corrcoef(ev_delta, gyro_delta)[0, 1]))
    scores = np.asarray(scores)
    best = int(np.argmax(scores))
    near = delays[scores >= scores[best]-.005]
    return dict(delay_ms=float(delays[best]*1000), correlation=float(scores[best]),
                near_peak_ms=(near[[0, -1]]*1000).tolist(),
                at_270_ms=float(scores[54]), at_225_ms=float(scores[45]),
                samples=len(grid), window_s=window, analysis_interval_s=[start, end],
                boundary_peak=best in (0, len(delays)-1),
                delays_ms=(delays*1000).tolist(), scores=scores.tolist())


def analyze(path):
    from pyulog import ULog
    from analyze_attitude_response import euler_degrees
    u = ULog(str(path))
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    ev, gyro, att = [d[n] for n in
                     ['vehicle_visual_odometry', 'vehicle_angular_velocity', 'vehicle_attitude']]
    base = u.start_timestamp/1e6
    gt = gyro['timestamp_sample']/1e6-base
    et = ev['timestamp_sample']/1e6-base
    at = att['timestamp_sample']/1e6-base
    angles = np.radians(euler_degrees(np.array([att[f'q[{i}]'] for i in range(4)])))
    roll = np.interp(gt, at, angles[0])
    pitch = np.interp(gt, at, angles[1])
    if np.max(abs(pitch)) > np.radians(45):
        raise ValueError('Euler yaw diagnostic requires near-level attitude')
    yaw_rate = (gyro['xyz[1]']*np.sin(roll)+gyro['xyz[2]']*np.cos(roll))/np.cos(pitch)
    integrated = np.cumsum(np.r_[0, np.diff(gt)*(yaw_rate[1:]+yaw_rate[:-1])/2])
    ev_yaw = np.unwrap(np.radians(euler_degrees(np.array([ev[f'q[{i}]'] for i in range(4)]))[2]))
    resets = np.flatnonzero(np.diff(ev['reset_counter'].astype(int)) != 0)
    excluded = [(et[i], et[i+1]) for i in resets]
    # Include attitude gaps: attitude only supplies roll/pitch Euler coupling.
    excluded += [(at[i], at[i+1]) for i in np.flatnonzero(np.diff(at) > .03)]
    status = d['vehicle_status']
    st = status['timestamp']/1e6-base
    transitions = st[(st > 3) & ((status['arming_state'] != 2) | (status['nav_state'] != 14))]
    end = min(et[-1]-.6, gt[-1]-.6, float(transitions[0]) if len(transitions) else np.inf)
    start = 3.0
    result = dict(ulog=str(path), configured_delay_ms=float(u.initial_parameters['EKF2_EV_DELAY']),
                  receipt_minus_sample_ms=np.percentile(
                      (ev['timestamp'].astype(float)-ev['timestamp_sample'])/1000, [0, 50, 100]).tolist(),
                  ev_reset_intervals=len(resets),
                  full=score_delays(et, ev_yaw, gt, integrated, start=start, end=end, excluded=excluded),
                  sensitivity=[], halves=[])
    for window in [.15, .35]:
        result['sensitivity'].append(score_delays(et, ev_yaw, gt, integrated,
            start=start, end=end, window=window, excluded=excluded))
    for lo, hi in [(start, (start+end)/2), ((start+end)/2, end)]:
        try:
            result['halves'].append(score_delays(et, ev_yaw, gt, integrated,
                start=lo, end=hi, excluded=excluded))
        except ValueError as error:
            result['halves'].append({'unavailable': str(error)})
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('ulogs', nargs='+', type=Path)
    p.add_argument('--out', required=True, type=Path)
    p.add_argument('--plot', action='store_true')
    args = p.parse_args()
    results = [analyze(path) for path in args.ulogs]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix('.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    for result in results:
        print(result['ulog'], result['full']['delay_ms'], result['full']['correlation'],
              'near peak', result['full']['near_peak_ms'],
              'halves', [h.get('delay_ms') for h in result['halves']])
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 5), layout='constrained')
        for result in results:
            r = result['full']
            ax.plot(r['delays_ms'], r['scores'], label=Path(result['ulog']).name)
        ax.axvline(270, color='red', ls='--', label='Configured: 270 ms')
        ax.axvline(225, color='black', ls=':', label='Candidate: 225 ms')
        ax.set(xlabel='VIO residual delay from timestamp_sample (ms)', ylabel='Yaw-increment correlation',
               title='VIO timing against onboard gyro — rotational timing diagnostic')
        ax.grid(alpha=.2)
        ax.legend()
        fig.savefig(args.out.with_suffix('.png'), dpi=150)


if __name__ == '__main__':
    main()
