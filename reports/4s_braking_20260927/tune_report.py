"""One-stop tuning report for lateral/forward stop flights (ULog only).

Usage: .venv-mavlink/bin/python reports/4s_braking_20260927/tune_report.py <ulog> [<ulog> ...]

Per flight: key params; level/lean; stops and holds along the test axis;
the lean response at the first leg; altitude hold and takeoff settle; slow sway;
roll/pitch angle and rate loop response; noise, motor effort; battery and IMU temperature.
"""
import sys

import numpy as np
from pyulog import ULog

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from lat_ulog import analyse as legs_of  # noqa: E402

PARAMS = ['MC_ROLL_P', 'MC_PITCH_P', 'MC_ROLLRATE_D', 'MC_PITCHRATE_D', 'IMU_DGYRO_CUTOFF',
          'MPC_XY_P', 'MPC_XY_VEL_P_ACC', 'MPC_XY_VEL_I_ACC', 'MPC_Z_P', 'MPC_Z_VEL_P_ACC',
          'MPC_Z_VEL_I_ACC', 'MPC_THR_HOVER', 'SENS_BOARD_X_OFF', 'SENS_BOARD_Y_OFF']


def eul(q):
    r = np.arctan2(2*(q[:, 0]*q[:, 1]+q[:, 2]*q[:, 3]), 1-2*(q[:, 1]**2+q[:, 2]**2))
    p = np.arcsin(np.clip(2*(q[:, 0]*q[:, 2]-q[:, 3]*q[:, 1]), -1, 1))
    return r, p


def band(x, dt, lo, hi):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), dt); X[(f < lo) | (f > hi)] = 0
    return np.sqrt(np.mean(np.fft.irfft(X, len(x))**2))


def tf(x, y, fs, n=1024):
    w = np.hanning(n); sxy = sxx = syy = 0
    for i in range(0, len(x) - n, n // 2):
        X = np.fft.rfft((x[i:i+n]-x[i:i+n].mean())*w); Y = np.fft.rfft((y[i:i+n]-y[i:i+n].mean())*w)
        sxy = sxy + np.conj(X)*Y; sxx = sxx + np.abs(X)**2; syy = syy + np.abs(Y)**2
    f = np.fft.rfftfreq(n, 1/fs); H = sxy/sxx; C = np.abs(sxy)**2/(sxx*syy)
    return lambda q: (abs(np.interp(q, f, H.real) + 1j*np.interp(q, f, H.imag)), np.interp(q, f, C))


def fmt_h(H, freqs):
    return ' '.join('%.2f%s' % (H(q)[0], '' if H(q)[1] > .5 else '?') for q in freqs)


def report(path):
    U = ULog(path); P = U.initial_parameters; T = {d.name: d.data for d in U.data_list if d.multi_id == 0}
    s = lambda a: (a - U.start_timestamp)/1e6
    lp = T['vehicle_local_position']; tl = s(lp['timestamp']); alt = -lp['z']
    ts = T['trajectory_setpoint']; tt = s(ts['timestamp']); okp = np.isfinite(ts['position[0]'])
    sp = T['vehicle_local_position_setpoint']; tsp = s(sp['timestamp'])
    a = T['vehicle_attitude']; ta = s(a['timestamp']); roll, pitch = eul(np.column_stack([a[f'q[{i}]'] for i in range(4)]))
    asp = T['vehicle_attitude_setpoint']; tas = s(asp['timestamp']); rs, ps = eul(np.column_stack([asp[f'q_d[{i}]'] for i in range(4)]))
    rt = T['vehicle_rates_setpoint']; trt = s(rt['timestamp']); av = T['vehicle_angular_velocity']; tav = s(av['timestamp'])
    mot = T['actuator_motors']; tm = s(mot['timestamp']); M = np.column_stack([mot[f'control[{i}]'] for i in range(4)])
    bat = T['battery_status']; tb = s(bat['timestamp'])
    d = legs_of(path); L = d['legs']
    name = path.split('px4_')[-1][:-4]
    print('=' * 110)
    print(name, ' '.join('%s=%g' % (k.replace('MPC_', '').replace('MC_', ''), round(P[k], 4)) for k in PARAMS if k in P))
    if not L:
        print('  no legs found'); return

    # test axis from the first leg's reference velocity
    rv = np.nan_to_num(np.column_stack([ts['velocity[0]'], ts['velocity[1]']])); k = np.argmax(np.linalg.norm(rv, axis=1))
    u = rv[k]/np.linalg.norm(rv[k]); h = np.median(lp['heading']); fwd = np.array([np.cos(h), np.sin(h)]); right = np.array([-np.sin(h), np.cos(h)])
    axis = 'FORWARD' if abs(u @ fwd) > .9 else 'SIDEWAYS'
    t_leg0, t_end = L[0]['t_start'], L[-1]['t_hold_end']

    # --- level / lean
    g0 = (ta >= 0) & (ta < .4); m_all = (tsp >= t_leg0) & (tsp <= t_end)
    acc = np.column_stack([sp['acceleration[0]'], sp['acceleration[1]']])
    mt = (ta >= t_leg0) & (ta <= t_end)
    print('  LEVEL   ground roll/pitch %+.2f/%+.2f deg | hover roll/pitch %+.2f/%+.2f | lean (steady accel demand) side %+.3f fwd %+.3f m/s2' % (
        np.degrees(roll[g0].mean()), np.degrees(pitch[g0].mean()), np.degrees(roll[mt].mean()), np.degrees(pitch[mt].mean()),
        (acc[m_all] @ right).mean(), (acc[m_all] @ fwd).mean()))

    # --- stops and holds
    ovs = [100*l['overshoot'] for l in L]
    err = np.column_stack([lp['x'], lp['y']]) - np.column_stack([np.interp(tl, tt[okp], ts['position[0]'][okp]), np.interp(tl, tt[okp], ts['position[1]'][okp])])
    refv = np.column_stack([np.interp(tl, tt, rv[:, 0]), np.interp(tl, tt, rv[:, 1])]) @ u
    mh = (tl >= t_leg0) & (tl <= t_end) & (np.abs(refv) < 1e-3)
    print('  STOPS   %s | holds %s s | overshoot %s cm (mean %.1f) | hold max err %s cm | hold offset along/across %+.1f/%+.1f cm' % (
        axis, ' '.join('%.1f' % l['hold_s'] for l in L), ' '.join('%.1f' % o for o in ovs), np.mean(ovs),
        ' '.join('%.1f' % (100*l['hold_max_poserr']) for l in L), 100*(err[mh] @ u).mean(), 100*(err[mh] @ np.array([-u[1], u[0]])).mean()))
    i0 = np.searchsorted(tl, t_leg0)
    arrive_idx = np.flatnonzero((alt >= .28))
    arrive = tl[arrive_idx[0]] if len(arrive_idx) else np.nan
    print('  LEG-0   starts %.1f s after reaching 0.28 m | alt %.3f | position error %.1f cm | first hold max err %.1f cm' % (
        t_leg0 - arrive, alt[i0], 100*np.linalg.norm(err[i0]), 100*L[0]['hold_max_poserr']))

    # --- altitude
    zsp = np.nan_to_num(-ts['position[2]']); tr = tt[np.flatnonzero(zsp >= .299)[0]]
    dt = .02; g = np.arange(tr + 3, tl[np.flatnonzero(alt > .25)[-1]] - 1, dt)
    ze = np.interp(g, tl, alt) - .30
    gg = np.arange(tr, tr + 15, dt); ee = np.abs(np.interp(gg, tl, alt) - .30)
    def settle(tol):
        for i in range(len(gg)):
            j = i + 100
            if j < len(gg) and np.all(ee[i:j] < tol): return gg[i] - tr
        return np.nan
    vz_nonzero = np.mean(np.abs(np.nan_to_num(ts['velocity[2]'][(tt > arrive + 1) & (tt < t_end)])) > 1e-4) if np.isfinite(arrive) else np.nan
    print('  ALT     takeoff peak %.3f | 2 s after sp %+.3f m | settle 2 cm %.1f s, 1 cm %.1f s | hold rms %.1f p2p %.1f cm | wander 0.1-0.5 Hz %.1f, bob 0.5-1.5 Hz %.1f cm | vz cmd active %.0f%%' % (
        alt[(tl > tr - 3) & (tl < tr + 6)].max(), np.interp(tr + 2, tl, alt) - .30, settle(.02), settle(.01),
        100*np.sqrt(np.mean(ze**2)), 100*np.ptp(ze), 100*band(ze, dt, .1, .5), 100*band(ze, dt, .5, 1.5), 100*vz_nonzero))

    # --- slow sway (horizontal) during the test
    g2 = np.arange(t_leg0, t_end, dt)
    vr = np.interp(g2, tl, np.column_stack([lp['vx'], lp['vy']]) @ right); vf = np.interp(g2, tl, np.column_stack([lp['vx'], lp['vy']]) @ fwd)
    print('  SWAY    horizontal speed 0.3-1 Hz side %.3f fwd %.3f m/s | 1-3 Hz side %.3f fwd %.3f' % (
        band(vr, dt, .3, 1), band(vf, dt, .3, 1), band(vr, dt, 1, 3), band(vf, dt, 1, 3)))

    # --- inner loops
    fs = 250.; dtf = 1/fs; gf = np.arange(tl[np.flatnonzero(alt > .25)[0]] + 2, tl[np.flatnonzero(alt > .25)[-1]] - 1, dtf)
    for ax, A, As, rk, wk in (('roll', roll, rs, 'roll', 'xyz[0]'), ('pitch', pitch, ps, 'pitch', 'xyz[1]')):
        x = np.interp(gf, tas, As); y = np.interp(gf, ta, A)
        lag = max((np.corrcoef(x[:len(gf)-j], y[j:])[0, 1], j*dtf) for j in range(1, 150))[1]
        Ha = tf(x, y, fs); Hr = tf(np.interp(gf, trt, rt[rk]), np.interp(gf, tav, av[wk]), fs)
        print('  %-7s angle lag %.2f s err %.2f deg | angle resp 1/1.5/2 Hz %s | rate resp 0.5/1/2/3/4 Hz %s | gyro >10 Hz %.2f dps' % (
            ax.upper(), lag, np.degrees(np.sqrt(np.mean((x-y)**2))), fmt_h(Ha, (1, 1.5, 2)), fmt_h(Hr, (.5, 1, 2, 3, 4)),
            np.degrees(band(np.interp(gf, tav, av[wk]), dtf, 10, 100))))
    mm = (tm > gf[0]) & (tm < gf[-1])
    mb = (tb > gf[0]) & (tb < gf[-1])
    tmp = T['vehicle_imu_status']['temperature_accel'] if 'vehicle_imu_status' in T else np.array([np.nan])
    print('  EFFORT  motor jitter %.4f | motor peak %.2f mean %.2f | pack %.2f V/cell | IMU %.0f-%.0f C' % (
        np.sqrt(np.mean(np.diff(M[mm], axis=0)**2)), M[mm].max(), M[mm].mean(),
        np.median(bat['voltage_v'][mb])/P['BAT1_N_CELLS'], np.nanmin(tmp), np.nanmax(tmp)))


if __name__ == '__main__':
    for p in sys.argv[1:]:
        report(p)
