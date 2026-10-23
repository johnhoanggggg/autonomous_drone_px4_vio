"""Does pack voltage matter on 4S? Thrust-vs-voltage fit, what MC_BAT_SCALE_EN would remove,
and paired (same-params, back-to-back) flight comparisons full vs lower pack.

Usage: .venv-mavlink/bin/python reports/4s_braking_20260927/voltage_effect.py
"""
import glob
import sys

import numpy as np
from pyulog import ULog

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from lat_ulog import analyse as legs_of  # noqa: E402
from tune_report import tf  # noqa: E402

KEYS = ['MC_ROLL_P', 'MC_PITCH_P', 'MC_ROLLRATE_P', 'MC_PITCHRATE_P', 'MC_ROLLRATE_D', 'MC_PITCHRATE_D',
        'MC_PITCHRATE_I', 'IMU_DGYRO_CUTOFF', 'MPC_XY_P', 'MPC_XY_VEL_P_ACC', 'MPC_XY_VEL_I_ACC', 'MPC_Z_P',
        'MPC_Z_VEL_I_ACC', 'MPC_THR_HOVER']


def flight(p):
    U = ULog(p); T = {d.name: d.data for d in U.data_list if d.multi_id == 0}; P = U.initial_parameters
    s = lambda a: (a - U.start_timestamp)/1e6
    lp = T['vehicle_local_position']; tl = s(lp['timestamp']); alt = -lp['z']
    air = np.flatnonzero(alt > .25)
    if len(air) < 50:
        return None
    t0, t1 = tl[air[0]] + 3, tl[air[-1]] - 1
    if t1 - t0 < 8:
        return None
    b = T['battery_status']; tb = s(b['timestamp']); mb = (tb > t0) & (tb < t1)
    th = T['vehicle_thrust_setpoint']; tth = s(th['timestamp']); mth = (tth > t0) & (tth < t1)
    r = dict(name=p.split('px4_')[1][:-4], vc=np.median(b['voltage_v'][mb])/P['BAT1_N_CELLS'],
             scale=b['scale'][mb].mean(), thrust=-th['xyz[2]'][mth].mean(), alt=alt[air].max(),
             params=tuple(round(P.get(k, np.nan), 3) for k in KEYS))
    try:
        L = legs_of(p)['legs']
    except Exception:
        L = []
    if len(L) == 4:
        r.update(overshoot=100*np.mean([l['overshoot'] for l in L]), holderr=100*np.mean([l['hold_max_poserr'] for l in L]),
                 hold_s=np.mean([l['hold_s'] for l in L]))
        rt = T['vehicle_rates_setpoint']; av = T['vehicle_angular_velocity']
        fs = 250.; g = np.arange(tl[air[0]] + 2, tl[air[-1]] - 1, 1/fs)
        for ax, rk, wk in (('roll', 'roll', 'xyz[0]'), ('pitch', 'pitch', 'xyz[1]')):
            H = tf(np.interp(g, s(rt['timestamp']), rt[rk]), np.interp(g, s(av['timestamp']), av[wk]), fs)
            r[ax + '_rate2hz'] = H(2)[0]; r[ax + '_rate4hz'] = H(4)[0]
        z = np.interp(np.arange(L[0]['t_start'], L[-1]['t_hold_end'], .02), tl, alt)
        r['alt_rms'] = 100*np.sqrt(np.mean((z - .30)**2)) if r['alt'] < .5 else np.nan
    return r


def main():
    F = [f for f in (flight(p) for p in sorted(glob.glob('ros2_ws/flight_logs/px4_2026092[78]_*.ulg'))) if f]
    v = np.array([f['vc'] for f in F]); th = np.array([f['thrust'] for f in F]); sc = np.array([f['scale'] for f in F])
    low = np.array([f['alt'] < .5 for f in F])
    print('THRUST vs PACK (%d flights, %.2f-%.2f V/cell in air)' % (len(F), v.min(), v.max()))
    for lab, y in (('thrust needed now', th), ('controller output if scale on', th/sc)):
        c = np.polyfit(v[low], y[low], 1)
        print('  %-30s %.3f @3.90  %.3f @3.70  slope %+.3f per V/cell  (%.1f%% change over 3.90->3.70)' % (
            lab, np.polyval(c, 3.9), np.polyval(c, 3.7), c[0], 100*(np.polyval(c, 3.7)/np.polyval(c, 3.9) - 1)))
    print('  PX4 scale factor over these flights %.3f-%.3f' % (sc.min(), sc.max()))

    print('\nPAIRED FLIGHTS (same day, identical params, both 4 legs, next completed flight): LOWER pack minus HIGHER pack')
    hdr = ['vc', 'overshoot', 'holderr', 'hold_s', 'alt_rms', 'roll_rate2hz', 'pitch_rate2hz', 'roll_rate4hz', 'pitch_rate4hz']
    print('  %-36s ' % 'pair' + ' '.join('%9s' % h[:9] for h in hdr))
    C = [f for f in F if 'overshoot' in f]
    D = []
    for a, b in zip(C, C[1:]):
        if a['params'] == b['params'] and a['name'][:8] == b['name'][:8] and abs(a['vc'] - b['vc']) > .02:
            hi, lo = (a, b) if a['vc'] > b['vc'] else (b, a)
            d = [lo[h] - hi[h] for h in hdr]; D.append(d)
            print('  %-36s ' % (a['name'] + ' / ' + b['name'][9:]) + ' '.join('%+9.3f' % x for x in d))
    D = np.array(D)
    print('  %-36s ' % 'mean' + ' '.join('%+9.3f' % x for x in np.nanmean(D, 0)))
    print('  %-36s ' % 'lower pack larger' + ' '.join('%9s' % ('%d/%d' % ((D[:, i] > 0).sum(), np.isfinite(D[:, i]).sum())) for i in range(len(hdr))))
    print('  (overshoot/holderr cm, hold_s s, alt_rms cm: larger = worse; rate columns = closed-loop rate gain, 1.0 ideal)')

if __name__ == '__main__':
    main()
