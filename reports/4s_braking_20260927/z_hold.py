"""Altitude-hold quality per flight: error vs setpoint, oscillation bands, thrust, VIO height noise."""
import sys
import numpy as np
from pyulog import ULog

def band(x, dt, lo, hi):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), dt); X[(f < lo) | (f > hi)] = 0
    return np.sqrt(np.mean(np.fft.irfft(X, len(x))**2))

def peak_freq(x, dt, lo=0.1, hi=5):
    X = np.abs(np.fft.rfft((x - x.mean())*np.hanning(len(x)))); f = np.fft.rfftfreq(len(x), dt)
    m = (f >= lo) & (f <= hi); return f[m][np.argmax(X[m])]

print('%-22s %5s %4s %4s %4s | z err rms p2p cm | osc cm 0.1-0.5/0.5-1.5/1.5-4 Hz | peak Hz | vz rms | thrust std | VIO z jitter mm | settle s' % (
    'flight', 'HOVER', 'ZP', 'ZVP', 'ZVI'))
for path in sys.argv[1:]:
    U = ULog(path); P = U.initial_parameters; T = {d.name: d.data for d in U.data_list if d.multi_id == 0}; s = lambda a: (a - U.start_timestamp)/1e6
    lp = T['vehicle_local_position']; tl = s(lp['timestamp']); alt = -lp['z']
    ts = T['trajectory_setpoint']; tt = s(ts['timestamp']); zsp = -ts['position[2]']; okz = np.isfinite(zsp)
    air = np.flatnonzero(alt > 0.25)
    if len(air) == 0: continue
    t_reach = tl[air[0]]; t0, t1 = t_reach + 3, tl[air[-1]] - 1
    dt = 0.02; g = np.arange(t0, t1, dt)
    err = np.interp(g, tl, alt) - np.interp(g, tt[okz], zsp[okz])
    vz = np.interp(g, tl, lp['vz'])
    th = T['vehicle_thrust_setpoint']; tth = s(th['timestamp']); mth = (tth > t0) & (tth < t1)
    ev = T['vehicle_visual_odometry']; te = s(ev['timestamp']); me = (te > t0) & (te < t1); ez = ev['position[2]'][me]
    jit = np.std(ez - np.convolve(ez, np.ones(5)/5, 'same')[:len(ez)]) if me.sum() > 10 else np.nan
    # settle: first time after reaching 0.25 m that |err| stays < 2 cm for 2 s
    gg = np.arange(t_reach, t1, dt); ee = np.abs(np.interp(gg, tl, alt) - np.interp(gg, tt[okz], zsp[okz]))
    settle = np.nan
    for i in range(len(gg)):
        j = i + int(2/dt)
        if j < len(gg) and np.all(ee[i:j] < 0.02): settle = gg[i] - t_reach; break
    name = path.split('px4_')[-1][:-4]
    print('%-22s %5.2f %4.1f %4.1f %4.1f |   %4.1f  %4.1f    |   %4.1f / %4.1f / %4.1f          |  %4.2f  | %.3f  |   %.4f    |     %.1f        | %s' % (
        name, P['MPC_THR_HOVER'], P['MPC_Z_P'], P['MPC_Z_VEL_P_ACC'], P['MPC_Z_VEL_I_ACC'], 100*np.sqrt(np.mean(err**2)), 100*np.ptp(err),
        100*band(err, dt, .1, .5), 100*band(err, dt, .5, 1.5), 100*band(err, dt, 1.5, 4), peak_freq(err, dt), np.sqrt(np.mean(vz**2)),
        np.std(-th['xyz[2]'][mth]), 1000*jit, 'n/a' if np.isnan(settle) else '%.1f' % settle))
