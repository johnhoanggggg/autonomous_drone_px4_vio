"""Per-leg braking asymmetry: rightward vs leftward stops (lateral-stop test)."""
import json, sys
import numpy as np
from pyulog import ULog

def eul(q):
    r = np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]), 1-2*(q[:,1]**2+q[:,2]**2))
    return r

rows = []
for d in json.load(open(sys.argv[1])):
    U = ULog(d['path']); T = {x.name: x.data for x in U.data_list if x.multi_id == 0}; s = lambda a: (a-U.start_timestamp)/1e6
    lp = T['vehicle_local_position']; tl = s(lp['timestamp'])
    sp = T['vehicle_local_position_setpoint']; tsp = s(sp['timestamp'])
    a = T['vehicle_attitude']; ta = s(a['timestamp']); roll = np.degrees(eul(np.column_stack([a[f'q[{i}]'] for i in range(4)])))
    asp = T['vehicle_attitude_setpoint']; tas = s(asp['timestamp']); roll_sp = np.degrees(eul(np.column_stack([asp[f'q_d[{i}]'] for i in range(4)])))
    rt = T['vehicle_rates_setpoint']; trt = s(rt['timestamp']); av = T['vehicle_angular_velocity']; tav = s(av['timestamp'])
    tq = T['vehicle_torque_setpoint']; ttq = s(tq['timestamp'])
    mot = T['actuator_motors']; tm = s(mot['timestamp']); M = np.column_stack([mot[f'control[{i}]'] for i in range(4)])
    # PX4 quad X: motor0 front-right, 1 rear-left, 2 front-left, 3 rear-right
    right_side = M[:, 0] + M[:, 3]; left_side = M[:, 1] + M[:, 2]
    w, x, y, z = (a[f'q[{k}]'] for k in range(4))
    bz = np.column_stack([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]); tilt_acc = -9.81*bz[:, :2]/bz[:, 2:3]
    for L in d['legs']:
        sign = L['direction']  # +1 moving right, -1 moving left
        h = np.interp(L['t_brake'], tl, np.unwrap(lp['heading'])); r_ax = np.array([-np.sin(h), np.cos(h)])
        b0, b1 = L['t_brake'], L['t_ref_stop'] + 0.6          # braking window
        mb = (tsp >= b0) & (tsp <= b1)
        dem = (np.column_stack([sp['acceleration[0]'][mb], sp['acceleration[1]'][mb]]) @ r_ax) * sign   # <0 = braking
        ma = (ta >= b0) & (ta <= b1)
        til = (tilt_acc[ma] @ r_ax) * sign
        v = (np.column_stack([lp['vx'], lp['vy']]) @ r_ax) * sign
        mv = (tl >= b0) & (tl <= b1)
        dec = np.polyfit(tl[mv], v[mv], 1)[0]
        # roll tracking: lag of roll behind roll_sp in the braking window
        g = np.arange(b0, b1, 0.004); R = np.interp(g, ta, roll); Rs = np.interp(g, tas, roll_sp)
        lag = max((np.corrcoef(Rs[:len(g)-j], R[j:])[0, 1], j*0.004) for j in range(1, 100))[1]
        err = np.mean(np.abs(R - Rs))
        mr = (tav >= b0) & (tav <= b1)
        rsp = np.interp(tav[mr], trt, rt['roll']); ract = av['xyz[0]'][mr]
        big = np.abs(rsp) > np.radians(3)
        deliv = np.mean(ract[big]/rsp[big]) if big.sum() > 10 else np.nan
        mt = (ttq >= b0) & (ttq <= b1); mm = (tm >= b0) & (tm <= b1)
        rows.append(dict(flight=d['path'][-12:-4], leg=L['leg'], dirn='right' if sign > 0 else 'left',
                         v_brake=L['v_at_brake'], v_stop=L['v_at_ref_stop'], overshoot=100*L['overshoot'],
                         dem=dem.mean(), tilt=til.mean(), dec=dec, lag=lag, roll_err=err, deliv=deliv,
                         torque=np.abs(tq['xyz[0]'][mt]).max(), mot_max=M[mm].max(), side_diff=(right_side-left_side)[mm].mean()))
print('%-9s leg dir   v@brake v@stop ovs cm | demand tilt measured (m/s2, - = braking) | roll lag s  |err| deg  rate deliv | max|tq| mot max  R-L motors' % 'flight')
for r in rows:
    print('%-9s %d  %-5s  %+.2f  %+.2f  %5.1f | %+.2f %+.2f %+.2f | %.3f  %.2f  %.2f | %.3f %.2f %+.3f' % (
        r['flight'], r['leg'], r['dirn'], r['v_brake'], r['v_stop'], r['overshoot'], r['dem'], r['tilt'], r['dec'], r['lag'], r['roll_err'], r['deliv'], r['torque'], r['mot_max'], r['side_diff']))
for dn in ('right', 'left'):
    sel = [r for r in rows if r['dirn'] == dn]
    print('%-5s (n=%d): overshoot %.1f cm | v@stop %+.3f | demand %+.2f tilt %+.2f measured %+.2f | roll lag %.3f s err %.2f deg deliv %.2f | R-L motors %+.3f' % (
        dn, len(sel), np.mean([r['overshoot'] for r in sel]), np.mean([r['v_stop'] for r in sel]), np.mean([r['dem'] for r in sel]), np.mean([r['tilt'] for r in sel]),
        np.mean([r['dec'] for r in sel]), np.mean([r['lag'] for r in sel]), np.mean([r['roll_err'] for r in sel]), np.nanmean([r['deliv'] for r in sel]), np.mean([r['side_diff'] for r in sel])))
