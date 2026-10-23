"""Offline bag exports + exact-paired ULogs. Run with .venv-mavlink/bin/python3."""
from pathlib import Path
import json
import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent

def idx(t, q):
    return np.clip(np.searchsorted(t, q, side='right') - 1, 0, len(t)-1)

def yaw(rows):
    return np.array([np.arctan2(2*(r['qw']*r['qz']+r['qx']*r['qy']),
                     1-2*(r['qy']**2+r['qz']**2)) for r in rows])

def xy(rows):
    return np.array([[r['x'],r['y']] for r in rows])

results = []
for stamp, axis, window in [('081620',0,(9,34)),('081709',0,(8.6,30.7))]:
    folder = ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{stamp}Z'
    b = json.loads((folder/'render/bag_data.json').read_text())
    u = ULog(str(next(folder.glob('px4*.ulg'))))
    d = {x.name:x.data for x in u.data_list if x.multi_id == 0}
    lp = d['vehicle_local_position']; sp = d['vehicle_local_position_setpoint']
    pairs = []
    for r in b['/fmu/out/vehicle_local_position_v1']:
        i = np.searchsorted(lp['timestamp'],r['timestamp'])
        if i<len(lp['timestamp']) and lp['timestamp'][i]==r['timestamp']:
            pairs.append((r,i))
    off = float(np.median([r['t']-r['timestamp']/1e6 for r,i in pairs]))
    pairing = dict(matches=len(pairs),max_x_error=max(abs(r['x']-float(lp['x'][i])) for r,i in pairs),boot_to_bag_s=off)
    (folder/'render/onboard_pairing.json').write_text(json.dumps(pairing,indent=2))
    t = lp['timestamp']/1e6+off
    si = idx(sp['timestamp'],lp['timestamp'])
    c = b['/rtabmap/odom_correction']; ct=np.array([r['t'] for r in c]); cy=yaw(c)
    angle=cy[idx(ct,t)]
    # NED north/east -> continuous ENU -> current map axes, rotation only.
    def lateral(north,east):
        return east*np.cos(angle)-north*np.sin(angle) if axis==0 else east*np.sin(angle)+north*np.cos(angle)
    vel=lateral(lp['vx'],lp['vy'])
    vd=lateral(sp['vx'][si],sp['vy'][si])
    ad=lateral(sp['acceleration[0]'][si],sp['acceleration[1]'][si])
    at=d['vehicle_attitude']; ai=idx(at['timestamp'],lp['timestamp'])
    w,x,y,z=[at[f'q[{i}]'][ai] for i in range(4)]
    bz=1-2*(x*x+y*y)
    tilt=lateral(-9.81*2*(x*z+w*y)/bz,-9.81*2*(y*z-w*x)/bz)
    ac=d['vehicle_acceleration']; aci=idx(ac['timestamp'],lp['timestamp'])
    fx,fy,fz=[ac[f'xyz[{i}]'][aci] for i in range(3)]
    an=(1-2*(y*y+z*z))*fx+2*(x*y-w*z)*fy+2*(x*z+w*y)*fz
    ae=2*(x*y+w*z)*fx+(1-2*(x*x+z*z))*fy+2*(y*z-w*x)*fz
    imu=lateral(an,ae)
    vs=d['vehicle_status']; vi=idx(vs['timestamp'],lp['timestamp'])
    valid=(t>=window[0])&(t<=window[1])&(vs['arming_state'][vi]==2)&(lp['timestamp']-sp['timestamp'][si]<200000)
    pose=b['/rtabmap/body_pose']; raw=b['/rtabmap/body_vio_pose'];pt=np.array([r['t'] for r in pose]);rt=np.array([r['t'] for r in raw])
    pxy=xy(pose);rxy=xy(raw)
    # Map change at a fixed raw pose: isolates correction from intervening motion.
    events=[]
    for j in range(1,len(c)):
        rr=rxy[idx(rt,ct[j])]
        def transform(k):
            co,so=np.cos(cy[k]),np.sin(cy[k])
            return np.array([co*rr[0]-so*rr[1]+c[k]['x'],so*rr[0]+co*rr[1]+c[k]['y']])
        step=transform(j)-transform(j-1)
        if np.linalg.norm(step)>.025:
            events.append(dict(t=float(ct[j]),fixed_pose_map_step=step.tolist(),translation_step=float(np.linalg.norm(xy(c)[j]-xy(c)[j-1])),yaw_step_deg=float(np.degrees(cy[j]-cy[j-1]))))
    rows=b['/fmu/in/trajectory_setpoint']; st=np.array([r['t'] for r in rows]);spos=np.array([r['position'][:2] for r in rows]);sv=np.array([r['velocity'][:2] for r in rows]);dt=np.diff(st)
    keep=(st[1:]>=9)&(st[1:]<=window[1])&(dt>.035)&(dt<.070)&np.isfinite(sv[1:]).all(axis=1)&np.isfinite(sv[:-1]).all(axis=1)
    dv=np.linalg.norm(np.diff(sv,axis=0),axis=1)
    jumps=np.linalg.norm(np.diff(spos,axis=0)-sv[1:]*dt[:,None],axis=1)
    err=np.hypot(lp['x']-sp['x'][si],lp['y']-sp['y'][si])
    result=dict(flight=stamp,pairing=pairing,window=window,peak_speed=float(np.hypot(lp['vx'][valid],lp['vy'][valid]).max()),peak_position_error=float(err[valid].max()),position_error_p90=float(np.quantile(err[valid],.9)),xy_reset=np.unique(lp['xy_reset_counter'][valid]).tolist(),vxy_reset=np.unique(lp['vxy_reset_counter'][valid]).tolist(),correction_events=events,max_finite_velocity_step=float(dv[keep].max()),max_position_secant_residual=float(jumps[keep].max()),dropouts=[dict(timestamp=int(x.timestamp),duration=int(x.duration)) for x in u.dropouts],parameters={k:v for k,v in u.initial_parameters.items() if k in ['EKF2_EV_DELAY','MC_ROLLRATE_P','MC_PITCHRATE_P']})
    for name in ['estimator_aid_src_ev_pos','estimator_aid_src_ev_yaw','control_allocator_status']:
        a=d[name];tt=a['timestamp']/1e6+off;mask=(tt>=window[0])&(tt<=window[1])
        result[name]={k:float(np.max(np.abs(v[mask]))) for k,v in a.items() if k.startswith(('innovation[','unallocated_torque[','unallocated_thrust[')) or k in ['innovation','torque_setpoint_achieved','thrust_setpoint_achieved']}
        for k in ['innovation_rejected','fused','torque_setpoint_achieved','thrust_setpoint_achieved']:
            if k in a:result[name][k+'_count']=int(a[k][mask].sum())
        result[name]['samples']=int(mask.sum())
    result['bins']=[]
    for start in np.arange(window[0],window[1],.5):
        m=valid&(t>=start)&(t<start+.5)
        if not m.any():continue
        result['bins'].append(dict(t=float(start),v=float(vel[m].mean()),v_demand=float(vd[m].mean()),a_demand=float(ad[m].mean()),a_tilt=float(tilt[m].mean()),a_imu=float(imu[m].mean()),position_error=float(err[m].mean())))
    fig,ax=plt.subplots(4,1,figsize=(13,11),sharex=True)
    ax[0].plot(pt,pxy[:,axis],label='corrected map body position')
    # Frozen map transform makes raw trajectory shape visible without correction jumps.
    ci=idx(ct,window[0]);co,so=np.cos(cy[ci]),np.sin(cy[ci]);frozen=np.column_stack((co*rxy[:,0]-so*rxy[:,1]+c[ci]['x'],so*rxy[:,0]+co*rxy[:,1]+c[ci]['y']))
    ax[0].plot(rt,frozen[:,axis],label='raw VIO, map transform frozen at window start',alpha=.7)
    ax[0].set_ylabel('Lateral coordinate (m)')
    ax[1].plot(t,vel,label='actual EKF velocity');ax[1].plot(t,vd,label='PX4 internal demand');ax[1].set_ylabel('Lateral velocity (m/s)')
    ax[2].plot(t,ad,label='PX4 acceleration demand');ax[2].plot(t,tilt,label='actual tilt equivalent');ax[2].plot(t,imu,label='rotated IMU acceleration',alpha=.25);ax[2].set_ylim(-1,1);ax[2].set_ylabel('Lateral acceleration (m/s²)')
    ax[3].plot(t,err*100,label='XY error against PX4 internal position demand');ax[3].set_ylabel('Demand error (cm)')
    for i,a in enumerate(ax):
        if i>0:a.axhline(0,color='gray',lw=.5)
        a.grid(alpha=.2);a.legend(loc='upper left');a.set_xlim(*window)
        for e in events:
            if np.linalg.norm(e['fixed_pose_map_step'])>.08:a.axvline(e['t'],color='purple',alpha=.5)
    ax[-1].set_xlabel('Seconds since exported bag origin')
    fig.suptitle(f'{stamp} — lateral map {"x" if axis==0 else "y"}; purple = correction >8 cm at vehicle')
    fig.tight_layout();fig.savefig(OUT/f'{stamp}_response.png');plt.close(fig)
    np.savez(OUT/f'{stamp}_series.npz',t=t,v=vel,vd=vd,ad=ad,tilt=tilt,imu=imu,error=err,valid=valid)
    results.append(result)
(OUT/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
for r in results:
    print(json.dumps({k:v for k,v in r.items() if k!='bins'},indent=2))
    print('HALF SECOND BINS: t, velocity, demand velocity, demand accel, tilt accel, IMU accel')
    for row in r['bins']:print(' '.join(f'{row[k]:.3f}' for k in ['t','v','v_demand','a_demand','a_tilt','a_imu']))
