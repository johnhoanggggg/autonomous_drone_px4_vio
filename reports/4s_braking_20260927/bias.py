import sys, json
import numpy as np
from pyulog import ULog
def eul(q):
    r=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)))
    p=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,2]-q[:,3]*q[:,1]),-1,1)))
    return r,p
for path in sys.argv[1:]:
    U=ULog(path); T={d.name:d.data for d in U.data_list if d.multi_id==0}
    s=lambda a:(a-U.start_timestamp)/1e6
    ts=T['trajectory_setpoint']; tt=s(ts['timestamp'])
    mv=np.flatnonzero(np.nan_to_num(np.hypot(ts['velocity[0]'],ts['velocity[1]']))>1e-4)
    t0=tt[mv[0]]
    cmd=T['vehicle_command']; land=[s(a) for a,c in zip(cmd['timestamp'],cmd['command']) if int(c)==21]
    t1=min(land) if land else tt[-1]
    a=T['vehicle_attitude']; ta=s(a['timestamp']); r,p=eul(np.column_stack([a[f'q[{i}]'] for i in range(4)]))
    asp=T['vehicle_attitude_setpoint']; tas=s(asp['timestamp']); rs,ps=eul(np.column_stack([asp[f'q_d[{i}]'] for i in range(4)]))
    m=(ta>t0)&(ta<t1); ms=(tas>t0)&(tas<t1)
    sp=T['vehicle_local_position_setpoint']; tsp=s(sp['timestamp']); msp=(tsp>t0)&(tsp<t1)
    lp=T['vehicle_local_position']; hd=np.median(lp['heading'][(s(lp['timestamp'])>t0)&(s(lp['timestamp'])<t1)])
    # body-frame accel demand: right = -sin(h)*ax + cos(h)*ay
    ar=-np.sin(hd)*sp['acceleration[0]'][msp]+np.cos(hd)*sp['acceleration[1]'][msp]
    af=np.cos(hd)*sp['acceleration[0]'][msp]+np.sin(hd)*sp['acceleration[1]'][msp]
    rt=T['vehicle_rates_setpoint']; trt=s(rt['timestamp']); av=T['vehicle_angular_velocity']; tav=s(av['timestamp'])
    w=(tav>t0)&(tav<t1)
    rx=np.interp(tav[w],trt,rt['roll'])
    ratio=np.degrees(np.sqrt(np.mean(av['xyz[0]'][w]**2)))/np.degrees(np.sqrt(np.mean(rx**2)))
    tq=T['vehicle_torque_setpoint']; ttq=s(tq['timestamp']); mt=(ttq>t0)&(ttq<t1)
    print('%-10s cells %d X_OFF %+.2f | roll mean %+.2f sp %+.2f | pitch mean %+.2f sp %+.2f | acc dem right mean %+.3f fwd %+.3f m/s2 | roll-rate act/sp rms %.2f | |roll torque| %.3f'%(
        path.split('/')[-1][-12:-4], U.initial_parameters['BAT1_N_CELLS'], U.initial_parameters['SENS_BOARD_X_OFF'],
        r[m].mean(), rs[ms].mean(), p[m].mean(), ps[ms].mean(), ar.mean(), af.mean(), ratio, np.abs(tq['xyz[0]'][mt]).mean()))
