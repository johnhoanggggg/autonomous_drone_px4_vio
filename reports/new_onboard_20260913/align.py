from pathlib import Path
import json,numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/john/autonomous_drone_px4_vio')
for stamp in ['125631','125730']:
 f=R/f'ros2_ws/flight_logs/offboard_global_20260913T{stamp}Z';b=json.loads((f/'render/bag_data.json').read_text());u=ULog(str(next(f.glob('px4*.ulg'))));d={x.name:x.data for x in u.data_list if x.multi_id==0};lp=d['vehicle_local_position'];sp=d['vehicle_local_position_setpoint'];pairs=[]
 for row in b['/fmu/out/vehicle_local_position_v1']:
  i=np.searchsorted(lp['timestamp'],row['timestamp'])
  if i<len(lp['timestamp'])and lp['timestamp'][i]==row['timestamp']:pairs.append((row,i))
 offset=np.median([row['t']-row['timestamp']/1e6 for row,i in pairs]);t=lp['timestamp']/1e6+offset;ts=sp['timestamp']/1e6+offset
 print(stamp,'pair',len(pairs),'maxx',max(abs(row['x']-lp['x'][i]) for row,i in pairs),'offset',offset)
 fig,ax=plt.subplots(4,1,figsize=(12,10),sharex=True)
 for k in ['x','y']:
  ax[0].plot(t,lp[k],label=f'actual NED {k}');ax[0].plot(ts,sp[k],'--',label=f'PX4 demand {k}')
 for k in ['vx','vy']:
  ax[1].plot(t,lp[k],label=f'actual {k}');ax[1].plot(ts,sp[k],'--',label=f'PX4 demand {k}')
 a=d['vehicle_attitude'];asp=d['vehicle_attitude_setpoint']
 def roll(z,prefix='q'):
  w,x,y,qz=[z[f'{prefix}[{i}]'] for i in range(4)];return np.arctan2(2*(w*x+y*qz),1-2*(x*x+y*y))
 ax[2].plot(a['timestamp']/1e6+offset,np.degrees(roll(a)),label='actual roll');ax[2].plot(asp['timestamp']/1e6+offset,np.degrees(roll(asp,'q_d')),label='desired roll')
 for name in ['estimator_aid_src_ev_pos']:
  z=d[name]
  for i in [0,1]:ax[3].plot(z['timestamp']/1e6+offset,z[f'innovation[{i}]']*100,label=f'EV innovation {i} cm')
 for x in ax:x.legend();x.grid(alpha=.2);x.set_xlim(8,32)
 ax[-1].set_xlabel('Bag time s');fig.suptitle(stamp+' demand and actual response');fig.tight_layout();fig.savefig(f/'render/onboard_tracking.png');plt.close(fig)
 (f/'render/onboard_pairing.json').write_text(json.dumps(dict(matches=len(pairs),max_x_difference=float(max(abs(row['x']-lp['x'][i]) for row,i in pairs)),boot_to_bag_s=float(offset)),indent=2))
