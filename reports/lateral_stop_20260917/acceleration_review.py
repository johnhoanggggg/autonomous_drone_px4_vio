"""Offline lateral acceleration audit; no hardware access or control changes."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
reviews=json.loads((OUT/'summary.json').read_text())
onboard=json.loads((OUT/'onboard.json').read_text())
fig,axes=plt.subplots(2,2,figsize=(13,7),sharex='col')
results=[]
for col,(r,o) in enumerate(zip(reviews,onboard)):
 f=r['flight'];raw=json.loads((OUT/f'{f}.data.json').read_text())
 yaw=raw['debug'][0]['yaw0'];c,s=np.cos(yaw),np.sin(yaw)
 lat=lambda x,y:-s*x+c*y
 path=next((ROOT/f'ros2_ws/flight_logs/offboard_lateral_stop_20260917T{f}Z').glob('*.ulg'))
 u=ULog(str(path),message_name_filter_list=['vehicle_local_position','vehicle_local_position_setpoint','trajectory_setpoint'])
 d=u.get_dataset('vehicle_local_position').data;t=d['timestamp']*1e-6+o['offset'];v=lat(d['vx'],d['vy']);ae=lat(d['ax'],d['ay'])
 sp=u.get_dataset('vehicle_local_position_setpoint').data;st=sp['timestamp']*1e-6+o['offset'];ad=lat(sp['acceleration[0]'],sp['acceleration[1]'])
 tr=u.get_dataset('trajectory_setpoint').data;tt=tr['timestamp']*1e-6+o['offset'];ff=lat(tr['acceleration[0]'],tr['acceleration[1]']);refv=lat(tr['velocity[0]'],tr['velocity[1]'])
 def window(a,b,sign=1):
  assert t[0]<a<b<t[-1]
  mask=(t>=a)&(t<=b);sm=(st>=a)&(st<=b)
  assert d['xy_valid'][mask].all() and d['v_xy_valid'][mask].all()
  va,vb=sign*np.interp([a,b],t,v)
  # Endpoint smoothing sensitivity: mean velocity within +/-50 ms.
  means=[sign*np.mean(v[np.abs(t-x)<=.05]) for x in (a,b)]
  return dict(interval_s=[a,b],velocity_start_m_s=float(va),velocity_end_m_s=float(vb),average_acceleration_m_s2=float((vb-va)/(b-a)),endpoint_smoothed_average_m_s2=float((means[1]-means[0])/(b-a)),logged_local_acceleration_mean_m_s2=float(sign*ae[mask].mean()),internal_acceleration_demand_mean_m_s2=float(sign*ad[sm].mean()),maximum_local_sample_gap_s=float(np.diff(t[mask]).max()))
 result=dict(flight=f,ulog=str(path.relative_to(ROOT)),offset=o['offset'],reference_peak_acceleration=.3,reference_ramp_average_acceleration=.2/r['config']['ramp_s'],legs=[])
 for l in r['legs']:
  sign=1 if l['direction']=='right' else -1
  result['legs'].append(dict(leg=l['leg']+1,direction=l['direction'],launch=window(l['start_s'],l['start_s']+r['config']['ramp_s'],sign),braking=window(l['brake_s'],l['reference_stop_s'],sign),first_hold_second=window(l['reference_stop_s'],l['reference_stop_s']+1,sign)))
  for ax in axes[:,col]:ax.axvspan(l['brake_s'],l['reference_stop_s'],color='orange',alpha=.16)
 result['selected_hold_windows']=[window(a,b) for a,b in ([(31.2,31.7),(38.1,38.6)] if col==0 else [(44.2,44.7)])]
 results.append(result)
 lo,hi=r['active_range'];grid=np.arange(lo+.1,hi-.1,.02)
 # Offline centered 200 ms velocity difference; not an independent accelerometer.
 acc=(np.interp(grid+.1,t,v)-np.interp(grid-.1,t,v))/.2
 axes[0,col].plot(t,v,label='PX4 estimated lateral velocity',lw=1)
 axes[0,col].plot(tt,refv,label='Trajectory velocity',lw=1)
 axes[1,col].plot(grid,acc,label='Velocity change / 0.20 s',lw=1)
 axes[1,col].plot(tt,ff,label='Trajectory acceleration feedforward',lw=1)
 axes[0,col].set_title(f'Flight {col+1} ({f})')
 for row,ax in enumerate(axes[:,col]):
  ax.set_xlim(lo,hi);ax.axhline(0,color='gray',lw=.6);ax.grid(alpha=.2);ax.legend(fontsize=8)
  ax.set_ylabel('Lateral velocity (m/s)' if row==0 else 'Lateral acceleration (m/s²)')
 axes[1,col].set_xlabel('Bag-aligned time (s); positive = initial-heading right')
fig.suptitle('Orange: scheduled braking. Feedforward is not total PX4 acceleration demand.')
fig.tight_layout();fig.savefig(OUT/'acceleration.png',dpi=150)
(OUT/'acceleration.json').write_text(json.dumps(results,indent=2)+'\n')
for r in results:
 print('Flight',r['flight'])
 for l in r['legs']:
  print(l['leg'],l['direction'],{k:round(l[k]['average_acceleration_m_s2'],3) for k in ['launch','braking','first_hold_second']},'endpoint smoothing delta',round(l['braking']['endpoint_smoothed_average_m_s2']-l['braking']['average_acceleration_m_s2'],3))
