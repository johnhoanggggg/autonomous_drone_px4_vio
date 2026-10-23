from pathlib import Path
import json,sys,numpy as np,hashlib
from pyulog import ULog
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_control_response import analyse as control_analyse
from analyze_attitude_response import analyse as attitude_analyse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(5,2,figsize=(15,13));results=[]
for col,(r,name) in enumerate(zip(json.loads((OUT/'summary.json').read_text()),['23_10_12','23_11_12'])):
 f=r['flight'];path=ROOT/f'ros2_ws/flight_logs/offboard_lateral_stop_20260917T{f}Z/px4_{name}.ulg'
 if not path.exists():continue
 b=json.loads((OUT/f'{f}.data.json').read_text());u=ULog(str(path));ds={d.name:d.data for d in u.data_list if d.multi_id==0};lp=ds['vehicle_local_position'];by={round(x['boot']*1e6):x for x in b['local']};matches=[(by[int(ts)]['time']-ts/1e6,float(lp['x'][i]-by[int(ts)]['x'])) for i,ts in enumerate(lp['timestamp']) if int(ts) in by];offset=float(np.median([m[0] for m in matches]));lo,hi=r['active_range']
 windows=[]
 for l in r['legs']:windows.extend([(f"leg{l['leg']+1}_move",l['start_s'],l['reference_stop_s']),(f"leg{l['leg']+1}_hold",l['reference_stop_s'],l['hold_end_s'])])
 att,stats=attitude_analyse(path,offset,windows);ctl,cstats=control_analyse(path,offset,lo,hi);c,s=np.cos(b['debug'][0]['yaw0']),np.sin(b['debug'][0]['yaw0'])
 for label,x,y in [('v_actual','vx_actual','vy_actual'),('v_demand','vx_demand','vy_demand'),('a_demand','ax_demand','ay_demand'),('a_tilt','ax_tilt_equivalent','ay_tilt_equivalent')]:ctl[label]=-s*ctl[x]+c*ctl[y]
 for key,arr in [('control',ctl),('attitude',att)]:np.savez(OUT/f'{f}_{key}.npz',**arr)
 result=dict(flight=f,offset=offset,source_matches=len(matches),max_source_x_error=max(abs(m[1]) for m in matches),firmware=u.msg_info_dict.get('ver_sw'),dropouts=[dict(t=x.timestamp/1e6+offset,duration_ms=x.duration) for x in u.dropouts],windows=stats['windows'],parameters={k:v for k,v in u.initial_parameters.items() if k.startswith(('MPC_XY','MC_ROLL','MC_PITCH','MC_YAW','EKF2_EV_DELAY'))},esc_status_present='esc_status' in ds)
 for key,start,end in windows:
  result['windows'][key]['aux']={}
  for topic,fields in [('rate_ctrl_status',['rollspeed_integ','pitchspeed_integ','yawspeed_integ']),('control_allocator_status',['torque_setpoint_achieved','thrust_setpoint_achieved']),('actuator_motors',[f'control[{k}]' for k in range(4)]),('battery_status',['voltage_v','current_a'])]:
   d=ds[topic];t=d['timestamp']/1e6+offset;keep=(t>=start)&(t<end);result['windows'][key]['aux'][topic]={k:np.percentile(d[k][keep].astype(float),[0,50,100]).tolist() for k in fields}
  keep=(ctl['time_s']>=start)&(ctl['time_s']<end);result['windows'][key]['opposing_velocity_fraction']=float(np.mean((ctl['v_actual'][keep]*ctl['v_demand'][keep]<0)&(np.abs(ctl['v_actual'][keep])>.03)&(np.abs(ctl['v_demand'][keep])>.03)))
 # Candidate sustained disagreement windows for human review; no latency fit.
 candidates=[]
 for start in np.arange(lo,hi-.5,.1):
  keep=(ctl['time_s']>=start)&(ctl['time_s']<start+.5)
  if not keep.any():continue
  va,vd=ctl['v_actual'][keep].mean(),ctl['v_demand'][keep].mean()
  if va*vd<0 and min(abs(va),abs(vd))>.04:
   kk=(att['time_s']>=start)&(att['time_s']<start+.5)
   candidates.append(dict(start=float(start),end=float(start+.5),v_actual=float(va),v_demand=float(vd),a_demand=float(ctl['a_demand'][keep].mean()),a_tilt=float(ctl['a_tilt'][keep].mean()),roll_target=float(att['roll_target_deg'][kk].mean()),roll_actual=float(att['roll_actual_deg'][kk].mean()),rate_target=float(att['roll_rate_target_deg_s'][kk].mean()),rate_actual=float(att['roll_rate_actual_deg_s'][kk].mean())))
 result['disagreement_windows']=candidates
 for ax,series in [(axes[0,col],[(ctl['time_s'],ctl['v_actual'],'Measured lateral velocity'),(ctl['time_s'],ctl['v_demand'],'PX4 internal velocity demand')]),(axes[1,col],[(ctl['time_s'],ctl['a_demand'],'Acceleration demand'),(ctl['time_s'],ctl['a_tilt'],'Actual tilt equivalent (approx.)')]),(axes[2,col],[(att['time_s'],att['roll_target_deg'],'Roll target'),(att['time_s'],att['roll_actual_deg'],'Roll actual')]),(axes[3,col],[(att['time_s'],att['roll_rate_target_deg_s'],'Roll-rate target'),(att['time_s'],att['roll_rate_actual_deg_s'],'Roll-rate actual')])]:
  for t,v,label in series:ax.plot(t,v,label=label,lw=.9)
 ri=ds['rate_ctrl_status'];ti=ri['timestamp']/1e6+offset
 for k in ['rollspeed_integ','pitchspeed_integ','yawspeed_integ']:axes[4,col].plot(ti,ri[k],label=k,lw=.9)
 for i,label in enumerate(['Velocity / m/s','Acceleration / m/s²','Roll / deg','Roll rate / deg/s','Rate integral']):
  a=axes[i,col];a.set_xlim(lo-.2,hi+.2);a.set_ylabel(label);a.legend(fontsize=7);a.grid(alpha=.2)
  for l in r['legs']:a.axvspan(l['brake_s'],l['reference_stop_s'],color='orange',alpha=.15)
 axes[0,col].set_title(f);axes[4,col].set_xlabel('Bag-aligned seconds; all control pairing uses PX4 timestamps')
 results.append(result)
(OUT/'onboard.json').write_text(json.dumps(results,indent=2));fig.tight_layout();fig.savefig(OUT/'onboard.png',dpi=130)
for r in results:
 print(r['flight'],'matches',r['source_matches'],'offset',r['offset'],'xerror',r['max_source_x_error'],'dropouts',r['dropouts'],'esc',r['esc_status_present'])
 print('disagreement',sorted(r['disagreement_windows'],key=lambda x:abs(x['v_actual']-x['v_demand']),reverse=True)[:8])
