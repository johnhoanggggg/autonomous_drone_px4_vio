"""Offline candidate bump-flight review. All times from individual ULog start."""
from pathlib import Path
import sys,json
import numpy as np
sys.path.insert(0,'scripts')
from pyulog import ULog
from analyze_attitude_response import euler_degrees
from analyze_control_response import recent_indices
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path('reports/bump_review_20260914')
base=ULog('ros2_ws/flight_logs/offboard_global_20260916T081709Z/px4_23_28_50.ulg')
results=[]
for path in sorted(Path('ros2_ws/flight_logs/bump_candidates_20260914').glob('*.ulg')):
 if path.stem=='px4_19_26_50':continue
 u=ULog(str(path));d={x.name:x.data for x in u.data_list if x.multi_id==0};origin=u.start_timestamp/1e6
 def time(topic,sample=False):
  a=d[topic];return a['timestamp_sample' if sample and 'timestamp_sample' in a else 'timestamp'].astype(float)/1e6-origin
 def arr(topic,fields):return np.column_stack([d[topic][f] for f in fields])
 lp=d['vehicle_local_position'];lt=time('vehicle_local_position');status=d['vehicle_status'];st=time('vehicle_status')
 si,sv=recent_indices(status['timestamp'],lp['timestamp'],1000000)
 ts=d['trajectory_setpoint'];rt=time('trajectory_setpoint');ri,rv=recent_indices(ts['timestamp'],lp['timestamp'],200000)
 pos=arr('vehicle_local_position',['x','y','z']);vel=arr('vehicle_local_position',['vx','vy','vz']);ref=arr('trajectory_setpoint',[f'position[{k}]' for k in range(3)])[ri]
 err=np.linalg.norm(pos[:,:2]-ref[:,:2],axis=1);speed=np.linalg.norm(vel[:,:2],axis=1)
 usable=sv&rv&(status['nav_state'][si]==14)&(status['arming_state'][si]==2)&(lt>=6)&(-pos[:,2]>.65)&np.all(np.isfinite(ref),axis=1)
 trans=np.flatnonzero((st>1)&((status['nav_state']!=14)|(status['arming_state']!=2)))
 end=float(st[trans[0]]) if len(trans) else float(lt[-1])
 accel=arr('vehicle_acceleration',[f'xyz[{k}]' for k in range(3)]);at=time('vehicle_acceleration',True);ah=np.linalg.norm(accel[:,:2],axis=1)
 idx=np.flatnonzero((ah>3)&(at>6)&(at<end))
 groups=np.split(idx,np.flatnonzero(np.diff(at[idx])>.45)+1) if len(idx) else []
 events=[]
 for group in groups:
  if len(group)==0:continue
  j=group[np.argmax(ah[group])];events.append({'start':float(at[group[0]]),'last_force':float(at[group[-1]]),'peak':float(at[j]),'peak_body_xy_accel':float(ah[j])})
 g=d['vehicle_angular_velocity'];gt=time('vehicle_angular_velocity',True);rate=arr('vehicle_angular_velocity',[f'xyz[{k}]' for k in range(3)])
 rs=d['vehicle_rates_setpoint'];rsi,rsv=recent_indices(rs['timestamp'],g['timestamp'],10000);rate_sp=arr('vehicle_rates_setpoint',['roll','pitch','yaw'])[rsi]
 attitude=np.radians(euler_degrees(arr('vehicle_attitude',[f'q[{k}]' for k in range(4)]).T));ast=time('vehicle_attitude',True)
 asp=d['vehicle_attitude_setpoint'];adt=time('vehicle_attitude_setpoint');target_angle=euler_degrees(arr('vehicle_attitude_setpoint',[f'q_d[{k}]' for k in range(4)]).T)
 for i,event in enumerate(events):
  begin=event['last_force']+.10;stop=events[i+1]['start'] if i+1<len(events) else end
  use=usable&(lt>=event['start']-.1)&(lt<stop)
  if not use.any():continue
  event.update(analysis_end=stop,max_xy_error=float(err[use].max()),max_xy_speed=float(speed[use].max()),target_xy_span=np.ptp(ref[use,:2],axis=0).tolist())
  # Require both gates continuously for 1 s, before next impulse/mode change.
  good=usable&(lt>=begin)&(lt<stop)&(err<=.10)&(speed<=.06)
  ranges=np.split(np.flatnonzero(good),np.flatnonzero(np.diff(np.flatnonzero(good))>1)+1)
  event['settled_1s_at']=None
  event['longest_joint_gate_s']=max([float(lt[a[-1]]-lt[a[0]]) for a in ranges if len(a)] or [0])
  for a in ranges:
   if len(a)>1 and lt[a[-1]]-lt[a[0]]>=1:
    event['settled_1s_at']=float(lt[a[0]]+1);break
  # Count a return only AFTER maximum displacement; early inside-gate samples
  # can precede the delayed position response to the shove.
  peak_error_idx=np.flatnonzero(use)[np.argmax(err[use])]
  event['peak_error_at']=float(lt[peak_error_idx])
  hits=np.flatnonzero(usable&(lt>lt[peak_error_idx])&(lt<stop)&(err<=.10))
  event['first_inside_10cm_at']=float(lt[hits[0]]) if len(hits) else None
  event['speed_at_return']=float(speed[hits[0]]) if len(hits) else None
  rm=rsv&(gt>=begin)&(gt<min(begin+1,stop))
  if rm.any():event['post_release_1s_rate']={axis:{'target_rms_deg_s':float(np.sqrt(np.mean(np.degrees(rate_sp[rm,k])**2))),'actual_rms_deg_s':float(np.sqrt(np.mean(np.degrees(rate[rm,k])**2))),'error_rms_deg_s':float(np.sqrt(np.mean(np.degrees(rate[rm,k]-rate_sp[rm,k])**2)))} for k,axis in enumerate(['roll','pitch'])}
 p=u.initial_parameters
 result=dict(ulog=str(path),duration_s=(u.last_timestamp-u.start_timestamp)/1e6,firmware=u.msg_info_dict.get('ver_sw'),end_offboard_s=end,events=events,dropouts=len(u.dropouts),parameter_differences={k:[v,base.initial_parameters.get(k)] for k,v in p.items() if k.startswith(('MC_','MPC_','EKF2_EV','SENS_BOARD','CAL_ACC','THR_MDL','PWM_MAIN','CA_ROTOR','IMU_')) and base.initial_parameters.get(k)!=v},changed_parameters=u.changed_parameters,reset_spans={k:int(np.ptp(lp[k][usable].astype(int))) for k in ['xy_reset_counter','vxy_reset_counter']} if usable.any() else {})
 result['hold_summary']=dict(reference_xy_span=np.ptp(ref[usable,:2],axis=0).tolist(),max_error=float(err[usable].max()),max_speed=float(speed[usable].max()),first_last_t=lt[usable][[0,-1]].tolist()) if usable.any() else {}
 bat=d['battery_status'];bt=time('battery_status');bm=(bt>6)&(bt<end);result['battery_v_p01_p50_p99']=np.percentile(bat['voltage_v'][bm],[1,50,99]).tolist() if bm.any() else []
 results.append(result)
 fig,axs=plt.subplots(5,1,figsize=(12,12),sharex=True,layout='constrained')
 axs[0].plot(lt,pos[:,0]-ref[:,0],label='North error');axs[0].plot(lt,pos[:,1]-ref[:,1],label='East error');axs[0].set_ylabel('Position error (m)')
 axs[1].plot(lt,vel[:,0],label='North velocity');axs[1].plot(lt,vel[:,1],label='East velocity');axs[1].set_ylabel('Velocity (m/s)')
 for k,axis in enumerate(['roll','pitch']):axs[2].plot(gt,np.degrees(rate[:,k]),label=axis+' gyro',lw=.6);axs[2].plot(gt,np.degrees(rate_sp[:,k]),label=axis+' target',lw=.8)
 axs[2].set_ylabel('Body rate (deg/s)')
 axs[3].plot(at,ah,label='Body XY specific force');axs[3].set_ylabel('m/s²')
 axs[4].plot(lt,-pos[:,2],label='Altitude');axs[4].plot(lt,-ref[:,2],label='Target altitude');axs[4].set_ylabel('m');axs[4].set_xlabel('Seconds from ULog start')
 for ax in axs:
  ax.set_xlim(5,min(end+1,lt[-1]));ax.grid(alpha=.2);ax.legend(fontsize=8,ncol=4)
  ax.axvline(end,color='red',ls='--')
  for event in events:ax.axvspan(event['start'],event['last_force']+.1,color='orange',alpha=.15)
 fig.suptitle(path.stem+' — candidate external disturbances; red = OFFBOARD/armed ends')
 fig.savefig(OUT/(path.stem+'.png'),dpi=130);plt.close(fig)
 print(json.dumps(result),flush=True)
(OUT/'analysis.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
