from pathlib import Path
import json,sys
import numpy as np
sys.path.insert(0,'scripts')
from pyulog import ULog
from analyze_control_response import recent_indices
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path('reports/bump_review_20260914');fig,axs=plt.subplots(4,2,figsize=(13,11),sharex=True,layout='constrained');results={}
for col,(name,pulse) in enumerate([('19_57_15',9.440121),('19_59_21',15.935202)]):
 u=ULog(f'ros2_ws/flight_logs/bump_candidates_20260914/px4_{name}.ulg');d={x.name:x.data for x in u.data_list if x.multi_id==0};base=u.start_timestamp/1e6
 lp=d['vehicle_local_position'];sp=d['vehicle_local_position_setpoint'];att=d['vehicle_attitude'];asp=d['vehicle_attitude_setpoint'];ts=d['trajectory_setpoint']
 stamp=lp['timestamp'];t=stamp.astype(float)/1e6-base-pulse
 si,sv=recent_indices(sp['timestamp'],stamp,200000);ai,av=recent_indices(att['timestamp'],stamp,50000);di,dv=recent_indices(asp['timestamp'],stamp,50000);ri,rv=recent_indices(ts['timestamp'],stamp,200000)
 def tilt(topic,idx,prefix):
  q=np.array([topic[f'{prefix}[{k}]'][idx] for k in range(4)],dtype=float);q/=np.linalg.norm(q,axis=0);w,x,y,z=q
  return -9.81*2*(x*z+w*y)/(1-2*(x*x+y*y))
 actual=tilt(att,ai,'q');desired=tilt(asp,di,'q_d')
 keep=sv&av&dv&rv&(t>=-.3)&(t<3.7)
 err=lp['x']-ts['position[0]'][ri]
 axs[0,col].plot(t[keep],err[keep],label='North position error');axs[0,col].axhline(0,color='gray',ls=':');axs[0,col].set_title(name)
 axs[1,col].plot(t[keep],lp['vx'][keep],label='Actual');axs[1,col].plot(t[keep],sp['vx'][si[keep]],label='Internal velocity demand')
 axs[2,col].plot(t[keep],sp['acceleration[0]'][si[keep]],label='Acceleration demand');axs[2,col].plot(t[keep],actual[keep],label='Actual tilt equivalent');axs[2,col].plot(t[keep],desired[keep],label='Desired tilt equivalent',ls='--')
 axs[2,col].set_ylim(-2,4)
 g=d['vehicle_angular_velocity'];gt=g['timestamp_sample']/1e6-base-pulse;rate=d['vehicle_rates_setpoint'];gi,gv=recent_indices(rate['timestamp'],g['timestamp'],10000);gm=gv&(gt>=-.3)&(gt<3.7)
 axs[3,col].plot(gt[gm],np.degrees(g['xyz[0]'][gm]),label='Actual roll rate');axs[3,col].plot(gt[gm],np.degrees(rate['roll'][gi[gm]]),label='Requested roll rate')
 axs[3,col].set_ylim(-70,50)
 stats={}
 for label,lo,hi in [('recovery',.3,1.3),('approach_reference',1.3,2.3),('after_crossing',2.3,3.3)]:
  m=keep&(t>=lo)&(t<hi);stats[label]={k:float(a[m].mean()) for k,a in {'v_actual':lp['vx'],'v_demand':sp['vx'][si],'a_demand':sp['acceleration[0]'][si],'desired_tilt':desired,'actual_tilt':actual}.items()}
 mot=d['actuator_motors'];mt=mot['timestamp']/1e6-base-pulse;mm=(mt>=0)&(mt<3.7);motor=np.array([mot[f'control[{k}]'][mm] for k in range(4)]);stats['motors']={'peak':float(motor.max()),'minimum':float(motor.min()),'fraction_above_095':float(np.mean(motor>.95))}
 ca=d['control_allocator_status'];ct=ca['timestamp']/1e6-base-pulse;cm=(ct>=0)&(ct<3.7);stats['allocation_failed_samples']=int(np.sum(~ca['torque_setpoint_achieved'][cm].astype(bool)))
 results[name]=stats
 for row,unit in enumerate(['m','m/s','m/s²','deg/s']):axs[row,col].set_ylabel(unit)
 axs[3,col].set_xlabel('Seconds after impulse peak')
for ax in axs.flat:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.axvline(0,color='orange',ls=':');ax.set_xlim(-.3,3.7)
fig.suptitle('Bump recovery with a fixed reference — approach, crossing and braking')
fig.savefig(out/'recovery_comparison.png',dpi=140)
(out/'recovery_comparison.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
