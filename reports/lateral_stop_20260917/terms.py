from pathlib import Path
import sys,json,numpy as np
from pyulog import ULog
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_control_response import recent_indices
from analyze_attitude_response import euler_degrees
results=[]
for r,name,windows in zip(json.loads((OUT/'onboard.json').read_text()),['23_10_12','23_11_12'],[[('early_hold',18.,19.),('late_left_stop',31.2,31.7),('hold_return',38.1,38.6)],[('early_hold',21.,22.),('late_return',44.2,44.7)]]):
 f=r['flight'];u=ULog(str(ROOT/f'ros2_ws/flight_logs/offboard_lateral_stop_20260917T{f}Z/px4_{name}.ulg'));d={x.name:x.data for x in u.data_list if x.multi_id==0};p=u.initial_parameters;off=r['offset'];b=json.loads((OUT/f'{f}.data.json').read_text());c,s=np.cos(b['debug'][0]['yaw0']),np.sin(b['debug'][0]['yaw0']);lat=lambda n,e:-s*n+c*e
 lp,sp,tr=d['vehicle_local_position'],d['vehicle_local_position_setpoint'],d['trajectory_setpoint'];lt=lp['timestamp']/1e6+off;st=sp['timestamp']/1e6+off;ii,valid=recent_indices(lp['timestamp'],sp['timestamp'],20000);ri,rv=recent_indices(tr['timestamp'],sp['timestamp'],200000)
 assert p['MPC_VEL_LP']==0 and p['MPC_VEL_NF_FRQ']==0
 error=lat(sp['vx']-lp['vx'][ii],sp['vy']-lp['vy'][ii]);ff=lat(np.nan_to_num(tr['acceleration[0]'][ri],nan=0),np.nan_to_num(tr['acceleration[1]'][ri],nan=0));acc=lat(sp['acceleration[0]'],sp['acceleration[1]']);vr=lat(lp['vx'],lp['vy']);prop=p['MPC_XY_VEL_P_ACC']*error;res=[]
 for half in [.025,.05,.10]:
  derivative=(np.interp(st+half,lt,vr)-np.interp(st-half,lt,vr))/(2*half);res.append(acc-ff-prop+p['MPC_XY_VEL_D_ACC']*derivative)
 tq,g,rate,integ=d['vehicle_torque_setpoint'],d['vehicle_angular_velocity'],d['vehicle_rates_setpoint'],d['rate_ctrl_status'];tt=tq['timestamp']/1e6+off;gi=np.clip(np.searchsorted(g['timestamp_sample'],tq['timestamp_sample']),0,len(g['timestamp'])-1);ri,rv2=recent_indices(rate['timestamp'],tq['timestamp'],10000);ii,iv=recent_indices(integ['timestamp'],tq['timestamp'],10000);good=(g['timestamp_sample'][gi]==tq['timestamp_sample'])&rv2&iv
 pt=p['MC_ROLLRATE_K']*p['MC_ROLLRATE_P']*(rate['roll'][ri]-g['xyz[0]'][gi]);dt=-p['MC_ROLLRATE_K']*p['MC_ROLLRATE_D']*g['xyz_derivative[0]'][gi];it=integ['rollspeed_integ'][ii];torque=tq['xyz[0]'];out=dict(flight=f,windows={})
 a=d['vehicle_attitude'];at=a['timestamp_sample']/1e6+off;ae=np.radians(euler_degrees(np.array([a[f'q[{i}]'] for i in range(4)])));gt=g['timestamp_sample']/1e6+off;roll=np.interp(gt,at,ae[0]);pitch=np.interp(gt,at,ae[1]);er=g['xyz[0]']+np.sin(roll)*np.tan(pitch)*g['xyz[1]']+np.cos(roll)*np.tan(pitch)*g['xyz[2]']
 for label,lo,hi in windows:
  mask=valid&rv&(st>=lo)&(st<hi);m=good&(tt>=lo)&(tt<hi);gm=(gt>=lo)&(gt<hi)
  out['windows'][label]=dict(interval=[lo,hi],velocity_loop=dict(p=float(prop[mask].mean()),acceleration_demand=float(acc[mask].mean()),feedforward=float(ff[mask].mean()),inferred_i=[float(x[mask].mean()) for x in res]),roll_rate_loop=dict(samples=int(m.sum()),target_deg_s=float(np.degrees(rate['roll'][ri][m]).mean()),actual_deg_s=float(np.degrees(g['xyz[0]'][gi][m]).mean()),p=float(pt[m].mean()),i=float(it[m].mean()),d=float(dt[m].mean()),torque=float(torque[m].mean()),reconstruction_error_p99=float(np.quantile(abs(torque[m]-pt[m]-it[m]-dt[m]),.99))),gyro_integrated_roll_change_deg=float(np.degrees(np.trapezoid(er[gm],gt[gm]))),attitude_roll_change_deg=float(np.degrees(np.interp(gt[gm][-1],at,ae[0])-np.interp(gt[gm][0],at,ae[0]))))
 mot=d['actuator_motors'];ti=np.clip(np.searchsorted(tq['timestamp_sample'],mot['timestamp_sample']),0,len(tt)-1);motor=np.column_stack([mot[f'control[{k}]'] for k in range(4)]);mt=mot['timestamp']/1e6+off;lo,hi=b['debug'][0]['time'],b['debug'][-1]['time'];review=json.loads((OUT/'summary.json').read_text());lo,hi=next(x['active_range'] for x in review if x['flight']==f);m=(tq['timestamp_sample'][ti]==mot['timestamp_sample'])&(mt>=lo)&(mt<=hi);coefs=[np.array([-1,1,1,-1])/(2*np.sqrt(2)),np.array([1,-1,1,-1])/(2*np.sqrt(2)),np.array([1,1,-1,-1])/4];out['allocation']=dict(exact_pairs=int(m.sum()),motor_command_max=float(motor[m].max()),torque_residual_p99=[float(np.quantile(abs(motor[m]@coef-tq[f'xyz[{k}]'][ti[m]]),.99)) for k,coef in enumerate(coefs)])
 results.append(out)
(OUT/'terms.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
