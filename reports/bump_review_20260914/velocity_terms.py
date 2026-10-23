"""Approximate horizontal velocity-loop integral residual, not logged I."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,'scripts')
from pyulog import ULog
from analyze_control_response import recent_indices
out=[]
for name,pulse in [('19_57_15',9.440121),('19_59_21',15.935202)]:
 u=ULog(f'ros2_ws/flight_logs/bump_candidates_20260914/px4_{name}.ulg');p=u.initial_parameters
 lp=u.get_dataset('vehicle_local_position').data;sp=u.get_dataset('vehicle_local_position_setpoint').data;ts=u.get_dataset('trajectory_setpoint').data
 lt=lp['timestamp'].astype(float)/1e6-u.start_timestamp/1e6;st=sp['timestamp'].astype(float)/1e6-u.start_timestamp/1e6
 ii,valid=recent_indices(lp['timestamp'],sp['timestamp'],20000);ri,rv=recent_indices(ts['timestamp'],sp['timestamp'],200000)
 assert p['MPC_VEL_LP']==0 and p['MPC_VEL_NF_FRQ']==0
 error=sp['vx']-lp['vx'][ii];feedforward=np.nan_to_num(ts['acceleration[0]'][ri],nan=0)
 # Finite-difference with multiple windows: independent estimates of derivative
 # contribution. PX4 internally uses its own 5 Hz filtered derivative.
 estimates=[]
 for half in [.025,.05,.10]:
  derivative=(np.interp(st+half,lt,lp['vx'])-np.interp(st-half,lt,lp['vx']))/(2*half)
  residual=sp['acceleration[0]']-feedforward-p['MPC_XY_VEL_P_ACC']*error+p['MPC_XY_VEL_D_ACC']*derivative
  estimates.append(residual)
 result={'ulog':name,'pulse':pulse,'windows':{}}
 for label,a,b in [('pre_bump',-.8,-.2),('recovery',.3,1.3),('approach_reference',1.3,2.3),('after_crossing',2.3,3.3)]:
  mask=valid&rv&(st>=pulse+a)&(st<pulse+b)
  result['windows'][label]={'p_mean':float(np.mean(p['MPC_XY_VEL_P_ACC']*error[mask])),'acceleration_demand_mean':float(np.mean(sp['acceleration[0]'][mask])),'inferred_i_means_derivative_windows': [float(x[mask].mean()) for x in estimates]}
 # Compare inferred integral change across a later unsaturated interval with
 # the configured integral equation; smoothed endpoint bands reduce log noise.
 a=pulse+1.0;b=pulse+3.2
 start=valid&rv&(st>=a-.1)&(st<a+.1);end=valid&rv&(st>=b-.1)&(st<b+.1)
 grid=np.arange(a,b,.01);prediction=p['MPC_XY_VEL_I_ACC']*np.trapezoid(np.interp(grid,st,error),grid)
 result['integral_consistency']={'interval':[a,b],'predicted_change_from_error':float(prediction),'inferred_changes':[float(x[end].mean()-x[start].mean()) for x in estimates]}
 out.append(result)
Path('reports/bump_review_20260914/velocity_terms.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
