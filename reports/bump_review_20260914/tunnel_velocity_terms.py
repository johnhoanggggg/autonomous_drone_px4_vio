import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,'scripts')
from pyulog import ULog
from analyze_control_response import recent_indices
results=[]
for stamp,off,lo,hi in [('081620',-131.5378796755,14.5,16.5),('081709',-179.9391028155,15.6,16.6)]:
 folder=Path(f'ros2_ws/flight_logs/offboard_global_20260916T{stamp}Z');u=ULog(str(next(folder.glob('*.ulg'))));p=u.initial_parameters
 lp=u.get_dataset('vehicle_local_position').data;sp=u.get_dataset('vehicle_local_position_setpoint').data;ts=u.get_dataset('trajectory_setpoint').data
 lt=lp['timestamp']/1e6+off;st=sp['timestamp']/1e6+off
 ii,valid=recent_indices(lp['timestamp'],sp['timestamp'],20000);ri,rv=recent_indices(ts['timestamp'],sp['timestamp'],200000)
 b=json.loads((folder/'render/bag_data.json').read_text());c=b['/rtabmap/odom_correction'];ct=np.array([r['t'] for r in c]);cy=np.array([np.arctan2(2*(r['qw']*r['qz']+r['qx']*r['qy']),1-2*(r['qy']**2+r['qz']**2)) for r in c]);ci=np.maximum(0,np.searchsorted(ct,st,side='right')-1);angle=cy[ci]
 def lateral(n,e):return e*np.cos(angle)-n*np.sin(angle)
 prop=[];ff=[];acc=[];derivatives=[]
 for k,axis in enumerate(['x','y']):
  error=sp['v'+axis]-lp['v'+axis][ii]
  prop.append(p['MPC_XY_VEL_P_ACC']*error);ff.append(np.nan_to_num(ts[f'acceleration[{k}]'][ri],nan=0));acc.append(sp[f'acceleration[{k}]'])
  derivatives.append([(np.interp(st+h,lt,lp['v'+axis])-np.interp(st-h,lt,lp['v'+axis]))/(2*h) for h in [.025,.05,.1]])
 mask=valid&rv&(st>=lo)&(st<hi)
 residual=[lateral(acc[0]-ff[0]-prop[0]+p['MPC_XY_VEL_D_ACC']*derivatives[0][k],acc[1]-ff[1]-prop[1]+p['MPC_XY_VEL_D_ACC']*derivatives[1][k]) for k in range(3)]
 r={'flight':stamp,'window':[lo,hi],'p_mean_map_x':float(lateral(*prop)[mask].mean()),'feedforward_mean_map_x':float(lateral(*ff)[mask].mean()),'acceleration_mean_map_x':float(lateral(*acc)[mask].mean()),'inferred_i_means_map_x':[float(a[mask].mean()) for a in residual]};results.append(r)
print(json.dumps(results,indent=2));Path('reports/bump_review_20260914/tunnel_velocity_terms.json').write_text(json.dumps(results,indent=2)+'\n')
