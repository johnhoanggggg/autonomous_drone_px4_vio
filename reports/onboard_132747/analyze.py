from pathlib import Path
import json,numpy as np
from pyulog import ULog
ROOT=Path('/home/john/autonomous_drone_px4_vio')
def pair(t,at):return np.clip(np.searchsorted(t,at,side='right')-1,0,len(t)-1)
out=[]
for stamp in ['132747']:
 p=next((ROOT/f'ros2_ws/flight_logs/offboard_global_20260913T{stamp}Z').glob('px4*.ulg'));u=ULog(str(p));d={x.name:x.data for x in u.data_list if x.multi_id==0}
 lp=d['vehicle_local_position'];sp=d['vehicle_local_position_setpoint'];vs=d['vehicle_status'];t=lp['timestamp'];j=pair(vs['timestamp'],t);mask=(vs['arming_state'][j]==2)&(vs['nav_state'][j]==14)&(t>u.start_timestamp+3e6);i=pair(sp['timestamp'],t);mask&=(t-sp['timestamp'][i]<2e5)
 v=np.hypot(lp['vx'],lp['vy']);err=np.hypot(lp['x']-sp['x'][i],lp['y']-sp['y'][i]);lat=-np.sin(lp['heading'])*lp['vx']+np.cos(lp['heading'])*lp['vy']
 r={'flight':stamp,'duration':(u.last_timestamp-u.start_timestamp)/1e6,'max_speed':float(max(v[mask])),'max_body_lateral_speed':float(max(abs(lat[mask]))),'position_error_p50_p90_max':np.quantile(err[mask],[.5,.9,1]).tolist(),'xy_reset':np.unique(lp['xy_reset_counter'][mask]).tolist(),'EV_DELAY':float(u.initial_parameters['EKF2_EV_DELAY'])}
 for name in ['estimator_aid_src_ev_pos','estimator_aid_src_ev_yaw']:
  if name not in d:continue
  a=d[name];k=pair(t,a['timestamp']);m=mask[k];r[name]={'samples':int(m.sum()),'rejected':int(a['innovation_rejected'][m].sum()),'fused':int(a['fused'][m].sum()),'innovations':{key:np.quantile(abs(a[key][m]),[.5,.9,1]).tolist() for key in a if key.startswith('innovation[') or key=='innovation'},'test_ratios':{key:float(np.max(a[key][m])) for key in a if key.startswith('test_ratio[') or key=='test_ratio'}}
 out.append(r)
print(json.dumps(out,indent=2));(ROOT/'reports/onboard_132747/metrics.json').write_text(json.dumps(out,indent=2))
