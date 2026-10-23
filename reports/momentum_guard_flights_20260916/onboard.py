from pathlib import Path
import json,numpy as np
from pyulog import ULog
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
results=[]
def ix(ts,t):return np.clip(np.searchsorted(ts,t,side='right')-1,0,len(ts)-1)
for flight,name in [('113228','02_39_19'),('113258','02_39_49')]:
 u=ULog(str(ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{flight}Z/px4_{name}.ulg'));d={r.name:r.data for r in u.data_list if r.multi_id==0};tr=d['trajectory_setpoint'];lp=d['vehicle_local_position'];sp=d['vehicle_local_position_setpoint'];st=d['vehicle_status'];ct=d['vehicle_command'];t=tr['timestamp']/1e6;origin=u.start_timestamp/1e6
 state=st['nav_state'][ix(st['timestamp'],tr['timestamp'])];brake=np.isnan(tr['position[0]'])&np.isfinite(tr['velocity[0]'])&(state==14)
 starts=np.flatnonzero(brake&~np.r_[False,brake[:-1]]);ends=np.flatnonzero(brake&~np.r_[brake[1:],False])
 print('\nFLIGHT',flight,'origin',origin,'duration',(u.last_timestamp-u.start_timestamp)/1e6)
 print('logs',[(m.timestamp/1e6-origin,m.message) for m in u.logged_messages])
 print('states',[(st['timestamp'][i]/1e6-origin,int(st['nav_state'][i]),int(st['arming_state'][i])) for i in range(len(st['timestamp'])) if i==0 or st['nav_state'][i]!=st['nav_state'][i-1] or st['arming_state'][i]!=st['arming_state'][i-1]])
 print('land commands',[(ct['timestamp'][i]/1e6-origin,int(ct['command'][i])) for i in range(len(ct['timestamp'])) if ct['command'][i] in [21,400]])
 intervals=[]
 for a,b in zip(starts,ends):
  start,end=t[a],t[b];l=(lp['timestamp']/1e6>=start)&(lp['timestamp']/1e6<=end);pos=np.column_stack([lp['x'][l],lp['y'][l]]);vel=np.column_stack([lp['vx'][l],lp['vy'][l]])
  z=np.flatnonzero(np.hypot(tr['velocity[0]'][a:b+1],tr['velocity[1]'][a:b+1])<1e-4)
  event=dict(start_boot=start,start_log=start-origin,end_log=end-origin,duration=end-start,initial_request=[float(tr['velocity[0]'][a]),float(tr['velocity[1]'][a])],zero_after=float(t[a+z[0]]-start) if len(z) else None,displacement=(pos[-1]-pos[0]).tolist(),span=np.ptp(pos,axis=0).tolist(),peak_speed=float(np.linalg.norm(vel,axis=1).max()))
  print('STOP',event)
  rows=[]
  for rel in [-.10,-.05,0,.025,.05,.10,.20,.35,.6,1.]:
   q=start+rel;j=ix(sp['timestamp'],q*1e6);k=ix(lp['timestamp'],sp['timestamp'][j]);h=ix(tr['timestamp'],sp['timestamp'][j]);r=dict(t=float(sp['timestamp'][j]/1e6-start),demand=[float(sp['vx'][j]),float(sp['vy'][j])],actual=[float(lp['vx'][k]),float(lp['vy'][k])],accel=[float(sp['acceleration[0]'][j]),float(sp['acceleration[1]'][j])],position=[float(tr['position[0]'][h]),float(tr['position[1]'][h])]);rows.append(r)
  print('entry demand',rows);event['demand_samples']=rows;intervals.append(event)
 start=intervals[0]['start_boot'];land=ct['timestamp'][np.flatnonzero((ct['command']==21)&(ct['timestamp']/1e6>=start))[0]]/1e6
 keep=(lp['timestamp']/1e6>=start)&(lp['timestamp']/1e6<land);pp=np.column_stack((lp['x'][keep],lp['y'][keep]));vv=np.column_stack((lp['vx'][keep],lp['vy'][keep]))
 until_land=dict(start_boot=start,land_boot=float(land),duration=float(land-start),displacement=(pp[-1]-pp[0]).tolist(),maxtravel=float(np.linalg.norm(pp-pp[0],axis=1).max()),speed_before_land=float(np.linalg.norm(vv[-1])),span=np.ptp(pp,axis=0).tolist())
 results.append(dict(flight=flight,origin=origin,stops=intervals,until_land=until_land))
(OUT/'onboard.json').write_text(json.dumps(results,indent=2))
