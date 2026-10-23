import json
from pathlib import Path
import numpy as np
from pyulog import ULog
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
rows=[]
for f,name,size in [('082729','23_06_28.ulg',5351975),('082834','23_07_33.ulg',3165340),('082906','23_08_05.ulg',7236033)]:
 p=OUT/name;assert p.stat().st_size==size
 b=json.loads((OUT/f'{f}.data.json').read_text());u=ULog(str(p));d={x.name:x.data for x in u.data_list if x.multi_id==0};lp=d['vehicle_local_position']
 by={round(r['boot']*1e6):r for r in b['local']};matches=[(i,by[int(ts)]) for i,ts in enumerate(lp['timestamp']) if int(ts) in by]
 assert len(matches)>1000
 offset=float(np.median([r['time']-lp['timestamp'][i]/1e6 for i,r in matches]));diff=max(abs(lp['x'][i]-r['x']) for i,r in matches);assert diff==0
 status=d['vehicle_status'];changes=np.r_[True,(np.diff(status['nav_state'])!=0)|(np.diff(status['arming_state'])!=0)]
 r=dict(flight=f,ulog=str(p.relative_to(ROOT)),bytes=size,offset=offset,matches=len(matches),position_difference=float(diff),onboard_end=u.last_timestamp/1e6+offset,status_transitions=[(float(t/1e6+offset),int(a),int(n)) for t,a,n in zip(status['timestamp'][changes],status['arming_state'][changes],status['nav_state'][changes])],events=[dict(time=m.timestamp/1e6+offset,text=m.message) for m in u.logged_messages],dropouts=[dict(time=x.timestamp/1e6+offset,duration_ms=x.duration) for x in u.dropouts])
 rows.append(r)
 print(f,json.dumps(r,indent=2))
 tr=d['trajectory_setpoint'];t=tr['timestamp']/1e6+offset;yaw=b['debug'][0]['yaw0'];lateral=np.array([-np.sin(yaw),np.cos(yaw)]);pos=(np.column_stack([tr['position[0]'],tr['position[1]']])-b['debug'][0]['origin'])@lateral;vel=np.column_stack([tr['velocity[0]'],tr['velocity[1]']])@lateral
 moving=abs(vel)>1e-6
 ix=np.flatnonzero(np.r_[True,moving[1:]!=moving[:-1]])
 print('TRAJECTORY_MOVING_TRANSITIONS',[(round(float(t[i]),3),bool(moving[i]),round(float(pos[i]),4)) for i in ix])
 print('RESET_COUNTS', {k:np.unique(lp[k]).tolist() for k in lp if 'reset_counter' in k})
(OUT/'alignment.json').write_text(json.dumps(rows,indent=2)+'\n')
