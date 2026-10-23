from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'scripts'))
from analyze_yaw_tracking import read
from render_flight_map import Grid
out=[]
for stamp,window in [('071623',(17,32)),('071845',(13,30))]:
 f=ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{stamp}Z';b=json.loads((f/'render/bag_data.json').read_text());t0=b['t0_receipt_ns'];topics=['/rtabmap/grid','/rtabmap/body_pose','/rtabmap/body_vio_pose','/rtabmap/odom_correction','/planner/follower/vio_displacement','/fmu/out/vehicle_status_v1','/rtabmap/vio_feature_count'];data,_=read(str(f),topics)
 grids=[((s-t0)/1e9,Grid(m)) for s,m in data['/rtabmap/grid']];gt=np.array([t for t,g in grids]);rows=[]
 for r in b['/rtabmap/body_pose']:
  if not window[0]<=r['t']<=window[1]:continue
  gi=np.searchsorted(gt,r['t'],side='right')-1
  if gi<0:continue
  clearance=grids[gi][1].point_clearance(r['x'],r['y'])
  if clearance is not None:rows.append(dict(t=r['t'],x=r['x'],y=r['y'],clearance=clearance,grid_time=grids[gi][0]))
 result=dict(flight=stamp,minimum_clearance=min(rows,key=lambda r:r['clearance']),samples_below_28cm=sum(r['clearance']<.28 for r in rows),samples=len(rows),clearance_samples=rows)
 result['gaps']={}
 for topic in topics[1:5]:
  rr=data[topic];ss=np.array([(s-t0)/1e9 for s,m in rr]);dt=np.diff(ss);valid=np.flatnonzero((ss[1:]>=window[0])&(ss[1:]<=window[1]));ii=valid[np.argmax(dt[valid])];result['gaps'][topic]=dict(start=float(ss[ii]),end=float(ss[ii+1]),gap=float(dt[ii]))
 result['message_order']=[]
 for topic in topics[1:5]:
  for s,m in data[topic]:
   t=(s-t0)/1e9
   if (stamp=='071623' and 21.1<t<21.25) or (stamp=='071845' and 18.1<t<18.25):
    result['message_order'].append(dict(topic=topic,t=t,header_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9))
 result['message_order'].sort(key=lambda r:r['t']);out.append(result)
 print(json.dumps({k:v for k,v in result.items() if k!='clearance_samples'},indent=2))
(ROOT/'reports/lateral_excursions_20260916/bag_audit.json').write_text(json.dumps(out,indent=2))
