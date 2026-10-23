from pathlib import Path
import sys,json,math,bisect
ROOT=Path(__file__).resolve().parents[2]; OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_yaw_tracking import read
rows=[]
for flight,lo,hi in [('111341',6.985,15.)]:
 folder=ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{flight}Z';b=json.loads((folder/'render/bag_data.json').read_text());t0=b['t0_receipt_ns'];lp=b['/fmu/out/vehicle_local_position_v1']; lt=[r['t'] for r in lp]
 data,read_origin=read(str(folder),['/rtabmap/grid','/rtabmap/body_pose','/rtabmap/odom_correction']);print(flight, 'raw reader origin', read_origin)
 # Reconstruct complete localization pairs by exact header stamp, in bag receipt order.
 events=sorted([(s,k,m) for k,rs in data.items() for s,m in rs]); poses={};corr={}; pairs=[]; grids=[]
 for s,k,m in events:
  t=(s-t0)/1e9
  if k=='/rtabmap/grid':
   grids.append((t,dict(width=m.info.width,height=m.info.height,resolution=m.info.resolution,origin_x=m.info.origin.position.x,origin_y=m.info.origin.position.y,data=list(m.data))));continue
  stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  (poses if k.endswith('body_pose') else corr)[stamp]=(t,m)
  if stamp in poses and stamp in corr:
   pt,p=poses[stamp];ct,c=corr[stamp];q=c.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));pairs.append((t,pt,ct,p.pose.position.x,p.pose.position.y,yaw))
 gt=[x[0] for x in grids];ptimes=[x[0] for x in pairs]
 for sp in b['/fmu/in/trajectory_setpoint']:
  t=sp['t']
  if not lo<=t<=hi:continue
  l=lp[bisect.bisect_right(lt,t)-1];p=pairs[bisect.bisect_right(ptimes,t)-1];gtime,g=grids[bisect.bisect_right(gt,t)-1]
  speed=math.hypot(l['vx'],l['vy']);factor=.25+speed/.6;n=l['vx']*factor;e=l['vy']*factor;dx=e*math.cos(p[5])-n*math.sin(p[5]);dy=e*math.sin(p[5])+n*math.cos(p[5])
  rows.append(dict(flight=flight,t=t,grid=g,start=[p[3],p[4]],end=[p[3]+dx,p[4]+dy],speed=speed,v=[l['vx'],l['vy']],ned=[l['x'],l['y']],age_local=t-l['t'],age_pose=t-p[1],age_correction=t-p[2],age_map=t-gtime))
(OUT/'audit_input.json').write_text(json.dumps(rows))
