from pathlib import Path
import sys,json
import numpy as np
sys.path.insert(0,'scripts')
from analyze_yaw_tracking import read
results=[]
for bag in sorted(Path('ros2_ws/flight_logs').glob('*20260914*')):
 if not bag.is_dir():continue
 topics=['/rosout','/fmu/out/vehicle_local_position_v1','/fmu/in/trajectory_setpoint','/fmu/out/sensor_combined','/waypoint/status','/fmu/in/vehicle_command']
 try:d,t0=read(str(bag),topics)
 except Exception as e:print(bag.name,str(e));continue
 lp=d[topics[1]];imu=d[topics[3]]
 r={'bag':str(bag),'t0':t0,'decode_errors':d['_decode_errors'],'messages':[]}
 if lp:
  speeds=np.array([np.hypot(m.vx,m.vy) for _,m in lp]);j=int(np.argmax(speeds))
  r.update(duration_s=(lp[-1][0]-t0)/1e9,max_speed=float(speeds[j]),max_speed_t=(lp[j][0]-t0)/1e9,max_altitude=float(max(-m.z for _,m in lp)))
 if imu:
  accel=np.array([np.linalg.norm(m.accelerometer_m_s2[:2]) for _,m in imu]);j=int(np.argmax(accel))
  r.update(max_body_xy_accel=float(accel[j]),accel_peak_t=(imu[j][0]-t0)/1e9)
 for st,m in d['/rosout']:
  if any(k in m.msg.lower() for k in ['arm','hold','leg','turn','land','abort','mode','reference','height','square','climb']):
   r['messages'].append([(st-t0)/1e9,m.name,m.msg])
 results.append(r)
 print(json.dumps(r),flush=True)
Path('reports/bump_review_20260914/scan.json').write_text(json.dumps(results,indent=2)+'\n')
