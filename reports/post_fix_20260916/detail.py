from pathlib import Path
import json,numpy as np
from pyulog import ULog
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
out=[]
for stamp,windows in [('081620',[(14.5,16.5),(16.5,17.5),(27.5,29.0)]),('081709',[(15.6,16.6),(16.6,17.38),(19.,20.)])]:
 folder=ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{stamp}Z';b=json.loads((folder/'render/bag_data.json').read_text());u=ULog(str(next(folder.glob('px4*.ulg'))));d={r.name:r.data for r in u.data_list if r.multi_id==0};z=np.load(OUT/f'{stamp}_series.npz');t=z['t'];summary={'flight':stamp,'windows':[]}
 for a,c in windows:
  m=(t>=a)&(t<c);r={'window':[a,c],**{k:float(z[k][m].mean()) for k in ['v','vd','ad','tilt','imu','error']}}
  for topic in ['/rtabmap/body_pose','/rtabmap/body_vio_pose']:
   rr=b[topic];ts=np.array([r['t'] for r in rr]);i=np.searchsorted(ts,a);j=np.searchsorted(ts,c);r[topic+'_dx']=rr[j]['x']-rr[i]['x']
  summary['windows'].append(r)
 off=json.loads((folder/'render/onboard_pairing.json').read_text())['boot_to_bag_s'];lp=d['vehicle_local_position'];sp=d['vehicle_local_position_setpoint'];ix=np.clip(np.searchsorted(sp['timestamp'],lp['timestamp'],side='right')-1,0,len(sp['timestamp'])-1);cc=b['/rtabmap/odom_correction'];ct=np.array([r['t'] for r in cc]);cy=np.array([np.arctan2(2*(r['qw']*r['qz']+r['qx']*r['qy']),1-2*(r['qy']**2+r['qz']**2)) for r in cc]);a=cy[np.clip(np.searchsorted(ct,t,side='right')-1,0,len(ct)-1)];err=(lp['y']-sp['y'][ix])*np.cos(a)-(lp['x']-sp['x'][ix])*np.sin(a);valid=z['valid'];ii=np.flatnonzero(valid)[np.argmax(abs(err[valid]))];summary['peak_lateral_error']={'t':float(t[ii]),'error':float(err[ii])}
 # Recorded hold boundaries and the internal controller's demand immediately before/after.
 summary['holds']=[]
 for tt in ([20.574,29.273] if stamp=='081620' else [17.388,21.738]):
  rows=[]
  for q in [tt-.10,tt+.15]:
   j=np.argmin(abs(t-q));rows.append({k:float(z[k][j]) for k in ['t','v','vd','ad','error']})
  summary['holds'].append(rows)
 vs=d['vehicle_status'];summary['end']={'ulog_end_bag_s':float(vs['timestamp'][-1]/1e6+off),'arming_transitions':[(float(vs['timestamp'][i]/1e6+off),int(vs['arming_state'][i])) for i in np.r_[0,np.flatnonzero(np.diff(vs['arming_state']))+1]],'nav_transitions':[(float(vs['timestamp'][i]/1e6+off),int(vs['nav_state'][i])) for i in np.r_[0,np.flatnonzero(np.diff(vs['nav_state']))+1]]}
 out.append(summary)
(OUT/'detail.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
