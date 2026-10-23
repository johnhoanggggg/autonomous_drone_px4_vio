from pathlib import Path
import hashlib,json,numpy as np
from pyulog import ULog
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
rows=[];manifest=[]
for flight,name in [('115304','02_59_25'),('115355','03_00_16'),('115422','03_00_43')]:
 folder=ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{flight}Z'
 for p in list(folder.glob('*.mcap'))+list(folder.glob('*.ulg')):
  manifest.append(dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
 u=ULog(str(folder/f'px4_{name}.ulg'));d={r.name:r.data for r in u.data_list if r.multi_id==0};lp=d['vehicle_local_position'];tr=d['trajectory_setpoint'];st=d['vehicle_status'];ct=d['vehicle_command']
 ix=lambda t,q:np.clip(np.searchsorted(t,q,side='right')-1,0,len(t)-1)
 t=tr['timestamp']/1e6;nav=st['nav_state'][ix(st['timestamp'],tr['timestamp'])];active=nav==14;route=active&np.isfinite(tr['position[0]'])&np.isfinite(tr['velocity[0]'])
 entry=np.flatnonzero(route)[0];p=np.column_stack([tr['position[0]'],tr['position[1]']]);start=t[entry];end=t[-1];land=ct['timestamp'][(ct['command']==21)&(ct['timestamp']/1e6>start)]/1e6
 if len(land):end=land[0]
 elif flight=='115355':end=ct['timestamp'][(ct['command']==400)&(ct['param1']==0)&(ct['timestamp']/1e6>start)][0]/1e6
 keep=(lp['timestamp']/1e6>=start)&(lp['timestamp']/1e6<end)
 r=dict(flight=flight,route_entry_boot=float(start),logged_entry_step=float(np.linalg.norm(p[entry]-p[entry-1])),logged_entry_dt=float(t[entry]-t[entry-1]),peak_route_speed=float(np.hypot(lp['vx'][keep],lp['vy'][keep]).max()),failsafe_before_end=bool(np.any(st['failsafe'][(st['timestamp']/1e6>=start)&(st['timestamp']/1e6<end)])))
 r['reset_counter_changes']={k:int(np.ptp(lp[k][keep])) for k in ['xy_reset_counter','vxy_reset_counter','heading_reset_counter']};r['all_xy_valid']=bool(lp['xy_valid'][keep].all());r['all_vxy_valid']=bool(lp['v_xy_valid'][keep].all())
 r['parameters']={k:u.initial_parameters.get(k) for k in ['MPC_XY_P','MPC_XY_VEL_P_ACC','MPC_XY_VEL_I_ACC','EKF2_EV_DELAY']}
 rows.append(r)
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2));(OUT/'summary_evidence.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
