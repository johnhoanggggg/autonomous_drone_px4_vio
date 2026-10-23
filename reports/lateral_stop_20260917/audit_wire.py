from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_yaw_tracking import read
rows=[]
for r in json.loads((OUT/'summary.json').read_text()):
 f=r['flight'];folder=ROOT/f'ros2_ws/flight_logs/offboard_lateral_stop_20260917T{f}Z';data,_=read(str(next(folder.glob('*.mcap'))),['/fmu/in/trajectory_setpoint','/fmu/out/vehicle_local_position_v1']);t0=r['t0'];lo,hi=r['active_range'];sp=[((t-t0)/1e9,m) for t,m in data['/fmu/in/trajectory_setpoint'] if lo<=(t-t0)/1e9<=hi];lp=[m for t,m in data['/fmu/out/vehicle_local_position_v1'] if lo<=(t-t0)/1e9<=hi];holds=[]
 for l in r['legs']:
  # Exclude the transition sample (transport delivery can differ by a few ms).
  msgs=[m for t,m in sp if l['reference_stop_s']+.1<=t<=l['hold_end_s']-.05]
  holds.append(dict(leg=l['leg'],samples=len(msgs),xy_position_range=np.ptp(np.array([m.position[:2] for m in msgs]),axis=0).tolist(),xy_velocity_max=float(np.max(np.abs([m.velocity[:2] for m in msgs]))),xy_acceleration_max=float(np.max(np.abs([m.acceleration[:2] for m in msgs])))))
 rows.append(dict(flight=f,holds=holds,reset_changes={k:int(np.ptp([getattr(m,k) for m in lp])) for k in ['xy_reset_counter','vxy_reset_counter','heading_reset_counter']},yaw_reference_range=float(np.ptp([m.yaw for _,m in sp])),max_setpoint_receipt_gap_s=float(np.diff([t for t,_ in sp]).max())))
(OUT/'wire.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
