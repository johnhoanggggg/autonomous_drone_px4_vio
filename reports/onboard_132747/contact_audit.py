from pathlib import Path
import json
import numpy as np
from pyulog import ULog
R=Path('/home/john/autonomous_drone_px4_vio');f=R/'ros2_ws/flight_logs/offboard_global_20260913T132747Z'
u=ULog(str(next(f.glob('px4*.ulg'))));d={x.name:x.data for x in u.data_list if x.multi_id==0}
o=json.loads((f/'render/onboard_pairing.json').read_text())['boot_to_bag_s']
a=d['vehicle_acceleration'];s=d['vehicle_status'];t=a['timestamp']/1e6+o
i=np.clip(np.searchsorted(s['timestamp'],a['timestamp'],side='right')-1,0,len(s['timestamp'])-1)
mag=np.linalg.norm([a[f'xyz[{k}]'] for k in range(3)],axis=0)/9.80665
out={'signal':'logged body specific-force magnitude; includes gravity; not a contact detector','windows':{}}
for name,m in [('armed_offboard',(s['nav_state'][i]==14)&(s['arming_state'][i]==2)),('first_excursion',(t>=16)&(t<19)),('second_excursion',(t>=24)&(t<28)),('landing',(s['nav_state'][i]==18)&(s['arming_state'][i]==2))]:
 idx=np.flatnonzero(m);k=idx[np.argmax(mag[idx])]
 out['windows'][name]={'peak_g':float(mag[k]),'bag_time_s':float(t[k]),'samples':len(idx),'max_sample_gap_s':float(np.max(np.diff(t[idx]))) if len(idx)>1 else None}
out['ulog_dropouts']=[{'timestamp':int(x.timestamp),'duration_ms':int(x.duration)}for x in u.dropouts]
p=R/'reports/onboard_132747/contact_audit.json';p.write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
