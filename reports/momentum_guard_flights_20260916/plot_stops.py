from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
rows=json.loads((OUT/'onboard.json').read_text());fig,aa=plt.subplots(2,2,figsize=(12,7),sharex=True)
for col,(r,name) in enumerate(zip(rows,['02_39_19','02_39_49'])):
 u=ULog(str(ROOT/f"ros2_ws/flight_logs/offboard_global_20260916T{r['flight']}Z/px4_{name}.ulg"));d={s.name:s.data for s in u.data_list if s.multi_id==0};start=r['until_land']['start_boot'];land=r['until_land']['duration']
 for row,(field,label) in enumerate([('vx','North'),('vy','East')]):
  a=aa[row,col]
  for topic,color,title in [('vehicle_local_position','tab:red','actual velocity'),('vehicle_local_position_setpoint','tab:blue','PX4 internal velocity demand')]:
   q=d[topic];t=q['timestamp']/1e6-start;keep=(t>=-.4)&(t<land);a.plot(t[keep],q[field][keep],color=color,label=title)
  a.axhline(0,color='gray',lw=.6);a.axvline(0,color='black',ls='--',label='stop entry');a.axvline(land,color='purple',ls=':',label='external LAND request');a.set_ylabel(label+' velocity / m/s');a.grid(alpha=.2);a.set_xlim(-.4,1.4)
  if row==0:a.set_title(r['flight']);a.legend(fontsize=8)
  else:a.set_xlabel('Seconds relative to velocity-only stop entry')
fig.tight_layout();fig.savefig(OUT/'onboard_stops.png',dpi=150)
