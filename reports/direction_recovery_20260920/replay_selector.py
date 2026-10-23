"""Recorded geometry snapshots, not a callback or closed-loop flight replay."""
import sys,json,subprocess
from pathlib import Path
import numpy as np
root=Path.cwd();sys.path.insert(0,str(root/'reports/tunnel_flights_20260920_1253'))
from flight_data import Flight,LP
out=[]
for binary in ['reports/active_recovery_20260920/selector_probe','reports/direction_recovery_20260920/selector_probe']:
 p=subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 for f,t in [('125348',17.075),('125509',30.87),('125509',31.52),('125509',31.97),('125509',32.17)]:
  F=Flight(f);body,r=F.frame(t);lp=F.at(LP,t);di=F.at('/planner/follower/vio_displacement',t)['vector']
  req={'grid':F.at('/rtabmap/grid',t),'body':body.tolist(),'velocity':(r@np.array([lp['vy'],lp['vx']])).tolist(),'forward':(r@np.array([di['x'],di['y']])).tolist()}
  p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();result=json.loads(p.stdout.readline());out.append({'binary':binary,'flight':f,'t':t,**result})
 p.stdin.close();assert p.wait()==0
Path('reports/direction_recovery_20260920/recorded_selector_results.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
