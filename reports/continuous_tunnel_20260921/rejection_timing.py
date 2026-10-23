import sys,json,subprocess
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('reports/tunnel_flights_20260921_0111').resolve()))
from flight_data import Flight,LP,SP,xy
R=Path(__file__).parent;F=Flight('011120');rows=[]
p=subprocess.Popen([str(R/'replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
for t in [18.9,19.97]:
 b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);d=F.at('/planner/follower/vio_displacement',t)['vector'];orig=F.at(LP,18.81);launch=F.at(LP,3.1)
 q=dict(grid=F.at('/rtabmap/grid',t),body=b.tolist(),velocity=(rt@np.array([lp['vy'],lp['vx']])).tolist(),forward=(rt@np.array([d['x'],d['y']])).tolist(),previous_position=(b+rt@(np.array(sp['position'][:2])-xy(lp))[::-1]).tolist(),previous_velocity=(rt@np.array(sp['velocity'][:2][::-1])).tolist(),origin=(b+rt@(xy(orig)-xy(lp))[::-1]).tolist(),launch=(b+rt@(xy(launch)-xy(lp))[::-1]).tolist())
 for adaptive in [False,True]:
  p.stdin.write(json.dumps({**q,'adaptive':adaptive,'reject_all':True})+'\n');p.stdin.flush();out=json.loads(p.stdout.readline());out.update(t=t,adaptive=adaptive);rows.append(out);print(t,'adaptive',adaptive,'accepted',out['accepted'],'ramped',out['ramped'],'mean ms',out['mean_ms'])
p.stdin.close();p.wait();(R/'rejection_timing.json').write_text(json.dumps(rows,indent=2))
