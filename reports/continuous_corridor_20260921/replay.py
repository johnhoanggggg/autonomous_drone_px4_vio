import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'tunnel_flights_20260921_0436'))
from flight_data import *
O=Path(__file__).resolve().parent
p=subprocess.Popen([str(O/'replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
rows=[]
for f,times in [('043557',[12.,14.,15.,17.,19.,20.,21.15,21.65,21.9,22.4]),('043633',[11.,12.,13.,15.5,17.,20.,23.,25.,27.,29.8,31.3,31.55,32.05,32.55])]:
 F=Flight(f)
 for t in times:
  b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t)
  path=F.at('/planner/path',t)
  # Reexpress the path with its own paired correction, not the latest transform.
  source=next((c for _,(bp,c) in reversed(F.pairs) if stamp(c)==stamp(path)),None)
  if source is None:continue
  current=F.pair(t)[1]
  pts=[pose(current)+rt@rot(yaw(source)).T@(pose(x)-pose(source)) for x in path['poses']]
  v=rt@np.array([lp['vy'],lp['vx']])
  ref=b+rt@np.array([sp['position'][1]-lp['y'],sp['position'][0]-lp['x']])
  rv=rt@np.array([sp['velocity'][1],sp['velocity'][0]])
  launch=b+rt@np.array([F.at(LP,F.phases()[0])['y']-lp['y'],F.at(LP,F.phases()[0])['x']-lp['x']])
  for seed in ['recorded_reference','body_zero_reference']:
   q=dict(grid=F.at('/rtabmap/grid',t),path=[x.tolist() for x in pts],body=b.tolist(),velocity=v.tolist(),reference=(ref if seed=='recorded_reference' else b).tolist(),reference_velocity=(rv if seed=='recorded_reference' else np.zeros(2)).tolist(),launch=launch.tolist())
   if not all(math.isfinite(x) for x in q['reference']):continue
   p.stdin.write(json.dumps(q)+'\n');p.stdin.flush();out=json.loads(p.stdout.readline())
   rows.append(dict(flight=f,t=t,seed=seed,actual_speed=speed(lp),recorded_reference_speed=float(np.linalg.norm(rv)),result=out))
p.stdin.close();p.wait();(O/'replay.json').write_text(json.dumps(rows,indent=2))
for seed in ['recorded_reference','body_zero_reference']:
 rr=[x for x in rows if x['seed']==seed]
 print(seed,len(rr),'accepted',sum(x['result']['accepted'] for x in rr),'p95_ms',np.percentile([x['result']['milliseconds'] for x in rr],95),'max_ms',max(x['result']['milliseconds'] for x in rr))
 for x in rr:
  if not x['result']['accepted']:print(' rejected',x['flight'],x['t'],x['result']['reason'])
