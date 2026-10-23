from flight_data import *
p={k:subprocess.Popen([str(R/(k+'_probe'))],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True) for k in ['old','new']}
rows=[]
for f,times in [('051440',[29.722]),('051813',[16.878,23.881])]:
 F=Flight(f)
 for t in times:
  b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);path=F.at('/planner/path',t)
  source=next(c for _,(bp,c) in reversed(F.pairs) if stamp(c)==stamp(path))
  current=F.pair(t)[1]
  pts=[pose(current)+rt@rot(yaw(source)).T@(pose(x)-pose(source)) for x in path['poses']]
  v=rt@np.array([lp['vy'],lp['vx']]);ref=b+rt@np.array([sp['position'][1]-lp['y'],sp['position'][0]-lp['x']]);rv=rt@np.array([sp['velocity'][1],sp['velocity'][0]])
  latch=next(tm for tm,m in F.d['/rosout'] if m['name']=='cpp_flight_adapter' and m['msg'].startswith('latched hold'))
  launch=b+rt@np.array([F.at(LP,latch)['y']-lp['y'],F.at(LP,latch)['x']-lp['x']])
  q=dict(grid=F.at('/rtabmap/grid',t),path=[x.tolist() for x in pts],body=b.tolist(),velocity=v.tolist(),reference=ref.tolist(),reference_velocity=rv.tolist(),launch=launch.tolist())
  results={}
  for mode in ['old','old_with_lead_only_relaxed','new']:
   proc=p['new' if mode=='new' else 'old'];query=dict(q)
   if mode=='old_with_lead_only_relaxed':query['lead_override']=.25
   proc.stdin.write(json.dumps(query)+'\n');proc.stdin.flush();results[mode]=json.loads(proc.stdout.readline())
  row=dict(flight=f,t=t,position_error=float(np.linalg.norm(ref-b)),actual_speed=speed(lp),reference_speed=float(np.linalg.norm(rv)),results=results)
  rows.append(row);print(json.dumps(row,indent=2))
for proc in p.values():proc.stdin.close();assert proc.wait()==0
(R/'replay.json').write_text(json.dumps(rows,indent=2))
