from flight_data import *
proc=subprocess.Popen([str(R.parent/'tunnel_flights_20260921_0518'/'new_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
rows=[]
for f,times in [('054552',[33.430,36.130,36.430,53.430])]:
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
  if not np.all(np.isfinite(ref)):
   q['reference']=b.tolist();q['reference_velocity']=[0.,0.]
  proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();result=json.loads(proc.stdout.readline())
  row=dict(flight=f,t=t,position_error=float(np.linalg.norm(ref-b)) if np.all(np.isfinite(ref)) else None,actual_speed=speed(lp),reference_speed=float(np.linalg.norm(rv)),result=result, follower_valid=F.at("/planner/follower/valid",t)["data"], follower_status=F.at("/planner/follower/status",t)["data"], stop_reseed=not bool(np.all(np.isfinite(ref))))
  rows.append(row);print(json.dumps(row,indent=2))
proc.stdin.close();assert proc.wait()==0
(R/'replay.json').write_text(json.dumps(rows,indent=2))
