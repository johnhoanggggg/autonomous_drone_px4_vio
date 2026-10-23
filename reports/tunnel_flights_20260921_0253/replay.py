"""Independent recorded snapshots, not a simulated flight or route-handover proof."""
from flight_data import *
p=subprocess.Popen([str(R.parent/'tunnel_flights_20260921_0226/replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
rows=[]
for f,t,origin_t in [('025229',28.35,26.496),('025229',28.75,26.496),('025229',29.00,26.496),('025312',9.41,9.162),('025312',9.55,9.162),('025312',9.85,9.162)]:
 F=Flight(f); b,rt=F.frame(t);lp=F.at(LP,t);originlp=F.at(LP,origin_t)
 v=rt@np.array([lp['vy'],lp['vx']]);g=F.at('/rtabmap/grid',t)
 if f=='025229':
  vec=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([vec['x'],vec['y']]);previous=b;pv=np.zeros(2)
 else:
  # Re-express the latest path under its exact source correction, then project
  # to its nearest segment and look 20 cm farther along the polyline.
  path=F.at('/planner/path',t);pc=next(m for _,m in F.d['/rtabmap/odom_correction'] if stamp(m)==stamp(path)); cc=F.pair(t)[1]
  rr=rot(yaw(cc))@rot(-yaw(pc)); pts=np.array([pose(m) for m in path['poses']]);pts=(rr@(pts-pose(pc)).T).T+pose(cc)
  lengths=np.linalg.norm(np.diff(pts,axis=0),axis=1);cumulative=np.r_[0,np.cumsum(lengths)];projections=[]
  for i,(a,z) in enumerate(zip(pts,pts[1:])):
   d=z-a;u=np.clip((b-a)@d/(d@d),0,1) if d@d>0 else 0;projections.append((np.linalg.norm(a+u*d-b),cumulative[i]+u*lengths[i]))
  along=min(projections)[1]+.2;i=min(np.searchsorted(cumulative,along,side='right')-1,len(lengths)-1);target=pts[i]+(pts[i+1]-pts[i])*np.clip((along-cumulative[i])/max(lengths[i],1e-9),0,1);fw=target-b
  sp=F.at(SP,9.39);previous=b+rt@np.array([sp['position'][1]-lp['y'],sp['position'][0]-lp['x']]);pv=rt@np.array([sp['velocity'][1],sp['velocity'][0]])
 origin=b+rt@np.array([originlp['y']-lp['y'],originlp['x']-lp['x']])
 q=dict(grid=g,body=b.tolist(),velocity=v.tolist(),forward=fw.tolist(),previous_position=previous.tolist(),previous_velocity=pv.tolist(),origin=origin.tolist(),launch=b.tolist(),adaptive=True)
 p.stdin.write(json.dumps(q)+'\n');p.stdin.flush();result=json.loads(p.stdout.readline());row=dict(flight=f,t=t,measured_speed=float(np.linalg.norm(v)),result=result);rows.append(row);print(f,t,result)
p.stdin.close();assert p.wait()==0
(R/'replay.json').write_text(json.dumps(rows,indent=2))
