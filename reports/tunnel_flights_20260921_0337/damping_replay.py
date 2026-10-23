from flight_data import *
old=subprocess.Popen([str(R.parent/'tunnel_flights_20260921_0226/replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
new=subprocess.Popen([str(R/'replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
rows=[]
for f,origin_t,times in [('033547',18.26,[19.4,19.9,20.4,20.6,20.9]),('033547',27.424,[28.4,28.8,29.2,29.5]),('033547',34.08,[34.7,35.2,35.8,36.2,36.5])]:
 F=Flight(f);orig=xy(F.at(LP,origin_t))
 for t in times:
  b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);v=rt@np.array([lp['vy'],lp['vx']]);vs=rt@np.array([sp['velocity'][1],sp['velocity'][0]]);vec=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([vec['x'],vec['y']]);fw/=np.linalg.norm(fw);err=rt@np.array([sp['position'][1]-lp['y'],sp['position'][0]-lp['x']]);origin=b+rt@(orig-xy(lp))[::-1]
  q=dict(grid=F.at('/rtabmap/grid',t),body=b.tolist(),velocity=v.tolist(),forward=fw.tolist(),previous_position=(b+err).tolist(),previous_velocity=vs.tolist(),origin=origin.tolist(),launch=b.tolist(),adaptive=True)
  row=dict(flight=f,t=t,measured=v.tolist(),forward=fw.tolist())
  for name,p in [('old',old),('new',new)]:
   p.stdin.write(json.dumps(q)+'\n');p.stdin.flush();row[name]=json.loads(p.stdout.readline())
  rows.append(row);print(f,t,'actual',np.round(v,3),'old',row['old'].get('desired_velocity'),'new',row['new'].get('desired_velocity'))
for p in [old,new]:p.stdin.close();p.wait()
(R/'damping_replay.json').write_text(json.dumps(rows,indent=2))
