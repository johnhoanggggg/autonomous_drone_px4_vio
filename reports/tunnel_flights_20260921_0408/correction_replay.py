from flight_data import *
p=subprocess.Popen([str(R.parent/'tunnel_flights_20260921_0337/replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
rows=[]
for f,times in [('040617',[14.42,14.67,19.75,20.42,20.67,21.0,22.8,23.5,24.5]),('040800',[12.15,12.4,13.1,13.6,27.,37.15,37.23,38.18])]:
 F=Flight(f)
 for t in times:
  b,rt=F.frame(t);lp=F.at(LP,t);v=rt@np.array([lp['vy'],lp['vx']]);vec=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([vec['x'],vec['y']]);q=dict(grid=F.at('/rtabmap/grid',t),body=b.tolist(),velocity=v.tolist(),forward=fw.tolist(),previous_position=b.tolist(),previous_velocity=[0,0],origin=b.tolist(),launch=b.tolist(),adaptive=True)
  p.stdin.write(json.dumps(q)+'\n');p.stdin.flush();result=json.loads(p.stdout.readline());valid=F.at('/planner/follower/valid',t)['data'];status=F.at('/planner/follower/status',t)['data'];row=dict(flight=f,t=t,speed=speed(lp),follower_valid=valid,follower_status=status,result=result);rows.append(row);print(f,t,round(speed(lp),3),valid,status[:80],result['accepted'])
p.stdin.close();p.wait();(R/'correction_replay.json').write_text(json.dumps(rows,indent=2))
