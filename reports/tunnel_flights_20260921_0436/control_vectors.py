from flight_data import *
p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
rows=[]
for f,lo,hi in [('043557',19.4,22.6),('043633',16.3,33.0)]:
 F=Flight(f)
 for t in np.arange(lo,hi,.25):
  b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t)
  pts=[b,b+[.03,0],b-[.03,0],b+[0,.03],b-[0,.03]]
  p.stdin.write(json.dumps({'grid':F.at('/rtabmap/grid',t),'queries':[[*x,*x] for x in pts]})+'\n');p.stdin.flush();cs=json.loads(p.stdout.readline())
  if any(x is None for x in cs):continue
  grad=np.array([cs[1]-cs[2],cs[3]-cs[4]])/.06
  if np.linalg.norm(grad)<.01:continue
  grad/=np.linalg.norm(grad)
  v=rt@np.array([lp['vy'],lp['vx']]);cv=rt@np.array([sp['velocity'][1],sp['velocity'][0]])
  lead=rt@np.array([sp['position'][1]-lp['y'],sp['position'][0]-lp['x']])
  fw=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([fw['x'],fw['y']]);fw/=np.linalg.norm(fw)
  state=F.at('/planner/flight/status',t)['data'].split()[0]
  rows.append(dict(flight=f,t=round(t,3),state=state,clearance=cs[0],actual_inward=float(v@grad),command_inward=float(cv@grad),position_inward=float(lead@grad),actual_forward=float(v@fw),command_forward=float(cv@fw),speed=float(np.linalg.norm(v))))
p.stdin.close();p.wait();(R/'control_vectors.json').write_text(json.dumps(rows,indent=2))
for f in ['043557','043633']:
 for state in ['ACTIVE_RECOVERY','ROUTE']:
  rr=[r for r in rows if r['flight']==f and r['state']==state]
  print(f,state,len(rr),{k:round(float(np.mean([r[k] for r in rr])),4) for k in ['actual_inward','command_inward','position_inward','actual_forward','command_forward']})
 print('last rows')
 for r in [r for r in rows if r['flight']==f][-7:]:print(r)
