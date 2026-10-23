from flight_data import *
rows=[]
for label in ['before','after']:
 p=subprocess.Popen([str(R/f'selector_{label}')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 for f,ts in [('133740',[22.4,22.9,23.3,23.7]),('133831',[22.9,23.3,23.8,24.3,24.8,25.6,25.8])]:
  F=Flight(f)
  for t in ts:
   body,rotation=F.frame(t); lp=F.at(LP,t)
   velocity=rotation@np.array([lp['vy'],lp['vx']])
   disp=F.at('/planner/follower/vio_displacement',t)['vector']
   forward=rotation@np.array([disp['x'],disp['y']])
   q=dict(grid=F.at('/rtabmap/grid',t),body=body.tolist(),velocity=velocity.tolist(),forward=forward.tolist())
   p.stdin.write(json.dumps(q)+'\n');p.stdin.flush();answer=json.loads(p.stdout.readline())
   drift=next(r for r in json.loads((R/'drift.json').read_text())[f] if abs(r['t']-t)<1e-6)
   answer.update(flight=f,t=t,version=label,inward=round(float(np.array(drift['gradient'])@np.array(answer['velocity'])),4) if answer['accepted'] else None)
   rows.append(answer)
 p.stdin.close();p.wait()
(R/'selector_comparison.json').write_text(json.dumps(rows,indent=2))
for before,after in zip(rows[:len(rows)//2],rows[len(rows)//2:]):
 print(before['flight'],before['t'],before['inward'],'->',after['inward'],'clear',before.get('endpoint_clearance'),'->',after.get('endpoint_clearance'))
