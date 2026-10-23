"""Recorded snapshot arithmetic for the node's final ramped transition checks.
Not a closed-loop or callback-order replay. Uses the compiled clearance probe.
"""
import sys,json,subprocess,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'reports/tunnel_flights_20260920_1200'))
from flight_data import Flight,LP,SP,xy,sd
R=Path(__file__).parent
F=Flight('120016');p=subprocess.Popen([str(root/'reports/tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def probe(g,qs):p.stdin.write(json.dumps({'grid':g,'queries':qs})+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
def clamp(v,r):return v*min(1,r/max(np.linalg.norm(v),1e-20))
rows=json.loads((root/'reports/tunnel_flights_20260920_1200/gate_reconstruction.json').read_text());results=[]
for row in rows:
 if row['t']<19 or row['t']<23.:continue
 t=row['t'];origin=np.array(row['origin_ned']);lp=F.at(LP,t);body=xy(lp);m,Rm=F.frame(t);grid=F.at('/rtabmap/grid',t);sp=F.at(SP,row['previous_setpoint_t']);prev=np.array(sp['position'][:2]);pv=np.array(sp['velocity'][:2]);desired=(Rm.T@np.array(row['candidate']['velocity']))[::-1]
 to_map=lambda q:m+Rm@(q-body)[::-1]
 actual=body+np.array([lp['vx'],lp['vy']])*(.25+math.hypot(lp['vx'],lp['vy'])/.2)
 trials=[]
 for k in range(8,-1,-1):
  v=pv+clamp(desired*k/8-pv,.15*.05);target=body+clamp(prev+v*.05-body,.05-.06*.05-1e-6);speed=np.linalg.norm(v);delta=v/max(speed,1e-9)*sd(speed);cmdend=body+delta;combined=actual+target-body+delta
  values=probe(grid,[[*to_map(a),*to_map(b)] for a,b in [(body,actual),(body,target),(body,cmdend),(actual,combined),(prev,target)]])
  bounds=[.28,min(row['body_clearance'],.38),min(row['body_clearance'],.38),.28,.28];radii={n:float(np.linalg.norm(q-origin)) for n,q in [('target',target),('measured_stop',actual),('command_stop',cmdend),('combined_stop',combined)]}
  okay=all(c is not None and c+1e-6>=minimum for c,minimum in zip(values,bounds)) and max(radii.values())<=.25
  trials.append({'fraction':k/8,'ramped_speed':float(speed),'velocity_change':float(np.linalg.norm(v-pv)),'position_lead':float(np.linalg.norm(target-body)),'radii':radii,'clearances':values,'accepted':okay})
  if okay:break
 results.append({'flight':'120016','t':t,'old_reconstructed_failure':row['inferred_failed_checks'],'trials':trials})
p.stdin.close();assert p.wait()==0
(R/'recorded_budget_results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
