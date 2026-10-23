import json,subprocess,bisect
from pathlib import Path
R=Path(__file__).parent; source=R.parent/'tunnel_flights_20260920_1047'
p=subprocess.Popen([str(R/'selector_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
geometry=json.loads((source/'geometry_candidates.json').read_text());results=[]
for f,rows in geometry.items():
 d=json.loads((source/f'{f}.json').read_text())
 for row in rows:
  if row['measured_speed']>.10:continue
  grid=d['/rtabmap/grid'];g=grid[max(0,bisect.bisect_right([t for t,m in grid],row['t'])-1)][1]
  p.stdin.write(json.dumps({'grid':g,'body':row['body_map'],'velocity':row['measured_map_velocity'],'forward':row['follower_direction']})+'\n');p.stdin.flush()
  out=json.loads(p.stdout.readline());out.update(flight=f,t=row['t'],body_clearance=row['body_clearance'])
  results.append(out)
p.stdin.close();assert p.wait()==0
(R/'recorded_selector_results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
