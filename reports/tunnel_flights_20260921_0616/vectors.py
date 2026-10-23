from flight_data import *
proc=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def probe(g,qs):
 proc.stdin.write(json.dumps({'grid':g,'queries':qs})+'\n');proc.stdin.flush();return json.loads(proc.stdout.readline())
for f,times in [('061515',[23.,24.4,25.,25.8,26.2,26.8]),('061647',[24.,28.,31.])]:
 F=Flight(f);print(f)
 for t in times:
  b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);g=F.at('/rtabmap/grid',t)
  v=rt@np.array([lp['vy'],lp['vx']]);r=b+rt@np.array([sp['position'][1]-lp['y'],sp['position'][0]-lp['x']]);rv=rt@np.array([sp['velocity'][1],sp['velocity'][0]])
  cs=probe(g,[[*(b+x),*(b+x)] for x in [np.array([0.,0.]),np.array([.03,0]),np.array([-.03,0]),np.array([0,.03]),np.array([0,-.03])]])
  inward=np.array([cs[1]-cs[2],cs[3]-cs[4]]);inward/=max(1e-9,np.linalg.norm(inward))
  prev=F.at(LP,t-.3);dp=rt@np.array([lp['y']-prev['y'],lp['x']-prev['x']])/.3
  print(t,'b',np.round(b,3),'err',np.round(r-b,3),'v',np.round(v,3),'rv',np.round(rv,3),'inward',np.round(inward,3),'c',round(cs[0],3),'v_in',round(float(v@inward),3),'dpos_in',round(float(dp@inward),3))
proc.stdin.close();proc.wait()
