from pathlib import Path
import json,sys,math,hashlib,numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_lateral_stop import summarize
results=[]
for f in ['080456','080556']:
 d=json.loads((OUT/f'{f}.data.json').read_text());active=[r for r in d['debug'] if r['state'] in ['LATERAL_MOVE','LATERAL_HOLD']]
 clean=dict(d,debug=active);r=summarize(clean);r['flight']=f;r['events']=d['events']
 r['active_range']=[active[0]['time'],active[-1]['time']];r['minimum_features']=min(x['count'] for x in d['features'] if active[0]['time']<=x['time']<=active[-1]['time'])
 r['maximum_debug_gap_s']=float(np.diff([x['time'] for x in active]).max())
 r['invalid_local_samples']=sum(not x['valid'] for x in d['local'] if active[0]['time']<=x['time']<=active[-1]['time'])
 for leg in r['legs']:
  hold=[x for x in d['local'] if leg['reference_stop_s']<=x['time']<=leg['hold_end_s']]
  target=leg['target_m']; cfg=r['config'];points=np.array([[x['right']-target,x['forward']] for x in hold]);posok=np.linalg.norm(points,axis=1)<=cfg['settle_position'];speedok=np.array([math.hypot(x['vr'],x['vf'])<=cfg['settle_speed'] for x in hold]);yawok=np.array([abs(x['yaw_error'])<=5 for x in hold]);zok=np.array([abs(-x['z']-.3)<=.07 for x in hold]);allok=posok&speedok&yawok&zok
  begin=None;longest=0
  for x,ok in zip(hold,allok):
   if ok:
    if begin is None:begin=x['time']
    longest=max(longest,x['time']-begin)
   else:begin=None
  leg.update(hold_lateral_min=float(points[:,0].min()),hold_lateral_max=float(points[:,0].max()),hold_lateral_span=float(np.ptp(points[:,0])),longest_settled_run_s=longest,gate_failure_fraction={k:float(1-np.mean(v)) for k,v in [('position',posok),('speed',speedok),('yaw',yawok),('altitude',zok)]})
  # Continuous VIO in the initial flight right/forward axes; ENU to NED.
  vv=[x for x in d['vio'] if leg['reference_stop_s']<=x['time']<=leg['hold_end_s']];c,s=math.cos(d['debug'][0]['yaw0']),math.sin(d['debug'][0]['yaw0']);raw=np.array([-s*x['y']+c*x['x'] for x in vv]);leg['raw_vio_hold_span_m']=float(np.ptp(raw)) if len(raw) else None
 (OUT/f'{f}.json').write_text(json.dumps(r,indent=2))
 results.append(r)
(OUT/'summary.json').write_text(json.dumps(results,indent=2))
for r in results:
 print('\nFLIGHT',r['flight'],'features',r['minimum_features'],'gaps',r['maximum_debug_gap_s'])
 for l in r['legs']:print(json.dumps(l))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(4,2,figsize=(14,11))
for col,r in enumerate(results):
 d=json.loads((OUT/f"{r['flight']}.data.json").read_text());start,end=r['active_range'];lp=[x for x in d['local'] if start-.4<=x['time']<=end];debug=[x for x in d['debug'] if start-.4<=x['time']<=end];t=np.array([x['time'] for x in lp]);td=[x['time'] for x in debug]
 axs[0,col].plot(t,[x['right'] for x in lp],label='PX4 lateral position');axs[0,col].plot(td,[x['offset'] for x in debug],label='Reference');axs[0,col].set_title(r['flight'])
 raw=[x for x in d['vio'] if start-.4<=x['time']<=end];c,s=math.cos(d['debug'][0]['yaw0']),math.sin(d['debug'][0]['yaw0']);rt=np.array([x['time'] for x in raw]);rp=np.array([-s*x['y']+c*x['x'] for x in raw]);rp+=np.interp(rt[0],t,[x['right'] for x in lp])-rp[0];axs[0,col].plot(rt,rp,alpha=.6,label='Raw body VIO, constant offset aligned')
 axs[1,col].plot(t,[x['vr'] for x in lp],label='PX4 lateral velocity');axs[1,col].plot(td,[x['velocity'] for x in debug],label='Velocity feedforward')
 axs[2,col].plot(t,[x['forward'] for x in lp],label='Forward position error');axs[2,col].axhline(.1,ls=':',c='gray');axs[2,col].axhline(-.1,ls=':',c='gray')
 axs[3,col].plot(t,[x['yaw_error'] for x in lp],label='Yaw error');axs[3,col].axhline(5,ls=':',c='gray');axs[3,col].axhline(-5,ls=':',c='gray')
 for row,label in enumerate(['Lateral position / m','Lateral velocity / m/s','Forward offset / m','Heading error / deg']):
  a=axs[row,col];a.set_ylabel(label);a.grid(alpha=.2);a.legend(fontsize=8);a.set_xlim(start-.4,end+.2)
  for l in r['legs']:a.axvspan(l['brake_s'],l['reference_stop_s'],color='orange',alpha=.15)
 axs[3,col].set_xlabel('Seconds from bag origin; orange = planned braking')
fig.tight_layout();fig.savefig(OUT/'motion.png',dpi=140)
