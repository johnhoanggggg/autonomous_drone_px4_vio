import json, math, bisect
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
R=Path(__file__).parent
metrics=json.loads((R/'metrics.json').read_text());geo=json.loads((R/'geometry_candidates.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(13,9),layout='constrained')
for ax,f in zip(axes,['104759','104844']):
 d=json.loads((R/f'{f}.json').read_text());mt=metrics[f];end=mt['land'];start=mt['route_entry']
 def at(topic,t):
  seq=d[topic];return seq[max(0,bisect.bisect_right([a for a,b in seq],t)-1)][1]
 def xy(m):return np.array([m['pose']['position'][k] for k in ['x','y']])
 g=at('/rtabmap/grid',end);info=g['info'];res=info['resolution'];ox=info['origin']['position']['x'];oy=info['origin']['position']['y']
 grid=np.array(g['data']).reshape(info['height'],info['width']);colors=np.where(grid<0,0,np.where(grid>=65,2,1))
 ax.imshow(colors,origin='lower',extent=[ox,ox+res*info['width'],oy,oy+res*info['height']],cmap=ListedColormap(['#d8dce3','#fafafa','#263340']),vmin=0,vmax=2,interpolation='nearest')
 path=np.array([xy(p) for p in at('/planner/path',end)['poses']]);ax.plot(*path.T,'--',color='#198349',lw=1.8,label='Last planner path')
 seen=set()
 for a,b,kind in mt['intervals']:
  track=np.array([xy(m) for t,m in d['/rtabmap/body_pose'] if a<=t<=b]);label='ROUTE' if kind=='route' else 'Zero-velocity braking'
  if len(track):ax.plot(*track.T,c='#246ce0' if kind=='route' else '#e07a12',lw=2.5,label=None if label in seen else label);seen.add(label)
  if kind!='route':ax.scatter(*xy(at('/rtabmap/body_pose',a)),c='#a35300',s=40,marker='s',zorder=6)
 targets=[]
 for t,m in d['/fmu/in/trajectory_setpoint']:
  if not start<=t<=end or not math.isfinite(m['position'][0]):continue
  lp=at('/fmu/out/vehicle_local_position_v1',t);body=at('/rtabmap/body_pose',t);q=at('/rtabmap/odom_correction',t)['pose']['orientation'];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2));co,si=math.cos(yaw),math.sin(yaw)
  targets.append(xy(body)+np.array([[co,-si],[si,co]])@np.array([m['position'][1]-lp['y'],m['position'][0]-lp['x']]))
 ax.scatter(*np.array(targets).T,c='#b337a7',s=4,label='Finite XY targets',alpha=.6)
 body=xy(at('/rtabmap/body_pose',end));ax.add_patch(Circle(body,.28,fill=False,color='#d02020',lw=1.5,label='0.28 m hard radius'));ax.scatter(*body,c='#d02020',marker='X',s=70,zorder=7);ax.annotate('LAND',body,xytext=(10,-5),textcoords='offset points',color='#d02020')
 sample=next(x for x in geo[f] if x['t']==(21.163 if f=='104759' else 22.5));p=np.array(sample['body_map']);q=np.array(sample['best_10cm_progress_candidate']['end'])
 ax.annotate('',q,p,arrowprops={'arrowstyle':'->','color':'#008b8b','lw':3});ax.plot([],[],c='#008b8b',lw=3,label='Tested wall-away direction*')
 v=np.array(sample['measured_map_velocity']);ax.annotate('',p+v/np.linalg.norm(v)*.16,p,arrowprops={'arrowstyle':'->','color':'#b41a00','lw':2});ax.plot([],[],c='#b41a00',lw=2,label='Measured drift direction')
 goal=at('/planner/effective_goal',end)['point'];ax.scatter(goal['x'],goal['y'],marker='*',c='#198349',s=120);ax.text(goal['x']+.06,goal['y'],'Goal',color='#198349')
 ax.set(xlim=(-.3,1.4),ylim=(-2.15,.4),xlabel='Map x (m)',ylabel='Map y (m)',title=f'Flight {f[:2]}:{f[2:4]}:{f[4:]} — 3 brakes, 2 rejoins\nROUTE {mt["route_seconds"]:.2f} s / brake {mt["preland_seconds"]-mt["route_seconds"]:.2f} s')
 ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=8,loc='lower right')
fig.suptitle('September 20 flights — drift at the tunnel entrance\n*10 cm geometric probe, not a flown or dynamically validated command. Unknown gray / free white / occupied dark.',fontsize=12)
fig.savefig(R/'flight_maps.png',dpi=180)
