import json, math, bisect
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation, PillowWriter
R=Path(__file__).parent
d=json.loads((R/'102622.json').read_text())
START,BRAKE,CAPTURE,LAND=8.615,15.471,19.263,35.864

def at(topic,t):
 s=d[topic];i=bisect.bisect_right([v[0] for v in s],t)-1
 return s[max(0,i)][1]
def xy(m):return np.array([m['pose']['position'][k] for k in ['x','y']])
def paint(ax,t,overview=False):
 ax.clear()
 g=at('/rtabmap/grid',t);info=g['info'];res=info['resolution'];ox=info['origin']['position']['x'];oy=info['origin']['position']['y']
 grid=np.array(g['data']).reshape(info['height'],info['width']);colors=np.where(grid<0,0,np.where(grid>=65,2,1))
 ax.imshow(colors,origin='lower',extent=[ox,ox+res*info['width'],oy,oy+res*info['height']],cmap=ListedColormap(['#d8dce3','#fafafa','#263340']),vmin=0,vmax=2,interpolation='nearest')
 path=at('/planner/path',t)['poses'];pp=np.array([xy(p) for p in path]);ax.plot(*pp.T,'--',c='#25844c',lw=1.8,label='Latest planner path')
 body=[(a,xy(m)) for a,m in d['/rtabmap/body_pose'] if START<=a<=t]
 for lo,hi,c,label in [(START,BRAKE,'#246ce0','Measured: ROUTE'),(BRAKE,LAND,'#e07a12','Measured: braking')]:
  track=np.array([p for a,p in body if lo<=a<=hi]);
  if len(track):ax.plot(*track.T,c=c,lw=2.3,label=label)
 targets=[]
 for a,m in d['/fmu/in/trajectory_setpoint']:
  if not START<=a<=t or not math.isfinite(m['position'][0]):continue
  b=at('/rtabmap/body_pose',a);c=at('/rtabmap/odom_correction',a);lp=at('/fmu/out/vehicle_local_position_v1',a)
  q=c['pose']['orientation'];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2));co,si=math.cos(yaw),math.sin(yaw)
  delta=np.array([m['position'][1]-lp['y'],m['position'][0]-lp['x']]);targets.append(xy(b)+np.array([[co,-si],[si,co]])@delta)
 if targets:ax.scatter(*np.array(targets).T,s=5,c='#bc39ab',alpha=.6,label='Finite XY targets')
 for tt,label,marker,c in [(START,'Route starts','o','#246ce0'),(BRAKE,'Brake','s','#bc5700'),(CAPTURE,'First capture','D','#b932a5'),(LAND,'LAND','X','#d32626')]:
  if tt<=t+.03:
   p=xy(at('/rtabmap/body_pose',tt));ax.scatter(*p,s=65,marker=marker,c=c,zorder=8,edgecolors='white',linewidths=.7)
   if overview:ax.annotate(label,p,xytext=(8,9 if tt!=CAPTURE else -20),textcoords='offset points',fontsize=9,color=c)
 p=xy(at('/rtabmap/body_pose',t));ax.add_patch(Circle(p,.28,fill=False,color='#c52626',lw=1.3,label='0.28 m clearance radius'))
 ax.scatter(*p,c='black',s=20,zorder=9)
 if overview:
  ax.set_xlim(ox,ox+res*info['width']);ax.set_ylim(oy,oy+res*info['height'])
 else:
  ax.set_xlim(-.65,1.6);ax.set_ylim(-1.5,.75)
 ax.set_aspect('equal');ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)');ax.grid(alpha=.15)
 state='ROUTE' if t<BRAKE else ('RESERVE_BRAKE' if t<LAND else 'LAND')
 ax.set_title(f'{t:.2f} s — {state}')
 return ax
fig,axes=plt.subplots(1,2,figsize=(13,10),layout='constrained',gridspec_kw={'width_ratios':[1,1.25]})
paint(axes[0],LAND,True);axes[0].set_title('Map at LAND request — full tunnel')
paint(axes[1],LAND);axes[1].set_title('Flight detail — 6.86 s route / 20.39 s braking');axes[1].legend(loc='upper right',fontsize=8)
fig.suptitle('Flight 2026-09-20 10:26:22 UTC\nGray: unknown • white: free • dark: occupied',fontsize=14)
fig.savefig(R/'flight_map.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(16,6),layout='constrained')
for ax,t in zip(axes,[BRAKE,CAPTURE,LAND]):paint(ax,t)
axes[0].legend(loc='lower left',fontsize=7)
fig.suptitle('Recorded map and corrected body pose at each stage; finite targets shown in magenta')
fig.savefig(R/'map_stages.png',dpi=170);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,7),layout='constrained')
def frame(t):
 paint(ax,t);ax.legend(loc='lower left',fontsize=7)
fig.suptitle('10:26 flight — map, route and braking drift')
FuncAnimation(fig,frame,frames=np.arange(START,LAND+.1,.25),interval=100).save(R/'flight_map.gif',writer=PillowWriter(fps=10),dpi=90)
plt.close(fig)
