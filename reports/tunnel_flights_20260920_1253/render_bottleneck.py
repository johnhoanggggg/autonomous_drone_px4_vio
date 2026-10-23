from flight_data import *
import re
F=Flight('125509');D=json.loads((R/'details.json').read_text())['125509'];rows=[s for s in D['snapshots'] if 30<=s['t']<=32.2]
fig,axs=plt.subplots(1,2,figsize=(12,5),layout='constrained')
ax=axs[0]
for n,label in [(0,'Follower direction: geometric cap'),(1,'Existing command direction: geometric cap')]:ax.plot([s['t'] for s in rows],[s['snapshot']['caps'][n] for s in rows],'o-',label=label)
sps=[(t,m) for t,m in F.d[SP] if 29.99<=t<=32.6]
ax.plot([t for t,m in sps],[math.hypot(*m['velocity'][:2]) for t,m in sps],label='Published velocity command',alpha=.8)
ax.axhline(.06,color='#777',ls='--',label='Slow rejoin ceiling');ax.set(xlabel='Seconds from bag start',ylabel='m/s',title='Direction limits the tunnel command');ax.legend(fontsize=8);ax.grid(alpha=.2)
ax=axs[1];s=rows[-2];t=s['t'];g=F.at('/rtabmap/grid',t);inf=g['info'];res=inf['resolution'];ox=inf['origin']['position']['x'];oy=inf['origin']['position']['y'];grid=np.array(g['data']).reshape(inf['height'],inf['width'])
ax.imshow(np.where(grid<0,0,np.where(grid>=65,2,1)),origin='lower',extent=[ox,ox+res*inf['width'],oy,oy+res*inf['height']],cmap=ListedColormap(['#d8dce3','#fafafa','#263340']),vmin=0,vmax=2,interpolation='nearest')
p=np.array(s['body_map']);ax.scatter(*p,c='k',s=45,zorder=5,label='Body');ax.add_patch(Circle(p,.28,fill=False,color='red',label='Hard clearance radius'))
for v,c,label in [(s['desired_direction_map'],'#246ce0','Follower direction'),(s['command_velocity_map'],'#dc851c','Published command direction')]:
 v=np.array(v);v=v/np.linalg.norm(v)*.25;ax.arrow(*p,*v,width=.003,head_width=.025,color=c,length_includes_head=True,label=label)
ax.set(xlim=(p[0]-.6,p[0]+.6),ylim=(p[1]-.6,p[1]+.4),xlabel='Map x (m)',ylabel='Map y (m)',title=f'{t:.2f}s: command points toward wall');ax.set_aspect('equal');ax.legend(fontsize=8,loc='lower left')
fig.suptitle('Third flight — recorded-state reconstruction; direction arrows normalized for visibility')
fig.savefig(R/'speed_bottleneck.png',dpi=160)
