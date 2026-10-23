from flight_data import *
F=Flight('011120');t=19.97;r=next(x for x in json.loads((R/'diagnosis.json').read_text()) if x['t']==t);g=F.at('/rtabmap/grid',t);i=g['info'];res=i['resolution'];o=i['origin']['position'];grid=np.array(g['data']).reshape(i['height'],i['width']);b=np.array(r['body']);fw=np.array(r['forward'])
fig,axes=plt.subplots(1,2,figsize=(13,6),layout='constrained');ax=axes[0]
ax.imshow(np.where(grid<0,0,np.where(grid>=65,2,1)),origin='lower',extent=[o['x'],o['x']+res*i['width'],o['y'],o['y']+res*i['height']],cmap=ListedColormap(['#d8dce3','#fafafa','#263340']),vmin=0,vmax=2)
path=np.array([pose(m) for m in F.at('/planner/path',t)['poses']]);ax.plot(*path.T,'--',color='forestgreen',label='Planner path: minimum clearance 35.1 cm')
ax.scatter(*b,c='black',s=35,zorder=6,label='Body at recovery exit')
ax.add_patch(Circle(b,.28,fill=False,color='firebrick',label='Required clearance: 28 cm'))
ax.add_patch(Circle(b,.38,fill=False,color='darkorange',linestyle='--',label='Adapter reserve: 38 cm'))
ax.annotate('',b+fw*.6,b,arrowprops=dict(arrowstyle='->',color='green',lw=3));ax.text(*(b+fw*.6+[.02,-.06]),'60 cm forward chord\nminimum clearance 34.8 cm',fontsize=8,color='darkgreen')
ax.set(xlim=(b[0]-.8,b[0]+.65),ylim=(b[1]-1.,b[1]+.5),xlabel='Map x (m)',ylabel='Map y (m)',title='Open planned passage, stricter local reserve');ax.set_aspect('equal');ax.legend(fontsize=7,loc='upper right');ax.grid(alpha=.15)
ax=axes[1]
for k,n,c in [(LP,'Measured speed','royalblue'),(SP,'Commanded speed','darkgreen')]:
 data=[(t,m) for t,m in F.d[k] if 17.2<t<21.1];ax.plot([t for t,m in data],[speed(m) if k==LP else math.hypot(*m['velocity'][:2]) for t,m in data],label=n,c=c)
ax.axvspan(19.973,21.05,color='darkorange',alpha=.15,label='Zero-velocity stop; XY position unset')
ax.axvline(18.808,color='#009981',ls='--');ax.axvline(19.973,color='darkorange',ls='--');ax.axvline(21.05,color='firebrick',ls='--')
ax.text(18.83,.149,'Active recovery',fontsize=8);ax.text(19.99,.149,'Selector rejects\nall full-speed candidates',fontsize=8)
ax.set(xlabel='Seconds from first bag message',ylabel='Horizontal speed (m/s)',title='Forward command disappears before drift grows',ylim=(0,.17));ax.legend(loc='lower left',fontsize=8);ax.grid(alpha=.2)
fig.suptitle('01:11:20 flight — the planner permits the corridor, but recovery requires extra reserve')
fig.savefig(R/'forward_bottleneck.png',dpi=160)
