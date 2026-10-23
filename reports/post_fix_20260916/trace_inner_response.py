"""Read-only control-chain audit; run from repository root with .venv-mavlink."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,'scripts')
from pyulog import ULog
from analyze_control_response import recent_indices
from analyze_attitude_response import euler_degrees
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path('reports/post_fix_20260916')

def stats(x):
    return dict(mean=float(np.mean(x)),p01_p50_p99=np.percentile(x,[1,50,99]).tolist())

def run(label,path,offset,windows):
    u=ULog(path); d={v.name:v.data for v in u.data_list if v.multi_id==0}; p=u.initial_parameters
    tq=d['vehicle_torque_setpoint'];g=d['vehicle_angular_velocity'];r=d['vehicle_rates_setpoint'];integ=d['rate_ctrl_status']
    stamps=tq['timestamp'];t=stamps/1e6+offset
    gi=np.clip(np.searchsorted(g['timestamp_sample'],tq['timestamp_sample']),0,len(g['timestamp'])-1)
    ri,rv=recent_indices(r['timestamp'],stamps,10000)
    # Status is published just before torque in this firmware, after integrating.
    ii,iv=recent_indices(integ['timestamp'],stamps,10000)
    valid=(g['timestamp_sample'][gi]==tq['timestamp_sample'])&rv&iv
    values={}
    for k,axis in enumerate(['roll','pitch']):
        pre='MC_'+axis.upper()+'RATE_'
        target=r[axis][ri]; actual=g[f'xyz[{k}]'][gi]
        pt=p[pre+'K']*p[pre+'P']*(target-actual)
        dt=-p[pre+'K']*p[pre+'D']*g[f'xyz_derivative[{k}]'][gi]
        ff=p[pre+'FF']*target; it=integ[axis+'speed_integ'][ii]
        out=tq[f'xyz[{k}]']; inferred=out-pt-dt-ff
        values[axis]=dict(target_deg_s=np.degrees(target),actual_deg_s=np.degrees(actual),p=pt,d=dt,logged_i=it,inferred_i=inferred,torque=out,reconstruction_error=out-(pt+dt+ff+it))
    results=dict(ulog=path,firmware=u.msg_info_dict.get('ver_sw'),windows={},pairing={})
    full=(t>=windows[0][1])&(t<windows[-1][2])
    results['pairing']=dict(available_torque_samples=int(full.sum()),exact_gyro_fresh_rate_integral_pairs=int((full&valid).sum()),target_age_ms=stats((stamps.astype(float)[full&valid]-r['timestamp'][ri[full&valid]])/1000))
    for name,lo,hi in windows:
        m=valid&(t>=lo)&(t<hi)
        row={axis:{key:stats(v[m]) for key,v in a.items()} for axis,a in values.items()}
        row['interval']=[lo,hi]
        for axis in values:
            a=values[axis];row[axis]['i_opposes_p_fraction']=float(np.mean(a['p'][m]*a['logged_i'][m]<0))
        for topic,fields in [('actuator_motors',[f'control[{i}]' for i in range(4)]),('battery_status',['voltage_v','current_a']),('rate_ctrl_status',['yawspeed_integ'])]:
            src=d[topic];mm=(src['timestamp']/1e6+offset>=lo)&(src['timestamp']/1e6+offset<hi)
            row[topic]={f:stats(src[f][mm]) for f in fields}
        results['windows'][name]=row
    # Exact control-sample pairing avoids interpreting logger subsampling as
    # allocation lag. X-geometry normalization gives 1/(2*sqrt(2)) for roll/pitch.
    mot=d['actuator_motors'];mt=mot['timestamp']/1e6+offset
    ti=np.clip(np.searchsorted(tq['timestamp_sample'],mot['timestamp_sample']),0,len(stamps)-1)
    motor=np.column_stack([mot[f'control[{i}]'] for i in range(4)])
    mm=(tq['timestamp_sample'][ti]==mot['timestamp_sample'])&(mt>=windows[0][1])&(mt<windows[-1][2])&np.all(np.isfinite(motor),axis=1)
    results['allocation']={'exact_pairs':int(mm.sum()),'publish_delay_ms':stats((mot['timestamp'].astype(float)[mm]-stamps[ti[mm]])/1000)}
    for k,(axis,coef) in enumerate(zip(['roll','pitch','yaw'],[np.array([-1,1,1,-1])/(2*np.sqrt(2)),np.array([1,-1,1,-1])/(2*np.sqrt(2)),np.array([1,1,-1,-1])/4])):
        y=tq[f'xyz[{k}]'][ti]
        results['allocation'][axis]=dict(coefficients=coef.tolist(),abs_residual=stats(abs(motor[mm]@coef-y[mm])))
    # Command to logged electrical output mapping with causal matches.
    out=d['actuator_outputs'];mi,mv=recent_indices(mot['timestamp'],out['timestamp'],10000)
    ot=out['timestamp']/1e6+offset;om=mv&(ot>=windows[0][1])&(ot<windows[-1][2])
    results['pwm']={str(k):stats(out[f'output[{k}]'][om]-(p[f'PWM_MAIN_MIN{k+1}']+(p[f'PWM_MAIN_MAX{k+1}']-p[f'PWM_MAIN_MIN{k+1}'])*motor[mi[om],k])) for k in range(4)}
    # Compare 50 ms averages because separately subsampled log topics do not
    # preserve every intermediate PWM/torque update. This is not a wire capture.
    pwm_motor=np.column_stack([(out[f'output[{i}]']-p[f'PWM_MAIN_MIN{i+1}'])/(p[f'PWM_MAIN_MAX{i+1}']-p[f'PWM_MAIN_MIN{i+1}']) for i in range(4)])
    coef=np.array([[-1,1,1,-1],[1,-1,1,-1]])/(2*np.sqrt(2))
    bins=[]
    for begin in np.arange(windows[0][1],windows[-1][2]-.049,.05):
        pm=(ot>=begin)&(ot<begin+.05);tm=(t>=begin)&(t<begin+.05)
        if pm.sum()>=5 and tm.sum()>=5:
            bins.append([*[float(tq[f'xyz[{k}]'][tm].mean()) for k in range(2)],*(pwm_motor[pm].mean(axis=0)@coef.T)])
    bins=np.array(bins);results['pwm_50ms']={}
    for k,axis in enumerate(['roll','pitch']):
        x=bins[:,k];y=bins[:,k+2]
        results['pwm_50ms'][axis]=dict(bins=len(x),correlation=float(np.corrcoef(x,y)[0,1]),mean_difference=float(np.mean(y-x)),rms_difference=float(np.sqrt(np.mean((y-x)**2))))
    # Does VIO also see the slow roll/pitch motion? Match EV's delayed orientation
    # to FC orientation, subtract only a constant offset; this is not ground truth.
    ev=d['vehicle_visual_odometry'];a=d['vehicle_attitude']
    et=ev['timestamp_sample']/1e6+offset-.215;at=a['timestamp_sample']/1e6+offset
    ee=euler_degrees(np.array([ev[f'q[{i}]'] for i in range(4)]));ae=euler_degrees(np.array([a[f'q[{i}]'] for i in range(4)]))
    em=(et>=windows[0][1])&(et<windows[-1][2])&(et>=at[0])&(et<=at[-1]); results['ev_vs_fc']={}
    for k,axis in enumerate(['roll','pitch']):
        aa=np.interp(et[em],at,ae[k]);diff=ee[k,em]-aa
        results['ev_vs_fc'][axis]=dict(ev_minus_fc_deg=stats(diff),correlation=float(np.corrcoef(ee[k,em],aa)[0,1]))
    # Euler integration checks whether attitude corrections, not physical gyro
    # motion, dominate reported roll/pitch changes. Evaluate in each short window.
    gt=g['timestamp_sample']/1e6+offset
    roll=np.radians(np.interp(gt,at,ae[0]));pitch=np.radians(np.interp(gt,at,ae[1]))
    xyz=np.array([g[f'xyz[{k}]'] for k in range(3)])
    euler_rate=np.array([xyz[0]+np.sin(roll)*np.tan(pitch)*xyz[1]+np.cos(roll)*np.tan(pitch)*xyz[2],np.cos(roll)*xyz[1]-np.sin(roll)*xyz[2]])
    for name,lo,hi in windows:
        gm=(gt>=lo)&(gt<hi);tt=gt[gm]
        results['windows'][name]['gyro_vs_attitude']={axis:dict(gyro_integral_deg=float(np.degrees(np.trapezoid(euler_rate[k,gm],tt))),attitude_change_deg=float(np.interp(tt[-1],at,ae[k])-np.interp(tt[0],at,ae[k]))) for k,axis in enumerate(['roll','pitch'])}
    fig,axs=plt.subplots(4,2,figsize=(13,11),sharex=True,layout='constrained')
    use=valid&full
    for c,axis in enumerate(['roll','pitch']):
        av=values[axis]
        axs[0,c].plot(t[use],av['target_deg_s'][use],label='Rate target')
        axs[0,c].plot(t[use],av['actual_deg_s'][use],label='Gyro',lw=.6)
        for f in ['p','logged_i','d','torque']:axs[1,c].plot(t[use],av[f][use],label=f,lw=.7)
        am=(at>=windows[0][1])&(at<windows[-1][2])
        axs[2,c].plot(at[am],ae[c,am],label='FC attitude')
        axs[2,c].plot(et[em],ee[c,em],label='VIO, shifted 215 ms',lw=.8)
        axs[3,c].plot(mt[mm],motor[mm,2*c],label=f'Motor {2*c+1}')
        axs[3,c].plot(mt[mm],motor[mm,2*c+1],label=f'Motor {2*c+2}')
        axs[0,c].set_title(axis);axs[0,c].set_ylabel('deg/s');axs[1,c].set_ylabel('Normalized torque');axs[2,c].set_ylabel('deg');axs[3,c].set_ylabel('Normalized output');axs[3,c].set_xlabel('Bag-aligned seconds')
    for ax in axs.flat:
        ax.grid(alpha=.2);ax.legend(fontsize=8)
        for _,lo,hi in windows[1:-1]:ax.axvspan(lo,hi,alpha=.08,color='orange')
    fig.suptitle(label+' — recorded controller terms and observed motion')
    fig.savefig(OUT/f'inner_trace_{label}.png',dpi=140);plt.close(fig)
    return results

results={}
for label,path,off,win in [
 ('first','ros2_ws/flight_logs/offboard_global_20260916T081620Z/px4_23_28_02.ulg',-131.5378796755,[('before',11,14.5),('lag',14.5,16.5),('response',16.5,17.5),('after',17.5,20)]),
 ('second','ros2_ws/flight_logs/offboard_global_20260916T081709Z/px4_23_28_50.ulg',-179.9391028155,[('before',12,15.6),('lag',15.6,16.6),('response',16.6,17.38),('after',17.38,20)])]:
    results[label]=run(label,path,off,win)
(OUT/'inner_trace.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
for label,r in results.items():
    print(label,'EV/FC',r['ev_vs_fc'],'PWM',r['pwm'],'allocation',r['allocation'])
    for name,w in r['windows'].items():
        print(name,'gyro/attitude',w['gyro_vs_attitude'])
        for axis in ['roll','pitch']:print(axis,{k:round(v['mean'],6) for k,v in w[axis].items() if isinstance(v,dict)})
