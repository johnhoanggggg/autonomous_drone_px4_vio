"""Compare the user's ON/OFF/ON trial with matched onboard evidence."""
import json
import math
import sys
from collections import Counter
from pathlib import Path
import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_lateral_stop import summarize
from analyze_control_response import analyse as control_analyse
from analyze_attitude_response import analyse as attitude_analyse

results=[]
fig,axs=plt.subplots(4,3,figsize=(16,12),layout='constrained')
compare,cax=plt.subplots(3,1,figsize=(10,9),layout='constrained',sharex=True)
response,rax=plt.subplots(3,3,figsize=(15,9),layout='constrained')
for col,alignment in enumerate(json.loads((OUT/'alignment.json').read_text())):
    f=alignment['flight'];offset=alignment['offset']
    b=json.loads((OUT/f'{f}.data.json').read_text())
    active=[r for r in b['debug'] if r['state'] in ['LATERAL_MOVE','LATERAL_HOLD']]
    result=summarize(dict(b,debug=active))
    result.update(flight=f,alignment=alignment,events=b['events'])
    enabled=result['config']['brake_damping']
    assert enabled == (f != '082834')
    u=ULog(str(ROOT/alignment['ulog']))
    d={x.name:x.data for x in u.data_list if x.multi_id==0}
    lp,tr=d['vehicle_local_position'],d['trajectory_setpoint']
    t=lp['timestamp']/1e6+offset;tt=tr['timestamp']/1e6+offset
    yaw=b['debug'][0]['yaw0'];origin=b['debug'][0]['origin']
    lateral=np.array([-np.sin(yaw),np.cos(yaw)])
    forward=np.array([np.cos(yaw),np.sin(yaw)])
    position=np.column_stack([lp['x'],lp['y']])-origin
    right=position@lateral;front=position@forward
    velocity=np.column_stack([lp['vx'],lp['vy']]);vr=velocity@lateral
    refp=np.column_stack([tr['position[0]'],tr['position[1]']])-origin
    refv=np.column_stack([tr['velocity[0]'],tr['velocity[1]']])
    refa=np.column_stack([tr['acceleration[0]'],tr['acceleration[1]']])
    result['ros_tail_truncated']=f=='082906'
    if f=='082906':
        # Recover missing legs from onboard reference motion, before AUTO.LAND.
        landing=next(r['time'] for r in alignment['events'] if 'Landing' in r['text'])
        moving=np.linalg.norm(refv,axis=1)>1e-6
        starts=np.flatnonzero(moving & ~np.r_[False,moving[:-1]] & (tt<landing))
        stops=np.flatnonzero(~moving & np.r_[False,moving[:-1]] & (tt<landing))
        assert len(starts)==len(stops)==4
        for j in (2,3):
            stop=stops[j]
            # For the last hold, use the last stationary-target command before LAND.
            end=starts[j+1]-1 if j<3 else np.flatnonzero((tt<landing)&(~moving)&(np.linalg.norm(refp,axis=1)<1e-5))[-1]
            result['legs'].append(dict(leg=j,direction='left' if j==2 else 'right',
                start_s=float(tt[starts[j]]),brake_s=float(tt[stop]-result['config']['ramp_s']),
                reference_stop_s=float(tt[stop]),hold_end_s=float(tt[end]),
                target_m=result['config']['endpoints'][j],boundaries_from_onboard=True))
        assert 9.8<result['legs'][-1]['hold_end_s']-result['legs'][-1]['reference_stop_s']<10.1
    lo=result['legs'][0]['start_s'];hi=result['legs'][-1]['hold_end_s']
    ctl,_=control_analyse(ROOT/alignment['ulog'],offset,lo,hi)
    response_fields={name:np.column_stack([ctl[key] for key in fields])@lateral
        for name,fields in dict(velocity=['vx_actual','vy_actual'],
            velocity_demand=['vx_demand','vy_demand'],
            acceleration_demand=['ax_demand','ay_demand'],
            tilt_equivalent_acceleration=['ax_tilt_equivalent','ay_tilt_equivalent']).items()}
    sample_times={'082729':[36,37], '082834':[18,19,20,21],
                  '082906':[17.5,18,18.5,19,44,45,50]}[f]
    result['response_samples']=[dict(time_s=sample,**{key:float(np.interp(sample,ctl['time_s'],value))
        for key,value in response_fields.items()}) for sample in sample_times]
    response_leg=result['legs'][3 if f=='082729' else 0]
    response_start=response_leg['reference_stop_s']
    response_end=min(response_leg['hold_end_s'],response_start+7)
    rax[0,col].plot(t,right,label='Estimated position')
    rax[0,col].plot(tt,refp@lateral,label='Reference')
    for key in ('velocity','velocity_demand'):
        rax[1,col].plot(ctl['time_s'],response_fields[key],label=key.replace('_',' '))
    for key in ('acceleration_demand','tilt_equivalent_acceleration'):
        rax[2,col].plot(ctl['time_s'],response_fields[key],label=key.replace('_',' '))
    for ax,ylabel in zip(rax[:,col],['Lateral position (m)','Lateral velocity (m/s)','Lateral acceleration (m/s²)']):
        ax.set(xlim=(response_start,response_end),ylabel=ylabel);ax.grid(alpha=.2);ax.legend(fontsize=7)
        ax.axhline(0,color='gray',lw=.6)
    rax[0,col].set_title(f"{f}: {'last' if f=='082729' else 'first'} hold")
    rax[2,col].set_xlabel('Bag-aligned seconds')
    windows=[(f"leg{leg['leg']+1}_hold_first3",leg['reference_stop_s'],leg['reference_stop_s']+3)
             for leg in result['legs']]
    _,astats=attitude_analyse(ROOT/alignment['ulog'],offset,windows)
    result['attitude_first3']=astats['windows']
    for leg in result['legs']:
        a,z=leg['reference_stop_s'],leg['hold_end_s']
        target=leg['target_m'];hold=(t>=a)&(t<=z);first=hold&(t<=a+3)
        assert lp['xy_valid'][hold].all() and lp['v_xy_valid'][hold].all()
        speed=np.linalg.norm(velocity,axis=1)
        yaw_error=np.degrees(np.arctan2(np.sin(lp['heading']-yaw),np.cos(lp['heading']-yaw)))
        gates=dict(position=np.hypot(right-target,front)<=.1,speed=speed<=.06,
                   yaw=abs(yaw_error)<=5,altitude=abs(-lp['z']-.3)<=.07)
        good=np.logical_and.reduce(list(gates.values()))
        began=None;longest=eligible=0.
        for ti,ok in zip(t[hold],good[hold]):
            if ok:
                if began is None:began=ti
                longest=max(longest,ti-began)
                if ti-a>=3:eligible=max(eligible,ti-began)
            else:began=None
        wire=(tt>=a+.1)&(tt<=z-.1)
        assert np.max(np.ptp(refp[wire],axis=0))==0
        assert np.max(abs(refv[wire]))==0
        acceleration=np.linalg.norm(refa[wire],axis=1)
        assert acceleration.max()<=.100001
        if not enabled:assert acceleration.max()==0
        sign=1 if leg['direction']=='right' else -1
        braking_t=leg['brake_s']
        va,vb=sign*np.interp([braking_t,a],t,vr)
        after_brake=(t>=braking_t)&(t<=z)
        leg.update(peak_speed_m_s=float(np.max(sign*vr[(t>=leg['start_s'])&(t<=z)])),
                   hold_duration_s=z-a,
                   hold_span_m=float(np.ptp(right[hold])),
                   endpoint_overshoot_m=float(max(0,np.max(sign*(right[after_brake]-target)))),
                   hold_max_forward_error_m=float(np.max(abs(front[hold]))),
                   hold_max_abs_yaw_error_deg=float(np.max(abs(yaw_error[hold]))),
                   hold_speed_above_gate_fraction=float(np.mean(speed[hold]>.06)),
                   first3_span_m=float(np.ptp(right[first])),
                   first3_speed_p95=float(np.percentile(speed[first],95)),
                   first3_lateral_speed_rms=float(np.sqrt(np.mean(vr[first]**2))),
                   longest_dwell_s=float(longest),eligible_dwell_s=float(eligible),
                   gate_failure_fraction={k:float(np.mean(~v[hold])) for k,v in gates.items()},
                   speed_at_reference_stop_m_s=float(vb),
                   average_braking_deceleration_m_s2=float((va-vb)/(a-braking_t)),
                   onboard_hold_acceleration_max=float(acceleration.max()),
                   hold_correction_above_095_fraction=float(np.mean(acceleration>.095)),
                   onboard_hold_acceleration_p95=float(np.percentile(acceleration,95)))
        for ax in axs[:,col]:ax.axvspan(braking_t,a,color='orange',alpha=.15)
    result['estimator_reset_counters']={k:np.unique(lp[k]).tolist() for k in lp if 'reset_counter' in k}
    result['damping_reasons']=dict(Counter(r['damping']['reason'] for r in active))
    result['maximum_velocity_age_s']=max(r['damping']['velocity_age_s'] for r in active)
    result['max_applied_correction_m_s2']=max(math.hypot(*r['damping']['applied_correction_ned']) for r in active)
    result['parameters']={k:v for k,v in u.initial_parameters.items() if k.startswith(
        ('MPC_XY','MC_ROLL','MC_PITCH','MC_YAW','EKF2_EV_DELAY','SENS_BOARD'))}
    result['parameter_changes']=u.changed_parameters
    # Battery figures use the actual matched first three hold seconds.
    battery=d['battery_status'];bt=battery['timestamp']/1e6+offset
    a=result['legs'][0]['reference_stop_s'];bm=(bt>=a)&(bt<a+3)
    result['first_hold_voltage_median']=float(np.median(battery['voltage_v'][bm]))
    result['first_hold_current_median']=float(np.median(battery['current_a'][bm]))
    first=result['legs'][0]
    rosfirst=[r for r in b['local'] if first['reference_stop_s']<=r['time']<=first['reference_stop_s']+3]
    result['first3_ros_lateral_speed_rms']=float(np.sqrt(np.mean([r['vr']**2 for r in rosfirst])))
    result['first3_ros_max_gap_s']=float(np.max(np.diff([r['time'] for r in rosfirst])))
    hold=(t>=first['reference_stop_s'])&(t<=first['hold_end_s'])
    label=f"{f}: {'ON' if enabled else 'OFF'}"
    cax[0].plot(t[hold]-first['reference_stop_s'],right[hold]-.35,label=label)
    cax[1].plot(t[hold]-first['reference_stop_s'],np.linalg.norm(velocity[hold],axis=1),label=label)
    cax[2].plot(t[hold]-first['reference_stop_s'],yaw_error[hold],label=label)
    axs[0,col].plot(t,right,label='Estimated lateral position')
    axs[0,col].plot(tt,refp@lateral,label='Reference')
    axs[1,col].plot(t,vr,label='Estimated lateral velocity')
    axs[1,col].plot(ctl['time_s'],np.column_stack([ctl['vx_demand'],ctl['vy_demand']])@lateral,label='PX4 internal velocity demand')
    axs[2,col].plot(tt,refa@lateral,label='Published lateral acceleration')
    axs[3,col].plot(t,front,label='Forward position error (m)')
    axs[3,col].axhline(.1,color='gray',ls=':');axs[3,col].axhline(-.1,color='gray',ls=':')
    axs[0,col].set_title(label)
    if f=='082906':
        for ax in axs[:,col]:ax.axvline(b['local'][-1]['time'],color='red',ls=':',label='ROS truncation; ULog continues')
    for i,title in enumerate(['Position (m)','Velocity (m/s)','Feedforward (m/s²)','Forward offset (m)']):
        axs[i,col].set(xlim=(lo-.3,hi),ylabel=title);axs[i,col].grid(alpha=.2);axs[i,col].legend(fontsize=7)
    axs[3,col].set_xlabel('Bag-aligned seconds; orange = reference braking')
    results.append(result)
assert results[0]['parameters']==results[1]['parameters']==results[2]['parameters']
assert all(not r['parameter_changes'] for r in results)
assert all(len(values)==1 for r in results for values in r['estimator_reset_counters'].values())
response.suptitle('PX4 already requests braking; achieved tilt can lag or oppose it\nTilt-equivalent acceleration assumes near-hover thrust; it is not measured net acceleration')
response.savefig(OUT/'control_response.png',dpi=150)
fig.suptitle('Damping ON / OFF / ON: measured flights, gain 0.75; onboard recovery of third flight tail')
fig.savefig(OUT/'all_flights.png',dpi=150)
for ax,label in zip(cax,['Lateral error from endpoint (m)','Horizontal speed (m/s)','Yaw error (degrees)']):
    ax.set(xlim=(0,3),ylabel=label);ax.grid(alpha=.2);ax.legend(fontsize=8)
cax[1].axhline(.06,color='gray',ls='--');cax[2].axhline(5,color='gray',ls='--');cax[2].axhline(-5,color='gray',ls='--')
cax[2].set_xlabel('Seconds after first reference stop; same 3-second comparison window')
compare.suptitle('First stop: equal-duration comparison, not whole-hold comparisons')
compare.savefig(OUT/'first_stop_comparison.png',dpi=160)
(OUT/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
for result in results:
    print(result['flight'],'enabled',result['config']['brake_damping'],
          'battery',result['first_hold_voltage_median'])
    for leg in result['legs']:
        print(leg)
