#!/usr/bin/env python3
"""Offline comparison of the Sep 8 baseline and roll-P trial; needs .venv-mavlink.

The trial ROS tail is truncated. Leg 4 hold timing comes from the surviving node
log (35.3798 s) and onboard LAND command (45.455 s). Never modifies recordings.
"""
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from analyze_attitude_response import analyse


def main():
    flights = [('baseline', '043031', '20_22_29'), ('trial', '050015', '20_51_29')]
    results, traces = {}, {}
    for label, stamp, sd in flights:
        bag = json.loads(Path(f'reports/lateral_stop_{stamp}.data.json').read_text())
        summary = json.loads(Path(f'reports/lateral_stop_{stamp}.json').read_text())
        path = Path(f'ros2_ws/flight_logs/px4_{sd}.ulg')
        u = ULog(str(path))
        ds = {d.name: d.data for d in u.data_list if d.multi_id == 0}
        offset = bag['boot_to_bag_offset']
        legs = summary['legs'].copy()
        if label == 'trial':
            rows = [r for r in bag['debug'] if r['leg'] == 3]
            legs.append(dict(leg=3, start_s=rows[0]['time'],
                             brake_s=next(r['time'] for r in rows if r['phase']=='BRAKE'),
                             reference_stop_s=35.379771217, hold_end_s=45.455,
                             target_m=0.0))
        windows = []
        for leg in legs:
            n = leg['leg']+1
            windows += [(f'move{n}', leg['start_s'], leg['reference_stop_s']),
                        (f'hold{n}_first3s', leg['reference_stop_s'], leg['reference_stop_s']+3),
                        (f'hold{n}_full', leg['reference_stop_s'], leg['hold_end_s'])]
        att, stats = analyse(path, offset, windows)
        lp = ds['vehicle_local_position']; t = lp['timestamp']/1e6+offset
        yaw = bag['debug'][0]['yaw0']; s,c = np.sin(yaw),np.cos(yaw)
        x0,y0 = bag['debug'][0]['origin']
        right = -s*(lp['x']-x0)+c*(lp['y']-y0)
        vr = -s*lp['vx']+c*lp['vy']
        motion = []
        for leg in legs:
            start,brake,stop,end = [leg[k] for k in ['start_s','brake_s','reference_stop_s','hold_end_s']]
            sign = 1 if leg['leg'] in (0,3) else -1
            hold = (t>=stop)&(t<end); move=(t>=start)&(t<stop)
            after = np.flatnonzero((t>=brake)&(t<end))
            reversal = next((i for i in after if sign*vr[i]<=0),after[-1])
            first = after[after<=reversal]
            motion.append(dict(leg=leg['leg']+1, start_s=start, brake_s=brake,
                               stop_s=stop, end_s=end,
                               peak_move_speed=float(np.max(sign*vr[move])),
                               first_overshoot_m=float(max(0,np.max(sign*(right[first]-leg['target_m'])))),
                               hold_range_m=[float(np.min(right[hold]-leg['target_m'])),float(np.max(right[hold]-leg['target_m']))],
                               hold_span_m=float(np.ptp(right[hold])),
                               hold_duration_s=end-stop))
        for name,lo,hi in windows:
            w=stats['windows'][name]
            m=(att['time_s']>=lo)&(att['time_s']<hi)
            for axis in ['roll','pitch','yaw']:
                w[axis]['rate_demand_rms_deg_s']=float(np.sqrt(np.mean(att[axis+'_rate_target_deg_s'][m]**2)))
                w[axis]['rate_actual_peak_abs_deg_s']=float(np.max(np.abs(att[axis+'_rate_actual_deg_s'][m])))
            for topic,fields in [('rate_ctrl_status',['rollspeed_integ','yawspeed_integ']),
                                 ('battery_status',['voltage_v','current_a']),
                                 ('control_allocator_status',['torque_setpoint_achieved','thrust_setpoint_achieved'])]:
                d=ds[topic];q=(d['timestamp']/1e6+offset>=lo)&(d['timestamp']/1e6+offset<hi)
                w[topic]={f:dict(zip(['min','median','max'],map(float,np.percentile(d[f][q],[0,50,100])))) for f in fields}
        results[label]=dict(ulog=str(path),bag=bag['bag'],offset=offset,
                            roll_p=u.initial_parameters['MC_ROLLRATE_P'],
                            motion=motion,windows=stats['windows'],
                            dropouts=[dict(t=d.timestamp/1e6+offset,ms=d.duration) for d in u.dropouts])
        traces[label]=(att,ds,offset,t,right,vr,motion)
    Path('reports/lateral_tuning_comparison.json').write_text(json.dumps(results,indent=2)+'\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    for label,(att,ds,o,t,right,vr,motion) in traces.items():
        for j,legn in enumerate([2,4]):
            leg=motion[legn-1];lo=leg['stop_s'];hi=leg['end_s']
            keep=(t>=lo)&(t<hi)
            axes[0,j].plot(t[keep]-lo,right[keep]*100,label=label)
            axes[1,j].plot(t[keep]-lo,vr[keep],label=label)
            axes[0,j].set_title(f'Hold after leg {legn}: stationary target at zero')
            for k in [0,1]: axes[k,j].set_xlabel('Seconds after reference stop')
        leg=motion[2];lo=leg['start_s'];hi=leg['stop_s']+3
        keep=(att['time_s']>=lo)&(att['time_s']<hi)
        for key,style in [('roll_rate_target_deg_s','--'),('roll_rate_actual_deg_s','-')]:
            axes[2,0].plot(att['time_s'][keep]-lo,att[key][keep],style,label=label+' '+('target' if 'target' in key else 'actual'),lw=.8)
        integ=ds['rate_ctrl_status'];ti=integ['timestamp']/1e6+o
        first=motion[0]['start_s'];end=motion[-1]['end_s'];keep=(ti>=first)&(ti<end)
        axes[2,1].plot(ti[keep]-first,integ['yawspeed_integ'][keep],label=label)
    for ax in axes.flat:
        ax.grid(alpha=.2);ax.axhline(0,color='gray',lw=.5);ax.legend(fontsize=8)
    axes[0,0].set_ylabel('Lateral position error (cm)');axes[0,1].set_ylabel('Lateral position error (cm)')
    axes[1,0].set_ylabel('Lateral velocity (m/s)');axes[1,1].set_ylabel('Lateral velocity (m/s)')
    axes[2,0].set_title('Third leg roll-rate tracking');axes[2,0].set_ylabel('Roll rate (deg/s)');axes[2,0].set_xlabel('Seconds after third leg starts')
    axes[2,1].set_title('Yaw correction throughout the maneuver');axes[2,1].set_ylabel('Yaw integral');axes[2,1].set_xlabel('Seconds after first leg starts')
    fig.suptitle('Roll-rate P 0.150 → 0.165: onboard comparison; hold lengths differ')
    fig.savefig('reports/lateral_tuning_comparison.png',dpi=150)
    for label,r in results.items():
        print(label,'motion',json.dumps(r['motion']))
        for name,w in r['windows'].items():
            print(name,'roll err/demand RMS',round(w['roll']['rate_error_rms_deg_s'],3),round(w['roll']['rate_demand_rms_deg_s'],3),
                  'roll angle RMS',round(w['roll']['angle_error_rms_deg'],3),
                  'yaw err RMS',round(w['yaw']['rate_error_rms_deg_s'],3),
                  'yaw I',w['rate_ctrl_status']['yawspeed_integ'],
                  'voltage',round(w['battery_status']['voltage_v']['median'],3))


if __name__=='__main__':
    main()
