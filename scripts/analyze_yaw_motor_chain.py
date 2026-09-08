#!/usr/bin/env python3
"""Offline Sep 8 yaw PID/filter/allocation/heading consistency checks.

Run with .venv-mavlink. Logged rates are subsampled: the filter reconstruction
approximates intervening control iterations and is not a motor/airframe model.
"""
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from analyze_attitude_response import euler_degrees
from analyze_control_response import recent_indices


def main():
    results, traces = {}, {}
    for label, stamp, sd, lo, hi in [
            ('baseline','043031','20_22_29',13.06,48.30),
            ('trial','050015','20_51_29',11.43,45.45)]:
        bag=json.loads(Path(f'reports/lateral_stop_{stamp}.data.json').read_text())
        offset=bag['boot_to_bag_offset']
        u=ULog(f'ros2_ws/flight_logs/px4_{sd}.ulg')
        d={v.name:v.data for v in u.data_list if v.multi_id==0};p=u.initial_parameters
        torque=d['vehicle_torque_setpoint'];gyro=d['vehicle_angular_velocity']
        rates=d['vehicle_rates_setpoint'];integ=d['rate_ctrl_status']
        gi=np.clip(np.searchsorted(gyro['timestamp_sample'],torque['timestamp_sample']),0,len(gyro['timestamp_sample'])-1)
        ri,rok=recent_indices(rates['timestamp'],torque['timestamp'],10000)
        ii,iok=recent_indices(integ['timestamp'],torque['timestamp'],10000)
        t=torque['timestamp_sample']/1e6+offset
        keep=(gyro['timestamp_sample'][gi]==torque['timestamp_sample'])&rok&iok
        k=p['MC_YAWRATE_K'];error=rates['yaw'][ri]-gyro['xyz[2]'][gi]
        proportional=k*p['MC_YAWRATE_P']*error
        integral=integ['yawspeed_integ'][ii]
        raw=proportional+integral-k*p['MC_YAWRATE_D']*gyro['xyz_derivative[2]'][gi]+p['MC_YAWRATE_FF']*rates['yaw'][ri]
        if p.get('MC_BAT_SCALE_EN',0):raise ValueError('battery scaling not modeled')
        tau=1/(2*np.pi*p['MC_YAW_TQ_CUTOFF'])
        pred=np.empty(len(t));pred[0]=torque['xyz[2]'][0]
        for j in range(1,len(t)):
            dt=t[j]-t[j-1]
            if not keep[j] or not keep[j-1] or dt<=0 or dt>.03:
                pred[j]=torque['xyz[2]'][j]
            else:
                pred[j]=pred[j-1]+dt/(tau+dt)*(raw[j]-pred[j-1])
        m=keep&(t>=lo)&(t<hi)
        motors=d['actuator_motors'];mt=motors['timestamp']/1e6+offset
        ti,tok=recent_indices(torque['timestamp'],motors['timestamp'],20000)
        split=(motors['control[0]']+motors['control[1]']-motors['control[2]']-motors['control[3]'])/4
        mm=tok&(mt>=lo)&(mt<hi)
        attitude=d['vehicle_attitude'];ai,aok=recent_indices(attitude['timestamp'],gyro['timestamp'],50000)
        e=np.radians(euler_degrees(np.array([attitude[f'q[{i}]'][ai] for i in range(4)])))
        yawdot=(gyro['xyz[1]']*np.sin(e[0])+gyro['xyz[2]']*np.cos(e[0]))/np.cos(e[1])
        gt=gyro['timestamp_sample']/1e6+offset;gm=aok&(gt>=lo)&(gt<hi)
        tt=gt[gm];delta=np.degrees(np.trapezoid(yawdot[gm],tt))
        yaw=np.unwrap(e[2,gm]);qdelta=np.degrees(yaw[-1]-yaw[0])
        result=dict(ulog=str(Path(f'ros2_ws/flight_logs/px4_{sd}.ulg')),window_s=[lo,hi],
                    filter_time_constant_s=tau,exact_gyro_pairs=int(m.sum()),
                    filter_reconstruction_abs_error_p50_p90_p99=np.percentile(abs(pred[m]-torque['xyz[2]'][m]),[50,90,99]).tolist(),
                    allocation_split_abs_error_p50_p90_p99=np.percentile(abs(split[mm]-torque['xyz[2]'][ti[mm]]),[50,90,99]).tolist(),
                    integrated_gyro_yaw_change_deg=float(delta),attitude_yaw_change_deg=float(qdelta),
                    net_heading_correction_deg=float(qdelta-delta),gyro_max_recorded_gap_s=float(np.max(np.diff(tt))),windows={})
        for name,start,end in [('early',lo,lo+5),('late',hi-5,hi)]:
            q=m&(t>=start)&(t<end)
            result['windows'][name]=dict(time_s=[start,end],p_mean=float(proportional[q].mean()),
                                        i_mean=float(integral[q].mean()),published_torque_mean=float(torque['xyz[2]'][q].mean()),
                                        requested_yaw_rate_mean_deg_s=float(np.degrees(rates['yaw'][ri[q]].mean())),
                                        measured_yaw_rate_mean_deg_s=float(np.degrees(gyro['xyz[2]'][gi[q]].mean())))
        results[label]=result
        traces[label]=(t,m,proportional,integral,raw,pred,torque['xyz[2]'],rates['yaw'][ri],gyro['xyz[2]'][gi],mt,mm,split)
    Path('reports/yaw_motor_chain_20260908.json').write_text(json.dumps(results,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    for col,(label,v) in enumerate(traces.items()):
        t,m,p,i,raw,pred,out,target,actual,mt,mm,split=v
        axes[0,col].plot(t[m],np.degrees(target[m]),label='Requested yaw rate',lw=.7)
        axes[0,col].plot(t[m],np.degrees(actual[m]),label='Gyro yaw rate',lw=.7)
        axes[1,col].plot(t[m],p[m],label='P term',lw=.7)
        axes[1,col].plot(t[m],i[m],label='Logged I term',lw=1)
        axes[2,col].plot(t[m],out[m],label='Published yaw torque',lw=1)
        axes[2,col].plot(t[m],pred[m],'--',label='Reconstructed 2 Hz filtered P+I',lw=.8)
        axes[2,col].plot(mt[mm],split[mm],':',label='Motor split / 4',lw=.8)
        axes[0,col].set_title(label)
        axes[0,col].set_ylabel('Yaw rate (deg/s)');axes[1,col].set_ylabel('Normalized PID contribution');axes[2,col].set_ylabel('Normalized yaw command')
        axes[2,col].set_xlabel('Bag-aligned seconds')
    for ax in axes.flat:ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.suptitle('Yaw correction follows rate error → PID → torque filter → motor allocation')
    fig.savefig('reports/yaw_motor_chain_20260908.png',dpi=150)
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
