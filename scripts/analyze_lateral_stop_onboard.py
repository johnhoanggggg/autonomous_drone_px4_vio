#!/usr/bin/env python3
"""Onboard control/allocator audit for an extracted lateral-stop recording.

Run with .venv-mavlink. All control pairing uses PX4 timestamps. Fixed-axis
tilt acceleration uses the existing near-hover approximation, not accelerometry.
The allocator's achieved flag describes commanded allocation, not motor RPM.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from analyze_attitude_response import analyse as attitude_analyse
from analyze_control_response import analyse as control_analyse


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('ulog', type=Path)
    p.add_argument('--bag-data', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    bag = json.loads(args.bag_data.read_text())
    offset = bag['boot_to_bag_offset']
    log = ULog(str(args.ulog))
    ds = {d.name:d.data for d in log.data_list if d.multi_id == 0}
    windows = [('first_leg',13,19), ('second_hold',22,31.5),
               ('third_leg',32,35), ('last_leg_hold',41.25,48.3)]
    att, summary = attitude_analyse(args.ulog, offset, windows)
    control, _ = control_analyse(args.ulog, offset, 13, 48.3)
    summary['dropout_events'] = [dict(time_s=d.timestamp/1e6+offset,
                                     duration_ms=d.duration) for d in log.dropouts]
    summary['firmware'] = log.msg_info_dict.get('ver_sw')
    for name, lo, hi in windows:
        result = summary['windows'][name]
        for topic, fields in [
                ('rate_ctrl_status', ['rollspeed_integ','pitchspeed_integ','yawspeed_integ']),
                ('control_allocator_status', ['torque_setpoint_achieved','thrust_setpoint_achieved',
                 'unallocated_torque[0]','unallocated_torque[1]','unallocated_torque[2]']),
                ('battery_status', ['voltage_v','current_a'])]:
            data = ds[topic]
            time = data['timestamp']/1e6+offset
            keep = (time>=lo)&(time<hi)
            result[topic] = {f:dict(zip(['min','median','max'],
                                np.percentile(data[f][keep],[0,50,100]).tolist())) for f in fields}
    local = ds['vehicle_local_position']
    stamps = np.array([r['boot'] for r in bag['local']])*1e6
    idx = np.clip(np.searchsorted(local['timestamp'], stamps),0,len(local['timestamp'])-1)
    keep = np.abs(local['timestamp'][idx].astype(float)-stamps)<.1
    summary['matching_local_position_samples'] = int(keep.sum())
    summary['max_matching_x_difference_m'] = float(np.max(np.abs(
        local['x'][idx[keep]]-np.array([r['x'] for r in bag['local']])[keep])))
    args.out.with_suffix('.json').write_text(json.dumps(summary,indent=2)+'\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(5,1,figsize=(12,12),sharex=True,layout='constrained')
    c,s = np.cos(bag['debug'][0]['yaw0']),np.sin(bag['debug'][0]['yaw0'])
    t=control['time_s']
    for x,y,label in [('vx_demand','vy_demand','PX4 velocity demand'),
                       ('vx_actual','vy_actual','Estimated velocity')]:
        axes[0].plot(t,-s*control[x]+c*control[y],label=label)
    for x,y,label in [('ax_demand','ay_demand','PX4 acceleration demand'),
                       ('ax_tilt_equivalent','ay_tilt_equivalent','Actual tilt equivalent')]:
        axes[1].plot(t,-s*control[x]+c*control[y],label=label)
    m=(att['time_s']>=13)&(att['time_s']<=48.3)
    for key,label in [('roll_rate_target_deg_s','Roll rate request'),
                       ('roll_rate_actual_deg_s','Measured roll rate')]:
        axes[2].plot(att['time_s'][m],att[key][m],label=label,lw=.8)
    i=ds['rate_ctrl_status'];ti=i['timestamp']/1e6+offset
    for key in ['rollspeed_integ','pitchspeed_integ','yawspeed_integ']:
        axes[3].plot(ti,i[key],label=key)
    motors=ds['actuator_motors'];tm=motors['timestamp']/1e6+offset
    for k in range(4):
        axes[4].plot(tm,motors[f'control[{k}]'],label=f'Motor {k}',lw=.6)
    for ax,label in zip(axes,['Rightward velocity (m/s)','Rightward acceleration (m/s²)',
                             'Roll rate (deg/s)','Rate integrals (normalized)','Motor command']):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper left')
    axes[-1].set_xlim(13,48.3)
    axes[-1].set_xlabel('Bag-aligned seconds; onboard control quantities share the PX4 clock')
    fig.suptitle('Lateral stop test: control response and growing yaw compensation')
    fig.savefig(args.out.with_suffix('.png'),dpi=150)
    print(json.dumps({k:v for k,v in summary.items() if k!='windows'},indent=2))


if __name__ == '__main__':
    main()
