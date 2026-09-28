#!/usr/bin/env python3
"""Extract a lateral-stop ROS bag, or plot its extracted JSON without ROS.

Bag extraction needs sourced ROS. --data uses .venv-mavlink for plotting.
All times are relative to the earliest selected ROS receipt timestamp.
First zero crossing is a reversal, not proof of settled hover or braking latency.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def extract(bag):
    from analyze_yaw_tracking import read
    names = ["/lateral_stop/debug", "/fmu/out/vehicle_local_position_v1",
             "/fmu/out/vehicle_attitude", "/fmu/out/sensor_combined",
             "/fmu/out/battery_status_v1", "/rtabmap/vio_feature_count", "/rosout",
             "/rtabmap/body_vio_pose", "/fmu/in/vehicle_command"]
    data, t0 = read(bag, names)
    debug = [dict(time=(t-t0)/1e9, **json.loads(m.data))
             for t, m in data[names[0]]]
    if not debug:
        raise ValueError("no lateral_stop debug messages")
    yaw = debug[0]["yaw0"]
    x0, y0 = debug[0]["origin"]
    # 'right' is the displacement along the TEST axis and 'forward' the cross
    # axis. For axis=forward runs the test axis is the nose, so rotate the
    # frame by +90 deg: 'right' then measures along the heading.
    axis = debug[0].get("config", {}).get("axis", "lateral")
    frame_yaw = yaw - math.pi / 2 if axis == "forward" else yaw
    c, s = math.cos(frame_yaw), math.sin(frame_yaw)
    local = []
    for t, m in data[names[1]]:
        local.append(dict(time=(t-t0)/1e9, boot=m.timestamp/1e6,
                          x=m.x, y=m.y, z=m.z, vx=m.vx, vy=m.vy,
                          ax=m.ax, ay=m.ay, heading=m.heading,
                          right=-(m.x-x0)*s+(m.y-y0)*c,
                          forward=(m.x-x0)*c+(m.y-y0)*s,
                          vr=-m.vx*s+m.vy*c, vf=m.vx*c+m.vy*s,
                          yaw_error=math.degrees(math.atan2(math.sin(m.heading-yaw),
                                                           math.cos(m.heading-yaw))),
                          valid=bool(m.xy_valid and m.v_xy_valid)))
    output = dict(bag=str(bag), t0=t0, debug=debug, local=local,
                  boot_to_bag_offset=float(np.median([r['time']-r['boot'] for r in local])),
                  decode_errors=data['_decode_errors'])
    output['attitude'] = [dict(time=(t-t0)/1e9, boot=m.timestamp/1e6, q=list(map(float, m.q)))
                          for t, m in data[names[2]]]
    output['sensor'] = [dict(time=(t-t0)/1e9, boot=m.timestamp/1e6,
                            gyro=list(map(float, m.gyro_rad)), accel=list(map(float, m.accelerometer_m_s2)))
                        for t, m in data[names[3]]]
    output['battery'] = [dict(time=(t-t0)/1e9, voltage=m.voltage_v, current=m.current_a)
                         for t, m in data[names[4]]]
    output['features'] = [dict(time=(t-t0)/1e9, count=m.data) for t, m in data[names[5]]]
    output['events'] = [dict(time=(t-t0)/1e9, text=m.msg) for t, m in data['/rosout']
                        if m.name == 'offboard_lateral_stop' and m.level >= 30]
    output['vio'] = [dict(time=(t-t0)/1e9, x=m.pose.position.x, y=m.pose.position.y,
                         z=m.pose.position.z) for t, m in data['/rtabmap/body_vio_pose']]
    output['commands'] = [dict(time=(t-t0)/1e9, command=m.command,
                              param1=m.param1, param2=m.param2)
                          for t, m in data['/fmu/in/vehicle_command']]
    return output


def summarize(data):
    debug, lp = data['debug'], data['local']
    config = debug[0]['config']
    legs = []
    for leg in range(4):
        trace = [r for r in debug if r['leg'] == leg]
        brakes = [r for r in trace if r['phase'] == 'BRAKE']
        holds = [r for r in trace if r['phase'] == 'HOLD']
        if not trace or not brakes or not holds:
            continue
        start, brake, end, finish = trace[0]['time'], brakes[0]['time'], holds[0]['time'], trace[-1]['time']
        sign = 1 if leg in (0, 3) else -1
        target = config['endpoints'][leg]
        points = [r for r in lp if start <= r['time'] <= finish and r['valid']]
        after = [r for r in points if r['time'] >= brake]
        hold = [r for r in points if r['time'] >= end]
        at = max((r for r in lp if r['time'] <= brake), key=lambda r:r['time'])
        reversal = next((r for r in after if sign*r['vr'] <= 0), None)
        extreme = max(after, key=lambda r:sign*r['right'])
        first_stop = [r for r in after if reversal is None or r['time'] <= reversal['time']]
        first_extreme = max(first_stop, key=lambda r:sign*r['right'])
        loaded = [r['voltage'] for r in data['battery']
                  if start <= r['time'] <= finish and r['current'] > 20]
        legs.append(dict(leg=leg, direction='right' if sign>0 else 'left',
                         start_s=start, brake_s=brake, reference_stop_s=end, hold_end_s=finish,
                         target_m=target, peak_speed_m_s=max(sign*r['vr'] for r in points),
                         speed_at_brake_m_s=sign*at['vr'],
                         first_reversal_after_brake_s=None if reversal is None else reversal['time']-brake,
                         first_stop_travel_m=sign*(first_extreme['right']-at['right']),
                         first_stop_endpoint_overshoot_m=max(0., sign*(first_extreme['right']-target)),
                         max_travel_after_brake_m=sign*(extreme['right']-at['right']),
                         endpoint_overshoot_m=max(0., sign*(extreme['right']-target)),
                         hold_duration_s=finish-end,
                         hold_max_forward_error_m=max(abs(r['forward']) for r in hold),
                         hold_max_abs_yaw_error_deg=max(abs(r['yaw_error']) for r in hold),
                         hold_speed_above_gate_fraction=float(np.mean([math.hypot(r['vr'],r['vf'])>config['settle_speed'] for r in hold])),
                         loaded_voltage_median=None if not loaded else float(np.median(loaded))))
    return dict(bag=data['bag'], t0=data['t0'], boot_to_bag_offset=data['boot_to_bag_offset'],
                config=config, legs=legs, decode_errors=data['decode_errors'])


def plot(data, summary, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, layout='constrained')
    d = [r for r in data['debug'] if 7 <= r['time'] <= 49]
    lp = [r for r in data['local'] if 7 <= r['time'] <= 49]
    t = [r['time'] for r in lp]
    axes[0].plot(t, [r['right'] for r in lp], label='Estimated lateral position')
    axes[0].plot([r['time'] for r in d], [r['offset'] for r in d], label='Reference')
    axes[0].set_ylabel('Rightward position (m)')
    axes[1].plot(t, [r['vr'] for r in lp], label='Estimated lateral velocity')
    axes[1].plot([r['time'] for r in d], [r['velocity'] for r in d], label='Reference')
    axes[1].set_ylabel('Rightward velocity (m/s)')
    axes[2].plot(t, [r['forward'] for r in lp], label='Forward position error (m)')
    axes[2].plot(t, [r['vf'] for r in lp], label='Forward velocity (m/s)')
    axes[2].set_ylabel('Uncommanded forward axis')
    axes[3].plot(t, [r['yaw_error'] for r in lp], label='Yaw error (deg)')
    axes[3].set_ylabel('Heading error (deg)')
    for ax in axes:
        for leg in summary['legs']:
            ax.axvspan(leg['brake_s'], leg['reference_stop_s'], color='orange', alpha=.15)
        ax.axhline(0, color='gray', lw=.5)
        ax.grid(alpha=.2)
        ax.legend(loc='upper left', fontsize=8)
    axes[-1].set_xlabel('Seconds since earliest selected bag receipt; orange = reference braking')
    axes[-1].set_xlim(7, 49)
    fig.suptitle('Fixed-heading lateral stops: right / return / left / return')
    fig.savefig(path, dpi=150)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--bag', type=Path)
    group.add_argument('--data', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--plot', action='store_true')
    args = p.parse_args()
    data = extract(args.bag) if args.bag else json.loads(args.data.read_text())
    summary = summarize(data)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix('.data.json').write_text(json.dumps(data)+'\n')
    args.out.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
    if args.plot:
        plot(data, summary, args.out.with_suffix('.png'))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
