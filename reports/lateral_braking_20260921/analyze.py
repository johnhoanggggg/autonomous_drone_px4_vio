"""Read-only audit of the two latest fixed-heading stop tests.

The candidate damping trace is a recorded-input calculation, not a simulation
of the vehicle response. All output stays in this report directory.
"""
import json
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
OLD = ROOT / 'reports/lateral_stop_20260917'
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/px4_vio_bridge'))
from px4_vio_bridge.brake_damping import BrakeDamping
from analyze_lateral_stop import summarize
from analyze_control_response import analyse, recent_indices
from analyze_attitude_response import analyse as attitude_analyse

results = []
fig, axes = plt.subplots(4, 2, figsize=(14, 11), layout='constrained')
for column, (flight, logname) in enumerate([
        ('080456', '23_10_12'), ('080556', '23_11_12')]):
    bag = json.loads((OLD / f'{flight}.data.json').read_text())
    # LAND keeps the last leg/phase labels in debug, but changes the command.
    # Exclude it explicitly rather than treating landing as stationary hold.
    summary = summarize(dict(bag, debug=[r for r in bag['debug']
                        if r['state'] in ('LATERAL_MOVE', 'LATERAL_HOLD')]))
    path = ROOT / f'ros2_ws/flight_logs/offboard_lateral_stop_20260917T{flight}Z/px4_{logname}.ulg'
    ulog = ULog(str(path))
    ds = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
    lp, tr = ds['vehicle_local_position'], ds['trajectory_setpoint']
    by_stamp = {round(r['boot'] * 1e6): r for r in bag['local']}
    matched = [(i, by_stamp[int(ts)]) for i, ts in enumerate(lp['timestamp'])
               if int(ts) in by_stamp]
    assert len(matched) > 1000
    max_error = max(abs(float(lp['x'][i]) - r['x']) for i, r in matched)
    assert max_error == 0
    offset = float(np.median([r['time'] - lp['timestamp'][i] / 1e6 for i, r in matched]))
    time = lp['timestamp'] / 1e6 + offset
    tt = tr['timestamp'] / 1e6 + offset
    yaw = bag['debug'][0]['yaw0']
    lateral = np.array([-np.sin(yaw), np.cos(yaw)])
    position = np.column_stack([lp['x'], lp['y']])
    velocity = np.column_stack([lp['vx'], lp['vy']])
    origin = np.array(bag['debug'][0]['origin'])
    right = (position - origin) @ lateral
    vr = velocity @ lateral
    ref_p = np.column_stack([tr['position[0]'], tr['position[1]']])
    ref_v = np.column_stack([tr['velocity[0]'], tr['velocity[1]']]).astype(float)
    ref_a = np.column_stack([tr['acceleration[0]'], tr['acceleration[1]']]).astype(float)
    lo = summary['legs'][0]['start_s']
    hi = summary['legs'][-1]['hold_end_s']
    control, control_summary = analyse(path, offset, lo, hi)
    ct = control['time_s']
    demand_v = np.column_stack([control['vx_demand'], control['vy_demand']]) @ lateral
    demand_a = np.column_stack([control['ax_demand'], control['ay_demand']]) @ lateral
    tilt_a = np.column_stack([control['ax_tilt_equivalent'], control['ay_tilt_equivalent']]) @ lateral
    result = dict(flight=flight, offset=offset, exact_matches=len(matched),
                  max_matching_x_error=max_error, legs=[], selected_windows=[],
                  parameters={k: ulog.initial_parameters[k] for k in [
                      'EKF2_EV_DELAY', 'SENS_BOARD_X_OFF', 'SENS_BOARD_Y_OFF',
                      'MPC_XY_P', 'MPC_XY_VEL_P_ACC', 'MPC_XY_VEL_I_ACC',
                      'MPC_XY_VEL_D_ACC']}, parameter_changes=ulog.changed_parameters)

    # Candidate: modest added velocity-error damping in BRAKE/HOLD only.
    # First-order smoothing includes ramp-out to zero; unchanged p/v references.
    index, fresh = recent_indices(lp['timestamp'], tr['timestamp'], 100_000)
    phases = np.full(len(tt), 'OTHER', dtype='U8')
    for leg in summary['legs']:
        phases[(tt >= leg['brake_s']) & (tt < leg['reference_stop_s'])] = 'BRAKE'
        phases[(tt >= leg['reference_stop_s']) & (tt <= leg['hold_end_s'])] = 'HOLD'
    enabled = np.isin(phases, ['BRAKE', 'HOLD']) & fresh
    enabled &= lp['v_xy_valid'][index].astype(bool)
    enabled &= np.isfinite(ref_v).all(axis=1) & np.isfinite(ref_a).all(axis=1)
    addition = np.zeros_like(ref_v)
    damper = BrakeDamping(enabled=True)
    candidate = np.zeros_like(ref_a)
    for i in range(len(tt)):
        measured = velocity[index[i]] if fresh[i] and lp['v_xy_valid'][index[i]] else None
        damping_result = damper.update(ref_v[i], ref_a[i], measured, float(tt[i]),
                                       active=lo <= tt[i] <= hi, braking=bool(enabled[i]))
        addition[i] = damping_result.filtered_correction
        candidate[i] = damping_result.acceleration
    assert np.max(np.linalg.norm(addition, axis=1)) <= .1 + 1e-9
    assert np.max(np.linalg.norm(candidate, axis=1)) <= .3 + 1e-9
    result['candidate'] = dict(gain_per_s=.5, correction_limit_m_s2=.1,
                              smoothing_s=.1, total_acceleration_limit_m_s2=.3,
                              note='Recorded-input shadow only; no prediction of improved flight.')
    for leg in summary['legs']:
        start, brake, stop, end = [leg[k] for k in [
            'start_s', 'brake_s', 'reference_stop_s', 'hold_end_s']]
        sign = 1 if leg['direction'] == 'right' else -1
        va, vb, vc = sign * np.interp([brake, stop, stop+1.], time, vr)
        hold = (time >= stop) & (time <= end)
        wire = (tt >= stop+.1) & (tt <= end-.1)
        assert np.max(np.ptp(ref_p[wire], axis=0)) == 0
        assert np.max(abs(ref_v[wire])) == 0
        assert np.max(abs(ref_a[wire])) == 0
        row = dict(leg=leg['leg']+1, direction=leg['direction'],
                   brake_start_s=brake, reference_stop_s=stop,
                   estimated_speed_at_brake_m_s=float(va),
                   estimated_speed_at_reference_stop_m_s=float(vb),
                   estimated_speed_one_second_later_m_s=float(vc),
                   average_braking_deceleration_m_s2=float((va-vb)/(stop-brake)),
                   first_endpoint_overshoot_m=leg['first_stop_endpoint_overshoot_m'],
                   later_endpoint_overshoot_m=leg['endpoint_overshoot_m'],
                   hold_lateral_span_m=float(np.ptp(right[hold])),
                   hold_duration_s=end-stop,
                   verified_constant_hold_position=True,
                   verified_zero_hold_velocity_and_acceleration=True,
                   candidate_hold_correction_p95_m_s2=float(np.quantile(np.linalg.norm(addition[wire],axis=1),.95)))
        result['legs'].append(row)
        for ax in axes[:, column]:
            ax.axvspan(brake, stop, color='orange', alpha=.15)

    windows = [(29.9, 30.9), (31.2, 31.7)] if flight == '080456' else [(44.2, 44.7)]
    attitude, attitude_summary = attitude_analyse(path, offset,
        [(str(a), a, b) for a,b in windows])
    result['attitude_windows'] = attitude_summary
    for a,b in windows:
        m = (ct >= a) & (ct < b)
        q = (tt >= a) & (tt < b)
        vactual = np.interp(ct[m],time,vr)
        pterm = ulog.initial_parameters['MPC_XY_VEL_P_ACC'] * (demand_v[m]-vactual)
        # Residual estimate, not a directly logged velocity integrator. Show
        # sensitivity to three derivative windows; no motor/trim diagnosis.
        residual=[]
        for half in [.025, .05, .1]:
            derivative=(np.interp(ct[m]+half,time,vr)-np.interp(ct[m]-half,time,vr))/(2*half)
            residual.append(float(np.mean(demand_a[m]-pterm+
                ulog.initial_parameters['MPC_XY_VEL_D_ACC']*derivative)))
        result['selected_windows'].append(dict(
            start_s=a, end_s=b,
            estimated_velocity_m_s=float(np.mean(np.interp(ct[m],time,vr))),
            internal_velocity_demand_m_s=float(np.mean(demand_v[m])),
            internal_acceleration_demand_m_s2=float(np.mean(demand_a[m])),
            tilt_equivalent_acceleration_m_s2=float(np.mean(tilt_a[m])),
            velocity_p_term_m_s2=float(np.mean(pterm)),
            inferred_residual_compensation_m_s2=residual,
            shadow_added_acceleration_m_s2=float(np.mean(addition[q] @ lateral))))

    axes[0,column].plot(time,right,label='Estimated position')
    axes[0,column].plot(tt,(ref_p-origin)@lateral,label='Reference')
    axes[1,column].plot(time,vr,label='Estimated velocity')
    axes[1,column].plot(ct,demand_v,label='PX4 internal velocity demand')
    axes[1,column].plot(tt,ref_v@lateral,':',label='Adapter velocity reference')
    axes[2,column].plot(ct,demand_a,label='PX4 acceleration demand')
    axes[2,column].plot(ct,tilt_a,label='Tilt-equivalent acceleration (approx.)')
    axes[3,column].plot(tt,ref_a@lateral,label='Recorded acceleration feedforward')
    axes[3,column].plot(tt,candidate@lateral,label='Candidate with bounded damping')
    axes[0,column].set_title(f'September 17 {flight} — fixed initial-heading right axis')
    for row,label in enumerate(['Position (m)','Velocity (m/s)','Acceleration (m/s²)','Adapter acceleration (m/s²)']):
        ax=axes[row,column];ax.set_ylabel(label);ax.set_xlim(lo-.5,hi)
        ax.grid(alpha=.2);ax.axhline(0,color='gray',lw=.5);ax.legend(fontsize=7)
    axes[-1,column].set_xlabel('Bag-aligned seconds; orange = scheduled braking')
    results.append(result)
fig.suptitle('Lateral stop audit: fixed targets still oscillate. Candidate commands do not predict vehicle response.')
fig.savefig(OUT/'braking_and_damping.png',dpi=160)
(OUT/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
