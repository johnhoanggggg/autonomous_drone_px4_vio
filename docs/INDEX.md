# Docs index

Everything here was written during the 2026 project and is kept as it was. Most of it
predates the October work, so where a note and the top-level [README](../README.md)
disagree, trust the README. Dates are when each note was written.

## Reference

- [fyp_paper_2026.pdf](fyp_paper_2026.pdf): the 2026 FYP paper (v5, 10-19); the best single overview of the design and results.
- [README_full_20260921.md](README_full_20260921.md): the previous 2,300-line README: launch reference, flight parameters, measured facts, known issues (current to 09-21).
- [CODEX_WORKFLOW.md](CODEX_WORKFLOW.md): how coding-agent sessions were run on this repo.
- [NEXT_SESSION.md](NEXT_SESSION.md): the to-do list as it stood on 09-21 (2D tunnel work); superseded.

## Handoffs (design notes per subsystem)

- [HANDOFF.md](HANDOFF.md): the original whole-system handoff: bring-up, PX4 bridge, flight nodes.
- [HANDOFF_ARCHIVE.md](HANDOFF_ARCHIVE.md): chronological log of the July bring-up.
- [HANDOFF_ROS_MIGRATION.md](HANDOFF_ROS_MIGRATION.md): moving from on-device DepthAI VIO to ROS-native RTAB-Map, with measurements.
- [HANDOFF_GLOBAL_PLANNER.md](HANDOFF_GLOBAL_PLANNER.md): the 2D A* global planner and route follower.
- [HANDOFF_PLANNER_STABILITY.md](HANDOFF_PLANNER_STABILITY.md): planner/follower stability changes.
- [HANDOFF_FOLLOWER_FOLLOWUP.md](HANDOFF_FOLLOWER_FOLLOWUP.md): follow-up on the same.
- [HANDOFF_3D_NAVIGATION.md](HANDOFF_3D_NAVIGATION.md): the 3D OctoMap planner (observation mode).
- [HANDOFF_3D_FLIGHT_ADAPTER.md](HANDOFF_3D_FLIGHT_ADAPTER.md): the unknown-aware 3D flight adapter that the 3D flights use.
- [HANDOFF_TRAJECTORY_SMOOTHNESS.md](HANDOFF_TRAJECTORY_SMOOTHNESS.md): trajectory smoothness and wall strikes.
- [HANDOFF_YAW_TRACKING.md](HANDOFF_YAW_TRACKING.md): heading versus direction of travel.
- [HANDOFF_LOOP_CLOSURE.md](HANDOFF_LOOP_CLOSURE.md): native loop-closure correction (parked).
- [HANDOFF_SOL_BRAKING_PID_20260921.md](HANDOFF_SOL_BRAKING_PID_20260921.md): roll-rate braking trial session.
- [HANDOFF_SOL_MIGRATION_TUNNEL.md](HANDOFF_SOL_MIGRATION_TUNNEL.md): tunnel smoothing on the migrated stack.

## 3D planner notes (September)

- [PLANNER3D_START_PERF_20260915.md](PLANNER3D_START_PERF_20260915.md): clearing the start area and planner performance.
- [PLANNER3D_LEVEL_CAMERA_20260915.md](PLANNER3D_LEVEL_CAMERA_20260915.md): route planning with a level camera.
- [PLANNER3D_SCAN_HANDOFF_20260915.md](PLANNER3D_SCAN_HANDOFF_20260915.md): guided coverage test and goal height.
- [PLANNER3D_LATEST_BAG_20260915.md](PLANNER3D_LATEST_BAG_20260915.md): waypoint test of 09-15 06:34.
- [CARRY_3D_ASSESSMENT_20260915.md](CARRY_3D_ASSESSMENT_20260915.md): three hand-carried 3D runs: clearance rejection and map lag.
- [WAYPOINT_START_DIAGNOSIS_20260915.md](WAYPOINT_START_DIAGNOSIS_20260915.md): why waypoints were rejected at the start.
- [VOXEL_BRANCH_COMPARISON_20260915.md](VOXEL_BRANCH_COMPARISON_20260915.md): ROS migration versus the `voxel-perception-tuning` branch.

## Control and tuning results

- [WALL_STRIKES_20260907.md](WALL_STRIKES_20260907.md): measured control response at wall strikes, first mitigation.
- [PLANNER_ZONE_20260907_0952.md](PLANNER_ZONE_20260907_0952.md): wall-strike zone, six runs on healthy packs.
- [HOVER_TURN_20260907.md](HOVER_TURN_20260907.md): hover-turn diagnosis.
- [HOVER_TURN_BATTERY_COMPARE_20260907.md](HOVER_TURN_BATTERY_COMPARE_20260907.md): hover turn, full versus low battery.
- [YAW_MOTOR_DIAGNOSIS_20260908.md](YAW_MOTOR_DIAGNOSIS_20260908.md): growing yaw correction: software checked, physical cause open.
- [ROLL_RATE_TUNING_20260908.md](ROLL_RATE_TUNING_20260908.md): roll-rate P trial plan.
- [ROLL_RATE_TRIAL_RESULTS_20260908.md](ROLL_RATE_TRIAL_RESULTS_20260908.md): roll-P trial: mixed, final hold still unstable.
- [ROLL_RATE_TRIAL_RESULTS_20260921.md](ROLL_RATE_TRIAL_RESULTS_20260921.md): roll-rate P lateral-stop results.
- [LATERAL_STOP_TEST_20260908.md](LATERAL_STOP_TEST_20260908.md): the fixed-heading lateral braking test.
- [LATERAL_STOP_RESULTS_20260908.md](LATERAL_STOP_RESULTS_20260908.md): braking overshoot and stationary-target oscillation.
- [TUNNEL_BRAKING_20260908.md](TUNNEL_BRAKING_20260908.md): braking response and a degraded-command discontinuity.
- [LATERAL_BRAKING_REVIEW_20260921.md](LATERAL_BRAKING_REVIEW_20260921.md): lateral stops: braking and renewed motion during hold.
- [LATERAL_DAMPING_TRIAL_20260921.md](LATERAL_DAMPING_TRIAL_20260921.md): velocity damping trial plan.
- [LATERAL_DAMPING_RESULTS_20260921.md](LATERAL_DAMPING_RESULTS_20260921.md): first damping comparison (ON/OFF/ON).
- [LATERAL_DAMPING_GAIN075_RESULTS_20260921.md](LATERAL_DAMPING_GAIN075_RESULTS_20260921.md): damping gain 0.75 results.
- [LATERAL_DAMPING_GAIN075_REPEAT_RESULTS_20260921.md](LATERAL_DAMPING_GAIN075_REPEAT_RESULTS_20260921.md): gain 0.75 repeat.
- [MPC_XY_P075_RESULTS_20260921.md](MPC_XY_P075_RESULTS_20260921.md): `MPC_XY_P` 0.75 trial.
- [MPC_XY_VEL_I030_RESULTS_20260921.md](MPC_XY_VEL_I030_RESULTS_20260921.md): `MPC_XY_VEL_I_ACC` 0.30 trial.
- [GLOBAL_DAMPING_OPT_IN_20260921.md](GLOBAL_DAMPING_OPT_IN_20260921.md): damping as an opt-in for global flights.

## 2D route following and heading (early September)

- [FLIGHT_DIAGNOSIS_20260906.md](FLIGHT_DIAGNOSIS_20260906.md): two flights with the constrained-command heading.
- [CORNER_WEAVE_20260906.md](CORNER_WEAVE_20260906.md): corner weave and the join lateral cap.
- [HEADING_CONTINUITY_20260908.md](HEADING_CONTINUITY_20260908.md): heading reversal diagnosis and fix.
- [HEADING_FLIGHTS_20260908.md](HEADING_FLIGHTS_20260908.md): three flights after the heading fix.
- [EV_DELAY_TRIAL_20260913.md](EV_DELAY_TRIAL_20260913.md): VIO timing correction (`EKF2_EV_DELAY`) trial.
- [CLUTTER_TRACKING_FIX_20260913.md](CLUTTER_TRACKING_FIX_20260913.md): tracking changes for cluttered routes.
- [CLUTTER_TRACKING_FOLLOWUP_20260913.md](CLUTTER_TRACKING_FOLLOWUP_20260913.md): follow-up after two tunnel flights.
- [NON_BRAKING_FIX_20260916.md](NON_BRAKING_FIX_20260916.md): correction and route-frame consistency.
