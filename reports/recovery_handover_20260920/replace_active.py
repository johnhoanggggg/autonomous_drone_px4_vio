from pathlib import Path
p=Path('ros2_ws/src/px4_vio_bridge/src/offboard_global_planner_node.cpp')
s=p.read_text();a=s.index('    const auto candidate = initial_candidate ? initial_candidate : active_recovery_candidate();');b=s.index('\n    return true;\n  }\n\n  void begin_fault_stop()',a)
s=s[:a]+'''    // Try the route from the last PUBLISHED reference, before integrating an
    // active step. Advancing both controllers in one tick doubles the reference
    // step and makes the route acceleration/leash checks use the wrong origin.
    const auto limits = route_clearance_speed_limits();
    const auto measured_block = measured_route_governor_rejection(false);
    bool joining = false;
    std::string handover;
    if (!limits || limits->command_cap < route_reserve_rejoin_min_speed_ || measured_block) {
      active_recovery_->clear_since.reset();
      handover = measured_block.value_or(format("route_cap=%.3f/%.3fm/s",
          limits ? limits->command_cap : -1.0, route_reserve_rejoin_min_speed_));
    } else {
      if (!active_recovery_->clear_since) {active_recovery_->clear_since = now;}
      if (now - *active_recovery_->clear_since >= route_reserve_rejoin_dwell_) {
        joining = true;
        const auto result = update_route_command(std::min(active_recovery_speed_, route_rejoin_speed_));
        if (!result.first && !result.second && !reserve_brake_active_) {
          active_recovery_.reset();
          publish_setpoint(hover_height_, route_yaw());
          publish_flight_telemetry();
          publish_route_status(route_status_text(), route_status_kind());
          return true;
        }
        handover = result.first ? result.first->reason :
          result.second.value_or("route recovery still latched");
        // Retain the completed clearance dwell while inputs remain admissible.
        // Ramp incompatible active velocity down and retry each tick from the
        // reference actually sent. This does not require measured capture.
      } else {
        handover = format("headroom_dwell=%.2f/%.2fs",
          now - *active_recovery_->clear_since, route_reserve_rejoin_dwell_);
      }
    }

    const double acceleration = limiter_->max_acceleration();
    const double time_left = std::max(0.0, active_recovery_time_ - elapsed - dt_);
    const double travel_left = std::max(0.0, active_recovery_travel_ - active_recovery_->travel);
    // These bounds taper the command before the hard measured episode limits.
    // They do not predict physical travel: the measured limits stay enforced
    // above, and all prospective stopping endpoints remain checked below.
    const double budget_speed = std::min({active_recovery_speed_, limiter_->max_speed(),
        acceleration * time_left, std::sqrt(2.0 * acceleration * travel_left)});
    Point2 desired{0., 0.};
    if (!joining) {
      const auto candidate = initial_candidate ? initial_candidate :
        active_recovery_candidate(budget_speed);
      if (!candidate) {
        stop_active_recovery("no known safe inward/forward transition");
        return state_ != State::Route;
      }
      desired = vio_enu_displacement_to_ned(
        map_displacement_to_vio(candidate->velocity, (*correction_)[3]));
      const double speed = std::hypot(desired.first, desired.second);
      if (speed > budget_speed) {
        desired.first *= budget_speed / speed;
        desired.second *= budget_speed / speed;
      }
    }
    const auto previous_velocity = limiter_->velocity();
    const auto previous = *limiter_->position();
    const double step = acceleration * dt_;
    // Leave one route tick of room inside the public lead limit. A join starts
    // from this command, and must pass its own exact lead/sweep validation.
    const double steering_lead = std::max(0.0,
      active_recovery_lead_ - std::min(active_recovery_speed_, limiter_->max_speed()) * dt_ - 1.0e-6);
    std::optional<std::pair<Point2, Point2>> accepted;
    std::optional<std::string> limited_by;
    std::optional<std::string> last_rejection;
    // The selector checks direction/lookahead. Evaluate actual ramped, leashed
    // commands here at decreasing desired speeds against the COMPLETE envelope,
    // including the combined stopping endpoint and recovery origin radius.
    for (int scale = 8; scale >= 0; --scale) {
      const double fraction = static_cast<double>(scale) / 8.0;
      Point2 change{fraction * desired.first - previous_velocity.first,
        fraction * desired.second - previous_velocity.second};
      const double magnitude = std::hypot(change.first, change.second);
      if (magnitude > step) {
        change.first *= step / magnitude;
        change.second *= step / magnitude;
      }
      const Point2 velocity{previous_velocity.first + change.first,
        previous_velocity.second + change.second};
      const Point2 target = clamp_to_disc(
        {previous.first + velocity.first * dt_, previous.second + velocity.second * dt_},
        body, steering_lead);
      if (const auto rejection = active_command_rejection(target, velocity)) {
        if (!limited_by) {limited_by = rejection;}
        last_rejection = rejection;
        if (joining) {break;}  // desired is already zero; other scales are identical.
      } else {
        accepted = std::make_pair(target, velocity);
        break;
      }
    }
    if (!accepted) {
      stop_active_recovery("no admissible ramped command: " +
        last_rejection.value_or("unknown rejection"));
      return state_ != State::Route;
    }
    const auto & [target, velocity] = *accepted;
    limiter_->adopt(target, velocity);
    clear_planner_fault();
    clear_route_command_stall();
    route_command_holding_ = false;
    degraded_ride_out_ = false;
    goal_since_.reset();
    publish_setpoint(hover_height_, route_yaw());
    publish_flight_telemetry();
    publish_route_status(format(
        "ACTIVE_RECOVERY speed=%.3fm/s lead=%.3f/%.3fm elapsed=%.2f/%.2fs travel=%.3f/%.3fm "
        "budget_speed=%.3fm/s joining=%d handover=[%s] limited_by=[%s]",
        std::hypot(velocity.first, velocity.second), distance(target, body), active_recovery_lead_,
        elapsed, active_recovery_time_, active_recovery_->travel, active_recovery_travel_,
        budget_speed, joining, handover.c_str(), limited_by.value_or("none").c_str()),
      "ACTIVE_RECOVERY");''' +s[b:]
p.write_text(s)
