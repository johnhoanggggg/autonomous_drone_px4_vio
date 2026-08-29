#pragma once

/// ROS-free route-follower state shared by the C++ follower node and parity tests.
///
/// This is the direct counterpart of path_follower.py's CorrectionReplanGate
/// and PositionRouteFollower. Keeping the state machine outside rclcpp makes
/// every safety latch and command update independently testable.

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "px4_vio_bridge/path_geometry.hpp"

namespace px4_vio_bridge
{

using Correction4 = std::array<double, 4>;

struct FollowResult
{
  std::string status;
  bool valid{};
  Point2 desired_carrot{};
  Point2 commanded_carrot{};
  Point2 commanded_displacement{};
  double path_progress{};
  double progress{};
  double remaining{};
  double cross_track{};
  std::int32_t generation{};
};

bool requested_goal_reached(const std::string & follower_status, bool goal_terminal);

class CorrectionReplanGate
{
public:
  CorrectionReplanGate(
    double translation_trigger = 0.05,
    double yaw_trigger = 0.026179938779914945,
    double filter_time_constant = 0.35,
    double material_translation = 0.03,
    double material_yaw = 0.013089969389957472,
    double quiet_time = 0.40,
    double cooldown = 8.0);

  bool observe(const Correction4 & correction, double now);
  void path_received(double now);
  bool waiting(double now);

  [[nodiscard]] bool pending() const {return pending_;}
  [[nodiscard]] const Point2 & last_trigger_delta() const {return last_trigger_delta_;}

private:
  double translation_trigger_;
  double yaw_trigger_;
  double filter_time_constant_;
  double material_translation_;
  double material_yaw_;
  double quiet_time_;
  double cooldown_;
  std::optional<Correction4> filtered_;
  std::optional<Correction4> baseline_;
  std::optional<Correction4> event_reference_;
  std::optional<double> last_observation_;
  double last_material_change_{};
  bool path_after_change_{false};
  bool pending_{false};
  double cooldown_until_{};
  Point2 last_trigger_delta_{};
};

class PositionRouteFollower
{
public:
  using CommandValidator = std::function<bool(const Point2 &)>;

  PositionRouteFollower(
    double lookahead = 0.60,
    double max_carrot_speed = 0.10,
    double max_carrot_acceleration = 0.30,
    double max_cross_track = 0.60,
    double cross_track_resume = 0.05,
    double cross_track_recovery_time = 1.0,
    double arrival_tolerance = 0.12,
    double arrival_release_tolerance = 0.20);

  void clear_path();
  void reset_route_progress();
  void interrupt_cross_track_recovery();
  void hold_command();
  bool set_path(const std::vector<Point2> & points, const Point2 & pose);
  FollowResult update(
    const Point2 & pose, double dt,
    std::optional<double> lookahead = std::nullopt,
    const CommandValidator & command_validator = {});

  [[nodiscard]] const Polyline * path() const {return path_.get();}
  [[nodiscard]] double lookahead() const {return lookahead_;}
  [[nodiscard]] double progress() const {return progress_;}
  [[nodiscard]] double path_progress() const {return path_progress_;}
  [[nodiscard]] const Point2 & commanded_displacement() const {return commanded_displacement_;}
  [[nodiscard]] const Point2 & command_velocity() const {return command_velocity_;}
  [[nodiscard]] std::int32_t generation() const {return generation_;}
  [[nodiscard]] bool cross_track_latched() const {return cross_track_latched_;}
  [[nodiscard]] bool at_goal() const {return at_goal_;}

private:
  double lookahead_;
  double max_carrot_speed_;
  double max_carrot_acceleration_;
  double max_cross_track_;
  double cross_track_resume_;
  double cross_track_recovery_time_;
  double arrival_tolerance_;
  double arrival_release_tolerance_;
  std::unique_ptr<Polyline> path_;
  PathFingerprint fingerprint_;
  std::int32_t generation_{};
  double progress_{};
  double path_progress_{};
  Point2 commanded_displacement_{};
  Point2 command_velocity_{};
  bool cross_track_latched_{false};
  std::int32_t cross_track_fault_generation_{};
  double cross_track_recovery_elapsed_{};
  bool at_goal_{false};
};

}  // namespace px4_vio_bridge
