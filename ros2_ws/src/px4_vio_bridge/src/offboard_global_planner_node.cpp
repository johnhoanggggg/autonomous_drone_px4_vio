/// Position-only PX4 adapter for the validated global-planner follower.
///
/// C++ counterpart of offboard_global_planner.py. It rebases the follower's
/// continuous-VIO-frame displacement from PX4's current local position,
/// advances the final command along the accepted map-frame polyline, and
/// validates that exact output against the raw occupancy map before publishing
/// a NED position setpoint. Invalid planner data latches a stationary HOLD;
/// persistent faults request AUTO.LAND.

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "offboard_hover_node.hpp"
#include "px4_vio_bridge/command_limiter.hpp"
#include "px4_vio_bridge/grid_clearance.hpp"
#include "px4_vio_bridge/route_follower.hpp"

namespace px4_vio_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr float kNanF = std::numeric_limits<float>::quiet_NaN();
const double kNan = std::numeric_limits<double>::quiet_NaN();

double degrees(double radians) {return radians * 180.0 / kPi;}
double radians_of(double degrees_value) {return degrees_value * kPi / 180.0;}

std::string format(const char * fmt, ...) __attribute__((format(printf, 1, 2)));
std::string format(const char * fmt, ...)
{
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return std::string(buffer);
}
}  // namespace

class OffboardGlobalPlanner final : public OffboardHover
{
public:
  OffboardGlobalPlanner()
  : OffboardHover("cpp_flight_adapter")
  {
    follower_displacement_topic_ = declare_parameter<std::string>(
      "follower_displacement_topic", "/planner/follower/vio_displacement");
    follower_valid_topic_ =
      declare_parameter<std::string>("follower_valid_topic", "/planner/follower/valid");
    follower_goal_topic_ =
      declare_parameter<std::string>("follower_goal_topic", "/planner/follower/goal_reached");
    const auto correction_topic =
      declare_parameter<std::string>("correction_topic", "/rtabmap/odom_correction");
    path_topic_ = declare_parameter<std::string>("path_topic", "/planner/path");
    map_topic_ = declare_parameter<std::string>("map_topic", "/rtabmap/grid");
    map_pose_topic_ = declare_parameter<std::string>("map_pose_topic", "/rtabmap/body_pose");
    follower_config_topic_ =
      declare_parameter<std::string>("follower_config_topic", "/planner/follower/config");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    follower_timeout_ = declare_parameter<double>("follower_timeout", 0.30);
    correction_timeout_ = declare_parameter<double>("correction_timeout", 1.0);
    path_timeout_ = declare_parameter<double>("path_timeout", 3.0);
    map_timeout_ = declare_parameter<double>("map_timeout", 3.0);
    map_pose_timeout_ = declare_parameter<double>("map_pose_timeout", 1.0);
    planner_fault_land_time_ = declare_parameter<double>("planner_fault_land_time", 3.0);
    // Must match the follower's value: the two nodes have to agree on what
    // counts as an escape, or one proposes a chord the other vetoes.
    escape_minimum_improvement_ =
      declare_parameter<double>("escape_minimum_improvement", 0.01);
    goal_hold_time_ = declare_parameter<double>("goal_hold_time", 3.0);
    max_follower_displacement_ =
      declare_parameter<double>("max_follower_displacement", 1.0);
    max_correction_m_ = declare_parameter<double>("max_correction_m", 0.25);
    max_correction_yaw_ =
      radians_of(declare_parameter<double>("max_correction_yaw_deg", 5.0));
    const auto command_speed = declare_parameter<double>("command_speed", 0.10);
    const auto command_acceleration =
      declare_parameter<double>("command_acceleration", 0.30);
    horizontal_feedforward_ = declare_parameter<bool>("horizontal_feedforward", true);
    const auto corner_blending = declare_parameter<bool>("corner_blending", false);
    const auto junction_deviation = declare_parameter<double>("junction_deviation", 0.05);
    const auto projection_tolerance =
      declare_parameter<double>("path_command_projection_tolerance", 0.05);
    const auto entry_tolerance =
      declare_parameter<double>("path_command_entry_tolerance", 0.30);
    const auto connector_tolerance =
      declare_parameter<double>("path_command_connector_tolerance", 0.20);
    const auto suffix_tolerance =
      declare_parameter<double>("path_command_suffix_tolerance", 0.01);
    const auto corner_tolerance = declare_parameter<double>("path_corner_tolerance", 0.05);
    route_command_grace_ = declare_parameter<double>("route_command_grace", 2.0);
    replan_during_yaw_align_ = declare_parameter<bool>("replan_during_yaw_align", false);
    geofence_radius_ = declare_parameter<double>("geofence_radius", 1.0);
    geofence_tolerance_ = declare_parameter<double>("geofence_tolerance", 0.15);
    transit_horizontal_error_ = declare_parameter<double>("transit_horizontal_error", 0.60);
    pre_route_max_horizontal_error_ =
      declare_parameter<double>("pre_route_max_horizontal_error", 0.15);
    yaw_track_ = declare_parameter<bool>("yaw_follows_heading", true);
    yaw_track_min_displacement_ =
      declare_parameter<double>("yaw_track_min_displacement", 0.15);
    yaw_track_deadband_ =
      radians_of(declare_parameter<double>("yaw_track_deadband_deg", 15.0));
    yaw_align_error_ = radians_of(declare_parameter<double>("yaw_align_error_deg", 40.0));
    yaw_resume_error_ = radians_of(declare_parameter<double>("yaw_resume_error_deg", 15.0));
    if (yaw_resume_error_ > yaw_align_error_) {
      yaw_resume_error_ = yaw_align_error_;
    }

    limiter_ = std::make_unique<HorizontalCommandLimiter>(
      command_speed, command_acceleration);
    route_limiter_ = std::make_unique<PathCommandLimiter>(
      limiter_->max_speed(), limiter_->max_acceleration(), projection_tolerance,
      corner_tolerance, entry_tolerance, connector_tolerance, suffix_tolerance,
      corner_blending, junction_deviation);

    displacement_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      follower_displacement_topic_, 10,
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
        on_follower_displacement(*msg);
      });
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      follower_valid_topic_, 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        follower_valid_ = msg->data;
        follower_valid_received_ = monotonic_time();
      });
    goal_sub_ = create_subscription<std_msgs::msg::Bool>(
      follower_goal_topic_, 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        goal_reached_ = msg->data;
        goal_received_ = monotonic_time();
      });
    correction_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      correction_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_correction(*msg);});
    const auto map_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {on_map(*msg);});
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {on_path(*msg);});
    map_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      map_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_map_pose(*msg);});
    config_sub_ = create_subscription<std_msgs::msg::String>(
      follower_config_topic_, map_qos,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {on_follower_config(*msg);});
    route_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/planner/flight/status", 10);

    RCLCPP_WARN(
      get_logger(),
      "GLOBAL PLANNER FLIGHT ADAPTER (C++): position control, horizontal "
      "velocity/acceleration feedforward=%d, auto_arm=%d, speed=%.2fm/s, "
      "geofence=%.2fm, correction gate=%.2fm/%.1fdeg",
      static_cast<int>(horizontal_feedforward_), static_cast<int>(auto_arm_),
      limiter_->max_speed(), geofence_radius_,
      max_correction_m_, degrees(max_correction_yaw_));
    RCLCPP_WARN(
      get_logger(),
      "BATTERY AUTHORITY: PX4 arming checks and battery failsafes; "
      "companion battery topics are telemetry only");
    if (!yaw_track_) {
      RCLCPP_WARN(get_logger(), "yaw tracking disabled; holding latched takeoff yaw");
    } else if (yaw_align_error_ > 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "YAW FOLLOWS PATH HEADING: slew %.0fdeg/s, translation pauses above "
        "%.0fdeg error and resumes below %.0fdeg",
        degrees(commanded_yaw_rate_), degrees(yaw_align_error_),
        degrees(yaw_resume_error_));
    } else {
      RCLCPP_WARN(
        get_logger(),
        "YAW FOLLOWS PATH HEADING: slew %.0fdeg/s, turning while translating "
        "(align gate disabled)", degrees(commanded_yaw_rate_));
    }

    setup_keyboard_controls();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), [this]() {tick();});
  }

protected:
  Point2 hold_point() const override
  {
    if (limiter_ && limiter_->position()) {
      return *limiter_->position();
    }
    return {x0_.value_or(0.0), y0_.value_or(0.0)};
  }

  double horizontal_error_limit() const override
  {
    const bool moving = limiter_ &&
      std::hypot(limiter_->velocity().first, limiter_->velocity().second) > 0.01;
    return moving ? transit_horizontal_error_ : max_horizontal_error_;
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition & msg) override
  {
    const std::array<int, 3> counters{
      static_cast<int>(msg.xy_reset_counter),
      static_cast<int>(msg.z_reset_counter),
      static_cast<int>(msg.heading_reset_counter)};
    if (local_reset_counters_ && counters != *local_reset_counters_) {
      const double now = monotonic_time();
      if (x0_ && std::isfinite(msg.x) && std::isfinite(msg.y)) {
        limiter_->reset({static_cast<double>(msg.x), static_cast<double>(msg.y)});
        route_limiter_->clear();
        holding_for_fault_ = true;
        require_follower_after_ = now;
        planner_fault_since_ = now;
        planner_fault_reason_text_ = format(
          "PX4 local reset counters (%d, %d, %d)->(%d, %d, %d)",
          (*local_reset_counters_)[0], (*local_reset_counters_)[1],
          (*local_reset_counters_)[2], counters[0], counters[1], counters[2]);
      }
      if (counters[2] != (*local_reset_counters_)[2] && std::isfinite(msg.heading)) {
        // The estimator's heading jumped; the tracked leg heading was expressed
        // in the old frame, so re-latch on the new one.
        yaw0_ = static_cast<double>(msg.heading);
        yaw_cmd_ = static_cast<double>(msg.heading);
        yaw_target_.reset();
        yaw_holding_ = false;
      }
      RCLCPP_ERROR(get_logger(), "%s", planner_fault_reason_text_.c_str());
    }
    local_reset_counters_ = counters;
    OffboardHover::on_local_position(msg);
  }

  void arm() override
  {
    if (const auto reason = preflight_reason()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "ARM INHIBITED: %s", reason->c_str());
      return;
    }
    OffboardHover::arm();
  }

  void publish_setpoint(double z_up, std::optional<double> yaw) override
  {
    ensure_limiter();
    if (!limiter_->position()) {
      return;
    }
    // Callers that do not name a yaw (the base class LAND and ground-hold
    // states) must hold the heading the route reached, NOT snap back to the
    // takeoff latch: reverting would command a large turn during a descent.
    const double target_yaw = yaw.value_or(route_yaw());
    const auto [yaw_sp, yawspeed] = ramp_yaw(target_yaw);
    const auto [z_sp, vz_sp] = ramp_z(z_up);
    const auto [vx_sp, vy_sp] = setpoint_velocity_xy();
    const auto [ax_sp, ay_sp] = setpoint_acceleration_xy(vx_sp, vy_sp);
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = now_us();
    msg.position = {
      static_cast<float>(limiter_->position()->first),
      static_cast<float>(limiter_->position()->second),
      static_cast<float>(-z_sp)};
    msg.velocity = {
      static_cast<float>(vx_sp), static_cast<float>(vy_sp), static_cast<float>(vz_sp)};
    msg.acceleration = {static_cast<float>(ax_sp), static_cast<float>(ay_sp), kNanF};
    msg.yaw = static_cast<float>(yaw_sp);
    msg.yawspeed = static_cast<float>(yawspeed);
    sp_pub_->publish(msg);
  }

  bool handle_flight_state() override
  {
    if (state_ != State::ClimbHold && state_ != State::Route) {
      return false;
    }

    ensure_limiter();
    if (!check_flight_position()) {
      return true;
    }
    if (geofence_breached()) {
      trigger_landing("vehicle crossed planner-flight geofence");
      return true;
    }

    auto fault = planner_health_reason();

    if (state_ == State::ClimbHold) {
      if (fault) {
        latch_fault_hold(*fault);
      } else {
        clear_planner_fault();
      }
      publish_setpoint(hover_height_, yaw0_);
      if (fault) {
        return true;
      }
      if (auto_arm_) {
        const bool altitude_ok = pos_ &&
          std::abs(-static_cast<double>(pos_->z) - hover_height_) <= reach_tol_;
        const bool horizontal_ok =
          horizontal_error_ && *horizontal_error_ <= pre_route_max_horizontal_error_;
        if (altitude_ok && horizontal_ok) {
          RCLCPP_WARN(get_logger(), "stable hover reached; enabling route command");
          set_state(State::Route);
        } else if (t_ > climb_timeout_) {
          trigger_landing("climb timeout without stable pre-route hover");
        }
      } else if (t_ > climb_timeout_) {
        RCLCPP_WARN(get_logger(), "dry run: enabling route command without arming");
        set_state(State::Route);
      }
      return true;
    }

    route_command_holding_ = false;
    if (!fault) {
      const auto [command_fault, deferral] = update_route_command();
      if (command_fault) {
        fault = command_fault;
      } else if (deferral) {
        fault = hold_route_command(*deferral);
      } else {
        clear_route_command_stall();
      }
    }
    if (fault) {
      latch_fault_hold(*fault);
      freeze_yaw_target();
      clear_route_command_stall();
    } else {
      clear_planner_fault();
    }
    publish_setpoint(hover_height_, route_yaw());

    if (goal_reached_ && !fault && !route_command_holding_) {
      if (!goal_since_) {
        goal_since_ = monotonic_time();
      }
      const double goal_elapsed = monotonic_time() - *goal_since_;
      publish_route_status(
        format("GOAL_REACHED holding %.1f/%.1fs", goal_elapsed, goal_hold_time_),
        "GOAL_REACHED");
      if (is_armed() && goal_elapsed >= goal_hold_time_) {
        trigger_landing("planner goal reached");
      }
    } else {
      goal_since_.reset();
      if (!fault && !route_command_holding_) {
        publish_route_status(route_status_text(), route_status_kind());
      }
    }
    return true;
  }

private:
  // --- subscriptions ------------------------------------------------------
  void on_follower_displacement(const geometry_msgs::msg::Vector3Stamped & msg)
  {
    const Point2 vector{msg.vector.x, msg.vector.y};
    if (msg.header.frame_id != "vio" || !finite(vector) ||
      std::hypot(vector.first, vector.second) > max_follower_displacement_)
    {
      vio_displacement_.reset();
    } else {
      vio_displacement_ = vector;
    }
    displacement_received_ = monotonic_time();
  }

  void on_follower_config(const std_msgs::msg::String & msg)
  {
    follower_config_received_ = monotonic_time();
    try {
      const auto config = nlohmann::json::parse(msg.data);
      if (config.at("frame_id").get<std::string>() != frame_id_) {
        throw std::invalid_argument(
                format(
                  "frame_id '%s' != '%s'",
                  config.at("frame_id").get<std::string>().c_str(), frame_id_.c_str()));
      }
      const auto radius = config.at("robot_radius").get<double>();
      const auto margin = config.at("safety_margin").get<double>();
      const auto threshold = config.at("occupied_threshold").get<int>();
      const auto speed = config.at("max_carrot_speed").get<double>();
      if (!std::isfinite(radius) || !std::isfinite(margin) || !std::isfinite(speed)) {
        throw std::invalid_argument("clearance and speed values must be finite");
      }
      if (radius < 0.0 || margin < 0.0 || radius + margin <= 0.0) {
        throw std::invalid_argument("robot_radius + safety_margin must be positive");
      }
      if (threshold < 0 || threshold > 100 || speed <= 0.0) {
        throw std::invalid_argument("occupied threshold or speed is invalid");
      }
      follower_required_clearance_ = radius + margin;
      follower_occupied_threshold_ = threshold;
      follower_command_speed_ = speed;
      // Take the escape rule's tolerance from the follower alongside the
      // clearance it belongs to. The two nodes have to agree on what counts as
      // an escape -- disagreeing means one proposes a chord the other vetoes --
      // and making it one more launch argument to keep in sync by hand is how
      // that goes wrong. Absent from the legacy Python follower's config, which
      // proposes no escapes at all, so fall back to this node's parameter.
      if (config.contains("escape_minimum_improvement")) {
        const auto improvement = config.at("escape_minimum_improvement").get<double>();
        if (!std::isfinite(improvement) || improvement <= 0.0) {
          throw std::invalid_argument("escape_minimum_improvement must be positive");
        }
        follower_escape_improvement_ = improvement;
      } else {
        follower_escape_improvement_.reset();
      }
      follower_config_reason_.clear();
    } catch (const std::exception & error) {
      follower_required_clearance_.reset();
      follower_escape_improvement_.reset();
      follower_occupied_threshold_.reset();
      follower_command_speed_.reset();
      follower_config_reason_ = error.what();
    }
  }

  void on_path(const nav_msgs::msg::Path & msg)
  {
    if (msg.header.frame_id != frame_id_) {
      return;
    }
    std::vector<Point2> points;
    points.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
      points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    if (points.empty()) {
      return;
    }
    for (const auto & point : points) {
      if (!finite(point)) {
        return;
      }
    }
    path_points_ = std::move(points);
    path_received_ = monotonic_time();
  }

  void on_map_pose(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (msg.header.frame_id != frame_id_) {
      return;
    }
    const Point2 point{msg.pose.position.x, msg.pose.position.y};
    if (finite(point)) {
      map_pose_ = point;
      map_pose_received_ = monotonic_time();
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    if (msg.header.frame_id != frame_id_) {
      return;
    }
    const auto & q = msg.info.origin.orientation;
    if (std::abs(q.x) > 1.0e-6 || std::abs(q.y) > 1.0e-6 ||
      std::abs(q.z) > 1.0e-6 || std::abs(q.w - 1.0) > 1.0e-6)
    {
      return;
    }
    GridMap next;
    next.width = msg.info.width;
    next.height = msg.info.height;
    next.resolution = msg.info.resolution;
    next.origin_x = msg.info.origin.position.x;
    next.origin_y = msg.info.origin.position.y;
    next.data.assign(msg.data.begin(), msg.data.end());
    if (!next.valid()) {
      return;
    }
    for (const auto value : next.data) {
      if (value < -1 || value > 100) {
        return;
      }
    }
    grid_ = std::move(next);
    map_received_ = monotonic_time();
  }

  void on_correction(const geometry_msgs::msg::PoseStamped & msg)
  {
    const auto & position = msg.pose.position;
    const auto & orientation = msg.pose.orientation;
    const std::array<double, 4> correction{
      position.x, position.y, position.z,
      yaw_from_quaternion(orientation.w, orientation.x, orientation.y, orientation.z)};
    const auto reason =
      correction_rejection_reason(correction, max_correction_m_, max_correction_yaw_);
    correction_valid_ = !reason.has_value();
    correction_reason_ = reason.value_or("");
    if (reason) {
      correction_.reset();
    } else {
      correction_ = correction;
    }
    correction_received_ = monotonic_time();
  }

  // --- health -------------------------------------------------------------
  void ensure_limiter()
  {
    if (!limiter_->position() && x0_) {
      limiter_->reset({*x0_, *y0_});
    }
  }

  std::optional<std::string> planner_health_reason()
  {
    const double now = monotonic_time();
    const std::pair<const char *, const char *> required[] = {
      {"/planner/status", "global planner status"},
      {"/planner/path", "global planner path"},
      {"/planner/inflated_map", "global planner costmap"},
      {follower_config_topic_.c_str(), "follower configuration"},
      {follower_valid_topic_.c_str(), "follower validity"},
      {follower_displacement_topic_.c_str(), "follower displacement"},
      {follower_goal_topic_.c_str(), "follower goal state"},
    };
    for (const auto & [topic, label] : required) {
      const auto publisher_count = count_publishers(topic);
      if (publisher_count != 1) {
        return format(
          "%s publisher count is %zu, expected exactly 1", label, publisher_count);
      }
    }
    if (!follower_valid_received_ || now - *follower_valid_received_ > follower_timeout_) {
      return format("follower validity stale for >%.2fs", follower_timeout_);
    }
    // Report invalidity before the remaining staleness checks. An invalid
    // follower stops publishing displacement and goal state, so those go stale a
    // fraction of a second later and would otherwise overwrite the real reason.
    if (!follower_valid_) {
      return std::string("follower validity is false");
    }
    if (!goal_received_ || now - *goal_received_ > follower_timeout_) {
      return format("follower goal state stale for >%.2fs", follower_timeout_);
    }
    if (!displacement_received_ || now - *displacement_received_ > follower_timeout_) {
      return format("VIO displacement stale for >%.2fs", follower_timeout_);
    }
    if (!vio_displacement_) {
      return std::string("VIO displacement is invalid");
    }
    if (!follower_config_received_) {
      return std::string("follower configuration not received");
    }
    if (!follower_config_reason_.empty()) {
      return "follower configuration rejected: " + follower_config_reason_;
    }
    if (std::abs(*follower_command_speed_ - limiter_->max_speed()) > 1.0e-6) {
      return format(
        "follower speed %.2fm/s does not match final command speed %.2fm/s",
        *follower_command_speed_, limiter_->max_speed());
    }
    const std::tuple<std::optional<double>, double, const char *> freshness[] = {
      {path_received_, path_timeout_, "path"},
      {map_received_, map_timeout_, "raw map"},
      {map_pose_received_, map_pose_timeout_, "map pose"},
    };
    for (const auto & [received, timeout, label] : freshness) {
      if (!received || now - *received > timeout) {
        return format("%s stale for >%.2fs", label, timeout);
      }
    }
    if (path_points_.empty() || !grid_ || !map_pose_) {
      return std::string("path-clearance validation inputs are invalid");
    }
    if (*displacement_received_ <= require_follower_after_) {
      return std::string("waiting for follower data after PX4 local reset");
    }
    if (!correction_received_ || now - *correction_received_ > correction_timeout_) {
      return format("native correction stale for >%.2fs", correction_timeout_);
    }
    if (!correction_valid_) {
      return "native correction rejected: " + correction_reason_;
    }
    if (!correction_) {
      return std::string("native correction is invalid");
    }
    return std::nullopt;
  }

  std::optional<std::string> preflight_reason()
  {
    if (!pos_valid()) {
      return std::string("PX4 local position invalid");
    }
    if (const auto reason = vio_fault_reason()) {
      return reason;
    }
    if (const auto reason = planner_health_reason()) {
      return reason;
    }
    if (goal_reached_) {
      return std::string("planner goal is already reached; provide a new route");
    }
    return std::nullopt;
  }

  // --- setpoint feedforward ------------------------------------------------
  std::pair<double, double> setpoint_velocity_xy() const
  {
    // HorizontalCommandLimiter::velocity is already the speed- and
    // acceleration-limited velocity of the point being published, in the same
    // NED frame, and by construction it is that point's own derivative.
    // Gated to an advancing route on purpose: in a fault hold latch_fault_hold
    // resets the limiter, but a command hold does not, and the base-class LAND
    // state publishes through here too. NaN restores pure position control.
    if (!horizontal_feedforward_) {
      return {kNan, kNan};
    }
    if (state_ != State::Route || holding_for_fault_ || route_command_holding_) {
      return {kNan, kNan};
    }
    return {limiter_->velocity().first, limiter_->velocity().second};
  }

  std::pair<double, double> setpoint_acceleration_xy(double vx, double vy)
  {
    // The derivative of the velocity just published. PX4 sums acceleration into
    // the velocity loop the same way it sums velocity into the position loop.
    // Safe to differentiate because the limiter has already
    // acceleration-bounded that velocity.
    if (std::isnan(vx) || std::isnan(vy) || !last_ff_velocity_) {
      last_ff_velocity_ =
        std::isnan(vx) ? std::nullopt : std::optional<Point2>{{vx, vy}};
      return {kNan, kNan};
    }
    const double limit = limiter_->max_acceleration();
    double ax = (vx - last_ff_velocity_->first) / dt_;
    double ay = (vy - last_ff_velocity_->second) / dt_;
    last_ff_velocity_ = Point2{vx, vy};
    const double magnitude = std::hypot(ax, ay);
    if (magnitude > limit && magnitude > 0.0) {
      const double scale = limit / magnitude;
      ax *= scale;
      ay *= scale;
    }
    return {ax, ay};
  }

  // --- status / holds ------------------------------------------------------
  void publish_route_status(const std::string & text, const std::string & kind)
  {
    const double now = monotonic_time();
    if (kind == last_route_status_kind_ && now - last_route_status_time_ < 0.2) {
      return;
    }
    if (text == last_route_status_ && now - last_route_status_time_ < 1.0) {
      return;
    }
    last_route_status_ = text;
    last_route_status_time_ = now;
    std_msgs::msg::String msg;
    msg.data = text;
    route_status_pub_->publish(msg);
    if (kind != last_route_status_kind_) {
      last_route_status_kind_ = kind;
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
    }
  }

  void latch_fault_hold(const std::string & reason)
  {
    const double now = monotonic_time();
    if (!holding_for_fault_) {
      if (pos_ && std::isfinite(pos_->x) && std::isfinite(pos_->y)) {
        limiter_->reset({static_cast<double>(pos_->x), static_cast<double>(pos_->y)});
      }
      holding_for_fault_ = true;
    }
    // Never let an unvalidated route command advance internally while the
    // published command is holding. A recovery restarts from actual pose.
    route_limiter_->clear();
    if (!planner_fault_since_) {
      planner_fault_since_ = now;
    }
    planner_fault_reason_text_ = reason;
    const double elapsed = now - *planner_fault_since_;
    publish_route_status(
      format(
        "HOLD planner fault: %s; land in %.1fs", reason.c_str(),
        std::max(0.0, planner_fault_land_time_ - elapsed)),
      "HOLD");
    if (is_armed() && elapsed >= planner_fault_land_time_) {
      trigger_landing("planner fault persisted: " + reason);
    }
  }

  void clear_planner_fault()
  {
    planner_fault_since_.reset();
    planner_fault_reason_text_.clear();
    holding_for_fault_ = false;
  }

  std::optional<std::string> hold_route_command(const std::string & reason)
  {
    // The accepted path stays installed and the last cleared setpoint stays on
    // the wire, so ordinary replanning jitter cannot start the land timer. Only
    // a stall that outlives the grace window becomes a flight fault.
    const double now = monotonic_time();
    if (!route_command_stall_since_) {
      route_command_stall_since_ = now;
    }
    const double elapsed = now - *route_command_stall_since_;
    if (elapsed >= route_command_grace_) {
      return format("route command stalled %.1fs: %s", elapsed, reason.c_str());
    }
    route_command_holding_ = true;
    publish_route_status(
      format(
        "COMMAND_HOLD %s; fault in %.1fs", reason.c_str(),
        std::max(0.0, route_command_grace_ - elapsed)),
      "COMMAND_HOLD");
    return std::nullopt;
  }

  void clear_route_command_stall() {route_command_stall_since_.reset();}

  // --- heading tracking ----------------------------------------------------
  double route_yaw() const
  {
    if (!yaw_track_ || !yaw_target_) {
      return yaw0_.value_or(0.0);
    }
    return *yaw_target_;
  }

  std::optional<double> yaw_track_error() const
  {
    if (!yaw_track_ || !yaw_target_) {
      return std::nullopt;
    }
    if (!pos_ || !std::isfinite(pos_->heading)) {
      return std::nullopt;
    }
    return wrap_pi(*yaw_target_ - static_cast<double>(pos_->heading));
  }

  void update_yaw_target(const Point2 & ned_displacement)
  {
    if (!yaw_track_) {
      return;
    }
    yaw_target_ = track_yaw_target(
      yaw_target_,
      ned_track_heading(ned_displacement, yaw_track_min_displacement_),
      yaw_track_deadband_);
    if (yaw_align_error_ <= 0.0) {
      yaw_holding_ = false;
      return;
    }
    const auto error = yaw_track_error();
    if (!error) {
      yaw_holding_ = false;
    } else if (std::abs(*error) > yaw_align_error_) {
      yaw_holding_ = true;
    } else if (std::abs(*error) <= yaw_resume_error_) {
      yaw_holding_ = false;
    }
  }

  void freeze_yaw_target()
  {
    yaw_holding_ = false;
    if (yaw_track_ && yaw_cmd_) {
      yaw_target_ = *yaw_cmd_;
    }
  }

  ClearanceProbe clearance_probe() const
  {
    return [this](const Point2 & start, const Point2 & end) -> std::optional<double> {
             if (!grid_ || !follower_occupied_threshold_) {
               return std::nullopt;
             }
             return segment_minimum_clearance(
               *grid_, start, end, *follower_occupied_threshold_);
           };
  }

  [[nodiscard]] ClearanceEscapeLimits escape_limits() const
  {
    ClearanceEscapeLimits limits;
    limits.required_clearance = *follower_required_clearance_;
    limits.minimum_improvement =
      follower_escape_improvement_.value_or(escape_minimum_improvement_);
    return limits;
  }

  /// Whether the chord this node is about to put on the wire may be commanded.
  ///
  /// At or above the hard clearance this is exactly the old
  /// segment_has_clearance() test -- the normal envelope is not relaxed here
  /// any more than it is in the follower. Below it, the same escape rule
  /// applies: no point of the chord may be closer to an occupied cell than the
  /// pose already is. Without this the adapter vetoed every escape the follower
  /// proposed (flight 20260829T085734Z, COMMAND_HOLD at 31.33s against
  /// CLEARANCE_ESCAPING start=0.227m), and the POSE_INSIDE_CLEARANCE deadlock
  /// simply moved one layer up. The endpoint-improvement half of the rule stays
  /// with the follower's target selection: this is the acceleration-limited
  /// command, and demanding a centimetre of gain from one 20 Hz step would
  /// reject every escape again.
  bool command_chord_permitted(const Point2 & start, const Point2 & end) const
  {
    if (!grid_ || !follower_required_clearance_ || !follower_occupied_threshold_) {
      return false;
    }
    const auto probe = clearance_probe();
    const auto start_clearance = probe(start, start);
    if (!start_clearance) {
      // Unknown or outside-map space under the vehicle stays blocked. It is
      // never something an escape may be attempted from.
      return false;
    }
    return command_chord_admissible(
      probe, start, end, escape_limits(), *start_clearance,
      ChordRole::IntermediateCarrot);
  }

  /// The pose's own clearance, for telemetry. No value when it is not known.
  [[nodiscard]] std::optional<double> pose_clearance() const
  {
    if (!grid_ || !map_pose_ || !follower_occupied_threshold_) {
      return std::nullopt;
    }
    return clearance_probe()(*map_pose_, *map_pose_);
  }

  // --- the route command ---------------------------------------------------
  /// Returns (fault, deferral). `fault` lands the aircraft on the usual timer.
  /// `deferral` is a transient the accepted route survives: the installed path
  /// and the last validated setpoint both stand.
  std::pair<std::optional<std::string>, std::optional<std::string>> update_route_command()
  {
    const auto ned_displacement = vio_enu_displacement_to_ned(*vio_displacement_);
    update_yaw_target(ned_displacement);
    std::optional<std::string> deferral;

    const auto clearance_check = [this](const Point2 & start, const Point2 & end) {
        return command_chord_permitted(start, end);
      };

    // Replanning mid-slew is what generates unjoinable paths: translation is
    // paused, the vehicle drifts as it pivots, and each republished route comes
    // out shifted from the command that is standing still. Keep the accepted
    // path until the nose is back on the leg.
    if (yaw_holding_ && !replan_during_yaw_align_) {
      if (!route_limiter_->path()) {
        return {std::nullopt, std::string("waiting for yaw alignment before route entry")};
      }
    } else {
      try {
        route_limiter_->set_path(path_points_, *map_pose_, clearance_check);
      } catch (const std::exception & error) {
        // An unjoinable replacement is not a flight fault. The accepted path is
        // still installed and still validated, so keep flying it and retry when
        // the planner republishes.
        deferral = std::string("path replacement deferred: ") + error.what();
      }
    }

    if (!route_limiter_->path()) {
      return {
        std::nullopt,
        deferral.value_or(std::string("waiting for a joinable route path"))};
    }

    const auto restore_point = route_limiter_->snapshot();
    Point2 final_map{};
    try {
      const auto map_displacement =
        vio_displacement_to_map(*vio_displacement_, (*correction_)[3]);
      const Point2 desired_map{
        map_pose_->first + map_displacement.first,
        map_pose_->second + map_displacement.second};
      final_map = route_limiter_->update(
        desired_map, dt_, !yaw_holding_, map_pose_);
    } catch (const std::exception & error) {
      route_limiter_->restore(restore_point);
      return {
        std::nullopt,
        std::string("path-constrained command rejected: ") + error.what()};
    }

    // This is deliberately after every limiting/projection operation: the exact
    // point and swept segment that PX4 will receive must be safe. Rewinding
    // leaves last tick's cleared command on the wire.
    if (!command_chord_permitted(*map_pose_, final_map)) {
      route_limiter_->restore(restore_point);
      const auto clearance = pose_clearance();
      return {
        std::nullopt,
        std::string("post-limiter command has insufficient clearance") +
        (clearance ? format(" (pose clearance %.3fm)", *clearance) : std::string())};
    }

    const Point2 final_map_displacement{
      final_map.first - map_pose_->first, final_map.second - map_pose_->second};
    const auto final_vio_displacement =
      map_displacement_to_vio(final_map_displacement, (*correction_)[3]);
    const auto final_ned_displacement =
      vio_enu_displacement_to_ned(final_vio_displacement);
    const Point2 final_ned{
      static_cast<double>(pos_->x) + final_ned_displacement.first,
      static_cast<double>(pos_->y) + final_ned_displacement.second};
    if (distance(final_ned, {*x0_, *y0_}) > geofence_radius_) {
      // The vehicle itself is still inside; holding the last command is the
      // recovery. An actual breach is caught by geofence_breached().
      route_limiter_->restore(restore_point);
      return {
        std::nullopt,
        std::string("path-constrained command lies outside flight geofence")};
    }

    const auto velocity_vio =
      map_displacement_to_vio(route_limiter_->velocity(), (*correction_)[3]);
    const auto velocity_ned = vio_enu_displacement_to_ned(velocity_vio);
    limiter_->adopt(final_ned, velocity_ned);
    return {std::nullopt, deferral};
  }

  bool geofence_breached() const
  {
    if (!pos_ || !x0_) {
      return false;
    }
    return std::hypot(pos_->x - *x0_, pos_->y - *y0_) >
           (geofence_radius_ + geofence_tolerance_);
  }

  // --- status text ---------------------------------------------------------
  std::string yaw_status_text() const
  {
    if (!yaw_track_ || !yaw_target_) {
      return " yaw=hold";
    }
    std::string text = format(" yaw_target=%.0fdeg", degrees(*yaw_target_));
    if (const auto error = yaw_track_error()) {
      text += format(" yaw_error=%.0fdeg", degrees(*error));
    }
    return text;
  }

  std::string route_status_kind() const
  {
    if (yaw_holding_) {
      return "YAW_ALIGN";
    }
    if (route_limiter_->waiting_vertex()) {
      return "CORNER_HOLD";
    }
    return "ROUTE";
  }

  std::string route_status_text() const
  {
    if (yaw_holding_) {
      return "YAW_ALIGN translation paused while turning;" + yaw_status_text();
    }
    if (route_limiter_->waiting_vertex()) {
      const auto vertex =
        route_limiter_->path()->point_at(*route_limiter_->waiting_vertex());
      return format(
        "CORNER_HOLD final command waiting for vehicle distance=%.2fm",
        distance(*map_pose_, vertex)) + yaw_status_text();
    }
    double path_offset = kNan;
    if (route_limiter_->path() && route_limiter_->position()) {
      path_offset = route_limiter_->path()->project(*route_limiter_->position()).cross_track;
    }
    auto text = format(
      "ROUTE valid displacement=%.2fm command_speed=%.2fm/s path_offset=%.3fm",
      std::hypot(vio_displacement_->first, vio_displacement_->second),
      std::hypot(limiter_->velocity().first, limiter_->velocity().second),
      path_offset);
    // Make it visible in the bag when the command on the wire was permitted by
    // the escape rule rather than by the full hard envelope.
    const auto clearance = pose_clearance();
    if (clearance && follower_required_clearance_ &&
      *clearance + 1.0e-9 < *follower_required_clearance_)
    {
      text += format(
        " escaping clearance=%.3f/%.3fm", *clearance, *follower_required_clearance_);
    }
    return text + yaw_status_text();
  }

  // --- configuration -------------------------------------------------------
  std::string follower_displacement_topic_, follower_valid_topic_, follower_goal_topic_;
  std::string path_topic_, map_topic_, map_pose_topic_, follower_config_topic_;
  std::string frame_id_;
  double follower_timeout_{}, correction_timeout_{}, path_timeout_{};
  double map_timeout_{}, map_pose_timeout_{};
  double planner_fault_land_time_{}, goal_hold_time_{};
  double escape_minimum_improvement_{};
  double max_follower_displacement_{}, max_correction_m_{}, max_correction_yaw_{};
  bool horizontal_feedforward_{};
  double route_command_grace_{};
  bool replan_during_yaw_align_{};
  double geofence_radius_{}, geofence_tolerance_{};
  double transit_horizontal_error_{}, pre_route_max_horizontal_error_{};
  bool yaw_track_{};
  double yaw_track_min_displacement_{}, yaw_track_deadband_{};
  double yaw_align_error_{}, yaw_resume_error_{};

  std::unique_ptr<HorizontalCommandLimiter> limiter_;
  std::unique_ptr<PathCommandLimiter> route_limiter_;

  // --- runtime state -------------------------------------------------------
  std::optional<Point2> last_ff_velocity_;
  bool follower_valid_{false};
  std::optional<double> follower_valid_received_;
  bool goal_reached_{false};
  std::optional<double> goal_received_;
  std::optional<Point2> vio_displacement_;
  std::optional<double> displacement_received_;
  bool correction_valid_{false};
  std::string correction_reason_{"not received"};
  std::optional<double> correction_received_;
  std::optional<std::array<double, 4>> correction_;
  std::vector<Point2> path_points_;
  std::optional<double> path_received_;
  std::optional<GridMap> grid_;
  std::optional<double> map_received_;
  std::optional<Point2> map_pose_;
  std::optional<double> map_pose_received_;
  std::optional<double> follower_config_received_;
  std::optional<double> follower_required_clearance_;
  std::optional<double> follower_escape_improvement_;
  std::optional<int> follower_occupied_threshold_;
  std::optional<double> follower_command_speed_;
  std::string follower_config_reason_{"not received"};
  std::optional<double> planner_fault_since_;
  std::string planner_fault_reason_text_;
  std::optional<double> route_command_stall_since_;
  bool route_command_holding_{false};
  std::optional<double> goal_since_;
  std::optional<double> yaw_target_;
  bool yaw_holding_{false};
  bool holding_for_fault_{true};
  double require_follower_after_{0.0};
  std::optional<std::array<int, 3>> local_reset_counters_;
  std::string last_route_status_;
  double last_route_status_time_{0.0};
  std::string last_route_status_kind_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr displacement_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_, goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr correction_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr map_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_status_pub_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px4_vio_bridge::OffboardGlobalPlanner>();
  rclcpp::spin(node);
  node->on_shutdown();
  rclcpp::shutdown();
  return 0;
}
