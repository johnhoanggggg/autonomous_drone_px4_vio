// Observation-only C++ counterpart of route_follower_monitor.py.
//
// It publishes the same proposed carrot, structured validity and telemetry,
// never publishes /fmu/in/*, and shares the Python node's singleton lock.

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "px4_vio_bridge/grid_clearance.hpp"
#include "px4_vio_bridge/path_geometry.hpp"
#include "px4_vio_bridge/route_follower.hpp"

namespace px4_vio_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double monotonic_seconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

std::string format(double value, int places)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(places);
  stream << value;
  return stream.str();
}

std::optional<std::string> pose_rejection_reason(
  const geometry_msgs::msg::PoseStamped & message)
{
  const auto & position = message.pose.position;
  const auto & orientation = message.pose.orientation;
  const std::array<double, 7> values{
    position.x, position.y, position.z,
    orientation.w, orientation.x, orientation.y, orientation.z};
  if (std::any_of(values.begin(), values.end(), [](double value) {return !std::isfinite(value);})) {
    return std::string("pose contains a non-finite value");
  }
  const double quaternion_norm = std::sqrt(
    orientation.w * orientation.w + orientation.x * orientation.x +
    orientation.y * orientation.y + orientation.z * orientation.z);
  if (quaternion_norm <= 1.0e-6) {
    return std::string("quaternion has zero norm");
  }
  const bool at_origin = std::abs(position.x) <= 1.0e-6 &&
    std::abs(position.y) <= 1.0e-6 && std::abs(position.z) <= 1.0e-6;
  const auto reset_distance = [&](double sign) {
      return std::max({
        std::abs(orientation.w - sign * 0.5),
        std::abs(orientation.x - sign * 0.5),
        std::abs(orientation.y - sign * 0.5),
        std::abs(orientation.z - sign * 0.5)});
    };
  if (at_origin && std::min(reset_distance(1.0), reset_distance(-1.0)) <= 1.0e-3) {
    return std::string("RTABMap reset sentinel");
  }
  return std::nullopt;
}

class ProcessSingleton
{
public:
  explicit ProcessSingleton(const std::string & role)
  {
    const auto * domain_env = std::getenv("ROS_DOMAIN_ID");
    const std::string domain = domain_env != nullptr ? domain_env : "0";
    const auto sanitise = [](std::string text) {
        for (auto & character : text) {
          const bool safe = std::isalnum(static_cast<unsigned char>(character)) != 0 ||
            character == '_' || character == '.' || character == '-';
          if (!safe) {
            character = '_';
          }
        }
        return text;
      };
    path_ = "/tmp/px4_vio_bridge_" + sanitise(role) +
      "_ros_domain_" + sanitise(domain) + ".lock";
    descriptor_ = ::open(path_.c_str(), O_RDWR | O_CREAT, 0644);
    if (descriptor_ < 0) {
      throw std::runtime_error("cannot open singleton lock " + path_);
    }
    if (::flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
      char holder[64] = {0};
      const auto read_bytes = ::read(descriptor_, holder, sizeof(holder) - 1);
      ::close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error(
              "duplicate " + role + " for ROS_DOMAIN_ID=" + domain +
              "; existing holder PID=" +
              (read_bytes > 0 ? std::string(holder) : std::string("unknown")));
    }
    if (::ftruncate(descriptor_, 0) != 0) {
      throw std::runtime_error("cannot truncate singleton lock " + path_);
    }
    const auto pid = std::to_string(::getpid());
    if (::write(descriptor_, pid.c_str(), pid.size()) < 0) {
      throw std::runtime_error("cannot write singleton lock " + path_);
    }
  }

  ~ProcessSingleton()
  {
    if (descriptor_ >= 0) {
      ::flock(descriptor_, LOCK_UN);
      ::close(descriptor_);
    }
  }

  ProcessSingleton(const ProcessSingleton &) = delete;
  ProcessSingleton & operator=(const ProcessSingleton &) = delete;

private:
  std::string path_;
  int descriptor_{-1};
};

}  // namespace

class RouteFollowerNode final : public rclcpp::Node
{
public:
  RouteFollowerNode()
  : Node("route_follower_monitor")
  {
    path_topic_ = declare_parameter<std::string>("path_topic", "/planner/path");
    map_topic_ = declare_parameter<std::string>("map_topic", "/rtabmap/grid");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/rtabmap/pose");
    raw_vio_topic_ = declare_parameter<std::string>("raw_vio_topic", "/rtabmap/vio_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/waypoint/clicked");
    goal_terminal_topic_ =
      declare_parameter<std::string>("goal_terminal_topic", "/planner/goal_terminal");
    correction_topic_ =
      declare_parameter<std::string>("correction_topic", "/rtabmap/odom_correction");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    path_timeout_ = declare_parameter<double>("path_timeout", 3.0);
    map_timeout_ = declare_parameter<double>("map_timeout", 3.0);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 1.0);
    vio_timeout_ = declare_parameter<double>("vio_timeout", 0.5);
    correction_timeout_ = declare_parameter<double>("correction_timeout", 1.0);
    max_correction_m_ = declare_parameter<double>("max_correction_m", 0.50);
    max_correction_yaw_deg_ = declare_parameter<double>("max_correction_yaw_deg", 15.0);
    lookahead_ = declare_parameter<double>("lookahead", 0.60);
    lookahead_step_ = std::max(0.01, declare_parameter<double>("lookahead_step", 0.05));
    min_lookahead_ = std::max(0.0, declare_parameter<double>("min_lookahead", 0.05));
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.25);
    safety_margin_ = declare_parameter<double>("safety_margin", 0.05);
    max_carrot_speed_ = declare_parameter<double>("max_carrot_speed", 0.10);
    max_carrot_acceleration_ = declare_parameter<double>("max_carrot_acceleration", 0.30);
    max_cross_track_ = declare_parameter<double>("max_cross_track", 0.60);
    cross_track_resume_ = declare_parameter<double>("cross_track_resume", 0.05);
    cross_track_recovery_time_ =
      declare_parameter<double>("cross_track_recovery_time", 1.0);
    path_start_tolerance_ = declare_parameter<double>("path_start_tolerance", 0.75);
    arrival_tolerance_ = declare_parameter<double>("arrival_tolerance", 0.12);
    arrival_release_tolerance_ =
      declare_parameter<double>("arrival_release_tolerance", 0.20);
    correction_translation_trigger_ =
      declare_parameter<double>("correction_translation_trigger", 0.05);
    correction_yaw_trigger_deg_ =
      declare_parameter<double>("correction_yaw_trigger_deg", 1.5);
    correction_filter_time_constant_ =
      declare_parameter<double>("correction_filter_time_constant", 0.35);
    correction_material_translation_ =
      declare_parameter<double>("correction_material_translation", 0.03);
    correction_material_yaw_deg_ =
      declare_parameter<double>("correction_material_yaw_deg", 0.75);
    correction_settle_time_ = declare_parameter<double>("correction_settle_time", 0.40);
    correction_cooldown_ = declare_parameter<double>("correction_cooldown", 8.0);

    required_clearance_ = robot_radius_ + safety_margin_;
    if (required_clearance_ <= 0.0) {
      throw std::invalid_argument("robot_radius + safety_margin must be positive");
    }
    if (min_lookahead_ > lookahead_) {
      throw std::invalid_argument("min_lookahead must not exceed lookahead");
    }
    correction_gate_ = std::make_unique<CorrectionReplanGate>(
      correction_translation_trigger_, correction_yaw_trigger_deg_ * kPi / 180.0,
      correction_filter_time_constant_, correction_material_translation_,
      correction_material_yaw_deg_ * kPi / 180.0, correction_settle_time_,
      correction_cooldown_);
    follower_ = std::make_unique<PositionRouteFollower>(
      lookahead_, max_carrot_speed_, max_carrot_acceleration_, max_cross_track_,
      cross_track_resume_, cross_track_recovery_time_, arrival_tolerance_,
      arrival_release_tolerance_);
    last_tick_ = monotonic_seconds();

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {on_map(*msg);});
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10, [this](nav_msgs::msg::Path::ConstSharedPtr msg) {on_path(*msg);});
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_pose(*msg);});
    raw_vio_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      raw_vio_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_raw_vio(*msg);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {on_goal(*msg);});
    goal_terminal_sub_ = create_subscription<std_msgs::msg::Bool>(
      goal_terminal_topic_, 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {goal_terminal_ = msg->data;});
    correction_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      correction_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_correction(*msg);});

    carrot_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner/follower/carrot", 10);
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner/follower/lookahead", 10);
    displacement_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/planner/follower/displacement", 10);
    vio_displacement_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/planner/follower/vio_displacement", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planner/follower/status", 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/planner/follower/valid", 10);
    goal_reached_pub_ =
      create_publisher<std_msgs::msg::Bool>("/planner/follower/goal_reached", 10);
    progress_pub_ = create_publisher<std_msgs::msg::Float32>("/planner/follower/progress", 10);
    path_progress_pub_ =
      create_publisher<std_msgs::msg::Float32>("/planner/follower/path_progress", 10);
    remaining_pub_ = create_publisher<std_msgs::msg::Float32>("/planner/follower/remaining", 10);
    cross_track_pub_ =
      create_publisher<std_msgs::msg::Float32>("/planner/follower/cross_track", 10);
    generation_pub_ =
      create_publisher<std_msgs::msg::Int32>("/planner/follower/path_generation", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planner/follower/markers", 10);
    config_pub_ = create_publisher<std_msgs::msg::String>(
      "/planner/follower/config", map_qos);
    publish_config();

    const double bounded_rate = std::max(1.0, rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / bounded_rate), [this]() {tick();});
    RCLCPP_WARN(
      get_logger(),
      "cpp_route_follower: OBSERVATION ONLY; position proposals only, publishes no PX4 commands");
  }

private:
  void publish_config()
  {
    nlohmann::json config{
      {"implementation", "cpp"}, {"path_topic", path_topic_}, {"map_topic", map_topic_},
      {"pose_topic", pose_topic_}, {"raw_vio_topic", raw_vio_topic_},
      {"goal_topic", goal_topic_}, {"goal_terminal_topic", goal_terminal_topic_},
      {"correction_topic", correction_topic_}, {"frame_id", frame_id_},
      {"rate_hz", rate_hz_}, {"path_timeout", path_timeout_},
      {"map_timeout", map_timeout_}, {"pose_timeout", pose_timeout_},
      {"vio_timeout", vio_timeout_}, {"correction_timeout", correction_timeout_},
      {"max_correction_m", max_correction_m_},
      {"max_correction_yaw_deg", max_correction_yaw_deg_}, {"lookahead", lookahead_},
      {"lookahead_step", lookahead_step_}, {"min_lookahead", min_lookahead_},
      {"occupied_threshold", occupied_threshold_}, {"robot_radius", robot_radius_},
      {"safety_margin", safety_margin_}, {"max_carrot_speed", max_carrot_speed_},
      {"max_carrot_acceleration", max_carrot_acceleration_},
      {"max_cross_track", max_cross_track_}, {"cross_track_resume", cross_track_resume_},
      {"cross_track_recovery_time", cross_track_recovery_time_},
      {"path_start_tolerance", path_start_tolerance_},
      {"arrival_tolerance", arrival_tolerance_},
      {"arrival_release_tolerance", arrival_release_tolerance_},
      {"correction_translation_trigger", correction_translation_trigger_},
      {"correction_yaw_trigger_deg", correction_yaw_trigger_deg_},
      {"correction_filter_time_constant", correction_filter_time_constant_},
      {"correction_material_translation", correction_material_translation_},
      {"correction_material_yaw_deg", correction_material_yaw_deg_},
      {"correction_settle_time", correction_settle_time_},
      {"correction_cooldown", correction_cooldown_}};
    std_msgs::msg::String message;
    message.data = config.dump();
    config_pub_->publish(message);
  }

  void publish_status(const std::string & status, bool valid = false, bool goal_reached = false)
  {
    std_msgs::msg::String status_message;
    status_message.data = status;
    status_pub_->publish(status_message);
    std_msgs::msg::Bool valid_message;
    valid_message.data = valid;
    valid_pub_->publish(valid_message);
    std_msgs::msg::Bool goal_message;
    goal_message.data = goal_reached;
    goal_reached_pub_->publish(goal_message);
  }

  void on_pose(const geometry_msgs::msg::PoseStamped & message)
  {
    if (message.header.frame_id != frame_id_) {
      return;
    }
    const auto & position = message.pose.position;
    if (std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z)) {
      pose_ = std::array<double, 3>{position.x, position.y, position.z};
      pose_received_ = monotonic_seconds();
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid & message)
  {
    if (message.header.frame_id != frame_id_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "map rejected: frame '%s' != '%s'",
        message.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    const auto & q = message.info.origin.orientation;
    if (std::abs(q.x) > 1.0e-6 || std::abs(q.y) > 1.0e-6 ||
      std::abs(q.z) > 1.0e-6 || std::abs(q.w - 1.0) > 1.0e-6)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "map rejected: rotated occupancy-grid origins are unsupported");
      return;
    }
    GridMap grid;
    grid.width = message.info.width;
    grid.height = message.info.height;
    grid.resolution = message.info.resolution;
    grid.origin_x = message.info.origin.position.x;
    grid.origin_y = message.info.origin.position.y;
    grid.data.assign(message.data.begin(), message.data.end());
    if (!grid.valid() || std::any_of(grid.data.begin(), grid.data.end(), [](std::int8_t value) {
        return value < -1 || value > 100;
      }))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "map rejected: invalid grid");
      return;
    }
    grid_ = std::move(grid);
    map_received_ = monotonic_seconds();
  }

  bool command_has_clearance(const Point2 & start, const Point2 & end) const
  {
    return grid_ && segment_has_clearance(
      *grid_, start, end, required_clearance_, occupied_threshold_);
  }

  std::optional<double> safe_lookahead(const Point2 & pose) const
  {
    const auto projection = follower_->path()->project(pose);
    for (double candidate = lookahead_;
      candidate + 1.0e-12 >= min_lookahead_; candidate -= lookahead_step_)
    {
      const auto target = follower_->path()->point_at(projection.along + candidate);
      if (command_has_clearance(pose, target)) {
        return candidate;
      }
    }
    return std::nullopt;
  }

  void on_raw_vio(const geometry_msgs::msg::PoseStamped & message)
  {
    const double now = monotonic_seconds();
    const auto reason = pose_rejection_reason(message);
    raw_vio_seen_ = true;
    raw_vio_received_ = now;
    raw_vio_valid_ = !reason;
    raw_vio_reason_ = reason.value_or("");
    if (reason) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "raw VIO rejected: %s", reason->c_str());
    }
  }

  void on_goal(const geometry_msgs::msg::PointStamped & message)
  {
    if (message.header.frame_id != frame_id_) {
      return;
    }
    follower_->reset_route_progress();
    follower_->clear_path();
    path_start_.reset();
    goal_terminal_ = false;
  }

  void on_path(const nav_msgs::msg::Path & message)
  {
    if (message.header.frame_id != frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "path rejected: frame '%s' != '%s'",
        message.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    const double now = monotonic_seconds();
    path_received_ = now;
    if (message.poses.empty()) {
      follower_->clear_path();
      path_start_.reset();
      return;
    }
    std::vector<Point2> points;
    points.reserve(message.poses.size());
    for (const auto & pose : message.poses) {
      points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    const Point2 anchor = pose_ ? Point2{(*pose_)[0], (*pose_)[1]} : points.front();
    try {
      const bool changed = follower_->set_path(points, anchor);
      correction_gate_->path_received(now);
      path_start_ = points.front();
      if (changed) {
        RCLCPP_INFO(
          get_logger(), "accepted follower route generation %d", follower_->generation());
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "path rejected: %s", error.what());
      follower_->clear_path();
      path_start_.reset();
    }
  }

  void on_correction(const geometry_msgs::msg::PoseStamped & message)
  {
    const double now = monotonic_seconds();
    const auto & p = message.pose.position;
    const auto & q = message.pose.orientation;
    const Correction4 correction{
      p.x, p.y, p.z, yaw_from_quaternion(q.w, q.x, q.y, q.z)};
    const auto reason = correction_rejection_reason(
      correction, max_correction_m_, max_correction_yaw_deg_ * kPi / 180.0);
    correction_seen_ = true;
    correction_received_ = now;
    correction_valid_ = !reason;
    correction_reason_ = reason.value_or("");
    if (reason) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "native correction rejected: %s",
        reason->c_str());
      return;
    }
    correction_ = correction;
    if (correction_gate_->observe(correction, now)) {
      const auto [translation, yaw] = correction_gate_->last_trigger_delta();
      RCLCPP_WARN(
        get_logger(),
        "persistent map correction %.3fm/%.2fdeg; coalescing optimization and "
        "waiting for one fresh path",
        translation, yaw * 180.0 / kPi);
    }
  }

  geometry_msgs::msg::PoseStamped pose_message(
    const Point2 & xy, double z, const builtin_interfaces::msg::Time & stamp) const
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.pose.position.x = xy.first;
    message.pose.position.y = xy.second;
    message.pose.position.z = z;
    message.pose.orientation.w = 1.0;
    return message;
  }

  void publish_markers(
    const FollowResult & result, const builtin_interfaces::msg::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    const std::array<Point2, 2> points{result.desired_carrot, result.commanded_carrot};
    const std::array<std::string, 2> names{"desired_lookahead", "commanded_carrot"};
    for (std::size_t index = 0; index < points.size(); ++index) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = stamp;
      marker.ns = names[index];
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = points[index].first;
      marker.pose.position.y = points[index].second;
      marker.pose.position.z = (*pose_)[2];
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.14;
      marker.color.r = index == 0 ? 1.0F : 0.0F;
      marker.color.g = index == 0 ? 0.7F : 1.0F;
      marker.color.b = index == 0 ? 0.0F : 1.0F;
      marker.color.a = 1.0F;
      array.markers.push_back(marker);
    }
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = stamp;
    line.ns = "commanded_displacement";
    line.id = 2;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.025;
    line.color.g = line.color.b = line.color.a = 1.0F;
    geometry_msgs::msg::Point first;
    first.x = (*pose_)[0];
    first.y = (*pose_)[1];
    first.z = (*pose_)[2];
    geometry_msgs::msg::Point second;
    second.x = result.commanded_carrot.first;
    second.y = result.commanded_carrot.second;
    second.z = (*pose_)[2];
    line.points = {first, second};
    array.markers.push_back(line);
    markers_pub_->publish(array);
  }

  void interrupt_and_publish(const std::string & status)
  {
    follower_->interrupt_cross_track_recovery();
    publish_status(status);
  }

  void tick()
  {
    const double now = monotonic_seconds();
    const double dt = std::max(1.0e-3, std::min(0.5, now - last_tick_));
    last_tick_ = now;
    if (!raw_vio_seen_) {interrupt_and_publish("WAITING_FOR_VIO"); return;}
    if (!raw_vio_valid_) {interrupt_and_publish("INVALID_VIO reason=" + raw_vio_reason_); return;}
    if (now - raw_vio_received_ > vio_timeout_) {
      interrupt_and_publish("STALE_VIO age=" + format(now - raw_vio_received_, 2) + "s"); return;
    }
    if (!correction_seen_) {interrupt_and_publish("WAITING_FOR_CORRECTION"); return;}
    if (!correction_valid_) {
      interrupt_and_publish("CORRECTION_REJECTED reason=" + correction_reason_); return;
    }
    if (now - correction_received_ > correction_timeout_) {
      interrupt_and_publish(
        "STALE_CORRECTION age=" + format(now - correction_received_, 2) + "s"); return;
    }
    if (!pose_) {interrupt_and_publish("WAITING_FOR_POSE"); return;}
    if (now - pose_received_ > pose_timeout_) {
      interrupt_and_publish("STALE_POSE age=" + format(now - pose_received_, 2) + "s"); return;
    }
    if (!grid_) {interrupt_and_publish("WAITING_FOR_MAP"); return;}
    if (now - map_received_ > map_timeout_) {
      interrupt_and_publish("STALE_MAP age=" + format(now - map_received_, 2) + "s"); return;
    }
    if (!follower_->path()) {interrupt_and_publish("WAITING_FOR_PATH"); return;}
    if (now - path_received_ > path_timeout_) {
      interrupt_and_publish("STALE_PATH age=" + format(now - path_received_, 2) + "s"); return;
    }
    const bool was_waiting_for_correction = correction_gate_->pending();
    if (correction_gate_->waiting(now)) {
      interrupt_and_publish("WAITING_FOR_POST_CORRECTION_PATH"); return;
    }
    if (was_waiting_for_correction) {
      RCLCPP_INFO(
        get_logger(), "map correction settled and fresh path received; follower resumed");
    }
    const Point2 pose_xy{(*pose_)[0], (*pose_)[1]};
    const double start_error = distance(pose_xy, *path_start_);
    if (start_error > path_start_tolerance_) {
      interrupt_and_publish("PATH_START_MISMATCH distance=" + format(start_error, 2) + "m");
      return;
    }
    const auto selected_lookahead = safe_lookahead(pose_xy);
    if (!selected_lookahead) {
      follower_->hold_command();
      const bool pose_safe = command_has_clearance(pose_xy, pose_xy);
      publish_status(
        std::string("CLEARANCE_BLOCKED reason=") +
        (pose_safe ? "NO_SAFE_LOOKAHEAD" : "POSE_INSIDE_CLEARANCE") +
        " required=" + format(required_clearance_, 2) + "m");
      return;
    }

    const auto result = follower_->update(
      pose_xy, dt, selected_lookahead,
      [this, &pose_xy](const Point2 & carrot) {
        return command_has_clearance(pose_xy, carrot);
      });
    const builtin_interfaces::msg::Time stamp = get_clock()->now();
    lookahead_pub_->publish(pose_message(result.desired_carrot, (*pose_)[2], stamp));
    carrot_pub_->publish(pose_message(result.commanded_carrot, (*pose_)[2], stamp));
    geometry_msgs::msg::Vector3Stamped displacement;
    displacement.header.stamp = stamp;
    displacement.header.frame_id = frame_id_;
    displacement.vector.x = result.commanded_displacement.first;
    displacement.vector.y = result.commanded_displacement.second;
    displacement_pub_->publish(displacement);
    const auto vio = map_displacement_to_vio(result.commanded_displacement, (*correction_)[3]);
    geometry_msgs::msg::Vector3Stamped vio_displacement;
    vio_displacement.header.stamp = stamp;
    vio_displacement.header.frame_id = "vio";
    vio_displacement.vector.x = vio.first;
    vio_displacement.vector.y = vio.second;
    vio_displacement_pub_->publish(vio_displacement);
    std_msgs::msg::Float32 scalar;
    scalar.data = static_cast<float>(result.progress); progress_pub_->publish(scalar);
    scalar.data = static_cast<float>(result.path_progress); path_progress_pub_->publish(scalar);
    scalar.data = static_cast<float>(result.remaining); remaining_pub_->publish(scalar);
    scalar.data = static_cast<float>(result.cross_track); cross_track_pub_->publish(scalar);
    std_msgs::msg::Int32 generation;
    generation.data = result.generation;
    generation_pub_->publish(generation);
    publish_status(
      result.status + " generation=" + std::to_string(result.generation) +
      " progress=" + format(result.progress, 2) + "m path_progress=" +
      format(result.path_progress, 2) + "m remaining=" + format(result.remaining, 2) +
      "m cross_track=" + format(result.cross_track, 2) + "m lookahead=" +
      format(*selected_lookahead, 2) + "/" + format(lookahead_, 2) + "m",
      result.valid, requested_goal_reached(result.status, goal_terminal_));
    publish_markers(result, stamp);
  }

  std::string path_topic_, map_topic_, pose_topic_, raw_vio_topic_, goal_topic_;
  std::string goal_terminal_topic_, correction_topic_, frame_id_;
  double rate_hz_{}, path_timeout_{}, map_timeout_{}, pose_timeout_{}, vio_timeout_{};
  double correction_timeout_{}, max_correction_m_{}, max_correction_yaw_deg_{};
  double lookahead_{}, lookahead_step_{}, min_lookahead_{};
  int occupied_threshold_{};
  double robot_radius_{}, safety_margin_{}, required_clearance_{};
  double max_carrot_speed_{}, max_carrot_acceleration_{}, max_cross_track_{};
  double cross_track_resume_{}, cross_track_recovery_time_{}, path_start_tolerance_{};
  double arrival_tolerance_{}, arrival_release_tolerance_{};
  double correction_translation_trigger_{}, correction_yaw_trigger_deg_{};
  double correction_filter_time_constant_{}, correction_material_translation_{};
  double correction_material_yaw_deg_{}, correction_settle_time_{}, correction_cooldown_{};
  std::unique_ptr<CorrectionReplanGate> correction_gate_;
  std::unique_ptr<PositionRouteFollower> follower_;

  std::optional<std::array<double, 3>> pose_;
  double pose_received_{};
  bool raw_vio_seen_{false}, raw_vio_valid_{false};
  std::string raw_vio_reason_;
  double raw_vio_received_{};
  bool correction_seen_{false}, correction_valid_{false};
  std::string correction_reason_;
  double correction_received_{};
  std::optional<Correction4> correction_;
  bool goal_terminal_{false};
  double path_received_{};
  std::optional<Point2> path_start_;
  std::optional<GridMap> grid_;
  double map_received_{};
  double last_tick_{};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, raw_vio_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_terminal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr correction_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_, lookahead_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    displacement_pub_, vio_displacement_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, config_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_, goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    progress_pub_, path_progress_pub_, remaining_pub_, cross_track_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr generation_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  try {
    px4_vio_bridge::ProcessSingleton singleton("route_follower_monitor");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_vio_bridge::RouteFollowerNode>());
    rclcpp::shutdown();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "FATAL: %s\n", error.what());
    return 1;
  }
  return 0;
}
