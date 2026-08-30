// C++ counterpart of px4_vio_bridge/global_planner_monitor.py.
//
// Observation-only: publishes no PX4 topics. Selected at launch by the
// cpp_nodes / cpp_astar toggle; it takes the same `global_planner_monitor`
// singleton lock as the Python node so the two can never both drive
// /planner/path in one ROS domain.

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "px4_vio_bridge/grid_planner.hpp"
#include "px4_vio_bridge/path_geometry.hpp"

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

// Same lock file, same role name, same domain as process_singleton.py, so a
// stray Python planner blocks this one and vice versa.
class ProcessSingleton
{
public:
  explicit ProcessSingleton(const std::string & role)
  {
    const auto * domain_env = std::getenv("ROS_DOMAIN_ID");
    const std::string domain = domain_env != nullptr ? domain_env : "0";
    const auto sanitise = [](std::string text) {
        for (auto & character : text) {
          const auto safe = std::isalnum(static_cast<unsigned char>(character)) != 0 ||
            character == '_' || character == '.' || character == '-';
          if (!safe) {
            character = '_';
          }
        }
        return text;
      };
    path_ = "/tmp/px4_vio_bridge_" + sanitise(role) + "_ros_domain_" + sanitise(domain) + ".lock";
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
              "duplicate " + role + " for ROS_DOMAIN_ID=" + domain + "; existing holder PID=" +
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

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_monitor")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/rtabmap/grid");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/rtabmap/body_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/waypoint/clicked");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const auto rate_hz = declare_parameter<double>("rate_hz", 2.0);
    map_timeout_ = declare_parameter<double>("map_timeout", 3.0);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 1.0);
    occupied_threshold_ = static_cast<int>(declare_parameter<int64_t>("occupied_threshold", 65));
    const auto robot_radius = declare_parameter<double>("robot_radius", 0.25);
    const auto safety_margin = declare_parameter<double>("safety_margin", 0.05);
    const auto inflation_extra = declare_parameter<double>("inflation_extra", 0.20);
    inflation_cost_scaling_ = declare_parameter<double>("inflation_cost_scaling", 3.0);
    start_recovery_radius_ = std::max(
      0.0, declare_parameter<double>("start_recovery_radius", 0.30));
    heuristic_weight_ = declare_parameter<double>("heuristic_weight", 1.0);
    cost_weight_ = declare_parameter<double>("cost_weight", 2.0);
    planning_timeout_ms_ = declare_parameter<double>("planning_timeout_ms", 100.0);
    switch_improvement_ = declare_parameter<double>("switch_improvement", 0.10);
    // Keep below the follower's max_cross_track so the planner gives up on a
    // corridor before the follower faults on it.
    path_retain_tolerance_ = std::max(
      0.0, declare_parameter<double>("path_retain_tolerance", 0.35));
    // Must stay under the follower's path_start_tolerance, which rejects a path
    // that starts too far away.
    path_head_margin_ = std::max(0.0, declare_parameter<double>("path_head_margin", 0.50));
    correction_topic_ =
      declare_parameter<std::string>("correction_topic", "/rtabmap/odom_correction");
    correction_timeout_ = declare_parameter<double>("correction_timeout", 1.0);
    max_correction_m_ = declare_parameter<double>("max_correction_m", 0.50);
    max_correction_yaw_deg_ = declare_parameter<double>("max_correction_yaw_deg", 15.0);
    // Distinct occupancy grids -- never planner ticks -- that must agree before
    // a semantic mode change is committed. At ~1 Hz maps, 2 commits inside the
    // flight adapter's 3 s planner-fault land timer.
    mode_confirmation_maps_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("mode_confirmation_maps", 2)));
    mode_ = GoalModeHysteresis(mode_confirmation_maps_);
    lethal_radius_ = robot_radius + safety_margin;
    inflation_radius_ = lethal_radius_ + inflation_extra;

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable().transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {on_map(*msg);});
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {on_pose(*msg);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {on_goal(*msg);});
    // The accepted route is stored in continuous VIO coordinates, so the
    // planner needs the same map<-vio transform the follower validates.
    correction_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      correction_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {on_correction(*msg);});

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/path", 10);
    candidate_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/candidate_path", 10);
    inflated_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/planner/inflated_map", map_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planner/markers", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planner/status", 10);
    planning_ms_pub_ = create_publisher<std_msgs::msg::Float32>("/planner/planning_ms", 10);
    path_length_pub_ = create_publisher<std_msgs::msg::Float32>("/planner/path_length", 10);
    expanded_pub_ = create_publisher<std_msgs::msg::Int32>("/planner/expanded_cells", 10);
    goal_exact_pub_ = create_publisher<std_msgs::msg::Bool>("/planner/goal_exact", 10);
    goal_terminal_pub_ = create_publisher<std_msgs::msg::Bool>("/planner/goal_terminal", 10);
    effective_goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/planner/effective_goal", 10);
    // Latched generation telemetry. The follower pairs these with its own
    // correction episodes so a path planned from the pre-correction grid can
    // never release a post-correction hold.
    map_generation_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/planner/map_generation", map_qos);
    path_map_generation_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/planner/path_map_generation", map_qos);
    // TRANSIENT_LOCAL so a recorder started after this long-running node still
    // captures the effective launch overrides -- every bag stays self-describing.
    config_pub_ = create_publisher<std_msgs::msg::String>("/planner/config", map_qos);
    publish_config();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.2, rate_hz)), [this] {tick();});
    RCLCPP_WARN(
      get_logger(),
      "global_planner_monitor (C++): OBSERVATION ONLY; publishes no PX4 commands. "
      "continuous_clearance=%.2fm unknown=blocked", lethal_radius_);
  }

private:
  void publish_config()
  {
    nlohmann::ordered_json config;
    config["correction_timeout"] = correction_timeout_;
    config["correction_topic"] = correction_topic_;
    config["cost_weight"] = cost_weight_;
    config["frame_id"] = frame_id_;
    config["goal_topic"] = goal_topic_;
    config["heuristic_weight"] = heuristic_weight_;
    config["implementation"] = "cpp";
    config["inflation_cost_scaling"] = inflation_cost_scaling_;
    config["inflation_extra"] = get_parameter("inflation_extra").as_double();
    config["inflation_radius"] = inflation_radius_;
    config["lethal_radius"] = lethal_radius_;
    config["map_timeout"] = map_timeout_;
    config["map_topic"] = map_topic_;
    config["max_correction_m"] = max_correction_m_;
    config["max_correction_yaw_deg"] = max_correction_yaw_deg_;
    config["mode_confirmation_maps"] = mode_confirmation_maps_;
    config["occupied_threshold"] = occupied_threshold_;
    config["path_head_margin"] = path_head_margin_;
    config["path_retain_tolerance"] = path_retain_tolerance_;
    config["planning_timeout_ms"] = planning_timeout_ms_;
    config["pose_timeout"] = pose_timeout_;
    config["pose_topic"] = pose_topic_;
    config["rate_hz"] = get_parameter("rate_hz").as_double();
    config["robot_radius"] = get_parameter("robot_radius").as_double();
    config["safety_margin"] = get_parameter("safety_margin").as_double();
    config["start_recovery_radius"] = start_recovery_radius_;
    config["switch_improvement"] = switch_improvement_;
    std_msgs::msg::String message;
    message.data = config.dump();
    config_pub_->publish(message);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_pub_->publish(message);
  }

  void on_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    if (msg.header.frame_id != frame_id_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "map rejected: frame '%s' != '%s'", msg.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    const auto & q = msg.info.origin.orientation;
    if (std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z) > 1e-6 ||
      std::abs(q.w - 1.0) > 1e-6)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "map rejected: rotated occupancy-grid origins are unsupported");
      return;
    }
    if (msg.info.width < 1 || msg.info.height < 1 || msg.info.resolution <= 0.0 ||
      msg.data.size() != static_cast<std::size_t>(msg.info.width) * msg.info.height)
    {
      RCLCPP_ERROR(get_logger(), "map rejected: invalid grid geometry");
      return;
    }
    for (const auto value : msg.data) {
      if (value < -1 || value > 100) {
        RCLCPP_ERROR(get_logger(), "map rejected: values must be -1 or 0..100");
        return;
      }
    }
    grid_ = GridMap{
      msg.info.width, msg.info.height, msg.info.resolution,
      msg.info.origin.position.x, msg.info.origin.position.y,
      std::vector<std::int8_t>(msg.data.begin(), msg.data.end())};
    grid_info_ = msg.info;
    ++grid_generation_;
    map_received_ = monotonic_seconds();
    std_msgs::msg::Int32 generation;
    generation.data = static_cast<std::int32_t>(grid_generation_);
    map_generation_pub_->publish(generation);
  }

  void on_pose(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (msg.header.frame_id != frame_id_) {
      return;
    }
    pose_ = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
    pose_received_ = monotonic_seconds();
  }

  void on_goal(const geometry_msgs::msg::PointStamped & msg)
  {
    if (msg.header.frame_id != frame_id_) {
      RCLCPP_WARN(
        get_logger(), "goal rejected: frame '%s' != '%s'",
        msg.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    if (!std::isfinite(msg.point.x) || !std::isfinite(msg.point.y)) {
      RCLCPP_WARN(get_logger(), "goal rejected: coordinates must be finite");
      return;
    }
    goal_ = Point2{msg.point.x, msg.point.y};
    ++goal_generation_;
    // A new requested goal carries no old semantic commitment.
    mode_.reset();
    RCLCPP_WARN(get_logger(), "planner goal=(%.2f, %.2f)", goal_->first, goal_->second);
  }

  void on_correction(const geometry_msgs::msg::PoseStamped & msg)
  {
    const auto & p = msg.pose.position;
    const auto & q = msg.pose.orientation;
    const Correction4 correction{
      p.x, p.y, p.z, yaw_from_quaternion(q.w, q.x, q.y, q.z)};
    const auto reason = correction_rejection_reason(
      correction, max_correction_m_, max_correction_yaw_deg_ * kPi / 180.0);
    correction_seen_ = true;
    correction_received_ = monotonic_seconds();
    correction_valid_ = !reason.has_value();
    correction_reason_ = reason.value_or("");
    if (reason) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "native correction rejected: %s", reason->c_str());
      return;
    }
    correction_ = correction;
  }

  // The accepted route lives in continuous VIO coordinates. Rendering it into
  // the map solution paired with the current grid is what stops a 5 cm loop
  // closure from looking like a path-replacement or off-path error.
  [[nodiscard]] std::vector<Point2> accepted_in_map() const
  {
    std::vector<Point2> points;
    if (!correction_) {
      return points;
    }
    points.reserve(accepted_vio_points_.size());
    for (const auto & point : accepted_vio_points_) {
      points.push_back(vio_point_to_map(point, *correction_));
    }
    return points;
  }

  void store_accepted(const std::vector<Point2> & points)
  {
    accepted_vio_points_.clear();
    accepted_vio_points_.reserve(points.size());
    for (const auto & point : points) {
      accepted_vio_points_.push_back(map_point_to_vio(point, *correction_));
    }
  }

  void publish_path_map_generation()
  {
    std_msgs::msg::Int32 message;
    message.data = static_cast<std::int32_t>(path_map_generation_);
    path_map_generation_pub_->publish(message);
  }

  void publish_goal_flags(bool raw_exact, bool raw_terminal)
  {
    std_msgs::msg::Bool flag;
    // Committed exact state. An exploration endpoint is never labelled exact
    // just because PATH_VALID is still the committed mode.
    flag.data = raw_exact && mode_.initialized() && mode_.stable() == GoalMode::PathValid;
    goal_exact_pub_->publish(flag);
    // Conservative completion permission: false the instant a raw result is
    // nonterminal, true only once the terminal mode is committed.
    flag.data = raw_terminal && mode_.initialized() && goal_mode_terminal(mode_.stable());
    goal_terminal_pub_->publish(flag);
  }

  void ensure_inflated()
  {
    if (inflated_generation_ == grid_generation_) {
      return;
    }
    inflated_ = inflate_occupancy(
      *grid_, occupied_threshold_,
      grid_lethal_radius(lethal_radius_, grid_->resolution),
      grid_lethal_radius(inflation_radius_, grid_->resolution),
      inflation_cost_scaling_);
    inflated_generation_ = grid_generation_;
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.info = grid_info_;
    message.info.map_load_time = message.header.stamp;
    message.data = inflation_display_data(*grid_, *inflated_, occupied_threshold_);
    inflated_pub_->publish(message);
  }

  bool path_valid(const std::vector<Point2> & points) const
  {
    if (points.empty()) {
      return false;
    }
    std::vector<Cell> cells;
    cells.reserve(points.size());
    for (const auto & point : points) {
      const auto cell = inflated_->world_to_cell(point);
      if (!cell.has_value()) {
        return false;
      }
      cells.push_back(*cell);
    }
    if (points.size() == 1) {
      return segment_has_clearance(
        *grid_, points[0], points[0], lethal_radius_, occupied_threshold_);
    }
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      if (!line_is_clear(*inflated_, cells[i], cells[i + 1]) ||
        !segment_has_clearance(
          *grid_, points[i], points[i + 1], lethal_radius_, occupied_threshold_))
      {
        return false;
      }
    }
    return true;
  }

  nav_msgs::msg::Path make_path(const std::vector<Point2> & points, double z) const
  {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.first;
      pose.pose.position.y = point.second;
      pose.pose.position.z = z;
      pose.pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }
    return message;
  }

  void clear_path(const std::string & status, const std::optional<Point2> & effective_goal)
  {
    accepted_vio_points_.clear();
    accepted_effective_goal_vio_.reset();
    path_map_generation_ = grid_generation_;
    const auto empty = make_path({}, pose_.has_value() ? (*pose_)[2] : 0.0);
    candidate_pub_->publish(empty);
    path_pub_->publish(empty);
    publish_path_map_generation();
    std_msgs::msg::Bool flag;
    flag.data = false;
    goal_exact_pub_->publish(flag);
    goal_terminal_pub_->publish(flag);
    publish_status(status);
    publish_markers(status, effective_goal);
  }

  void publish_markers(const std::string & status, const std::optional<Point2> & effective_goal)
  {
    const auto stamp = now();
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const auto sphere = [&](int id, const std::string & ns, double x, double y, double z,
        double r, double g, double b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.16;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        return marker;
      };

    const auto z = pose_.has_value() ? (*pose_)[2] : 0.0;
    if (pose_.has_value()) {
      markers.markers.push_back(
        sphere(0, "planner_start", (*pose_)[0], (*pose_)[1], (*pose_)[2], 0.1, 1.0, 0.1));
    }
    if (goal_.has_value()) {
      markers.markers.push_back(
        sphere(1, "planner_goal", goal_->first, goal_->second, z, 1.0, 1.0, 1.0));
    }
    if (effective_goal.has_value()) {
      markers.markers.push_back(
        sphere(
          3, "planner_effective_goal", effective_goal->first, effective_goal->second, z,
          1.0, 0.5, 0.0));
    }
    visualization_msgs::msg::Marker text;
    text.header.frame_id = frame_id_;
    text.header.stamp = stamp;
    text.ns = "planner_status";
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    if (pose_.has_value()) {
      text.pose.position.x = (*pose_)[0];
      text.pose.position.y = (*pose_)[1];
      text.pose.position.z = (*pose_)[2] + 0.35;
    }
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.12;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
    text.text = status;
    markers.markers.push_back(text);
    markers_pub_->publish(markers);
  }

  void publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher, double value)
  {
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(value);
    publisher->publish(message);
  }

  void tick()
  {
    const auto now_s = monotonic_seconds();
    if (!grid_.has_value()) {
      publish_status("WAITING_FOR_MAP");
      return;
    }
    if (now_s - map_received_ > map_timeout_) {
      publish_status("STALE_MAP age=" + format(now_s - map_received_, 2) + "s");
      return;
    }
    // Inflation is a property of the map, not of a requested route: keep its
    // visualisation current before a goal exists, so clicking in Foxglove does
    // not appear to turn nearby free space into obstacles.
    ensure_inflated();
    if (!pose_.has_value() || now_s - pose_received_ > pose_timeout_) {
      publish_status("WAITING_FOR_POSE");
      return;
    }
    // Fail closed: without a valid, fresh correction the accepted route cannot
    // be expressed in the current map solution at all. Publishing nothing lets
    // the follower go stale and the flight adapter hold, then land.
    if (!correction_seen_) {
      publish_status("WAITING_FOR_CORRECTION");
      return;
    }
    if (!correction_valid_) {
      publish_status("CORRECTION_REJECTED reason=" + correction_reason_);
      return;
    }
    if (now_s - correction_received_ > correction_timeout_) {
      publish_status("STALE_CORRECTION age=" + format(now_s - correction_received_, 2) + "s");
      return;
    }
    if (!goal_.has_value()) {
      publish_status("WAITING_FOR_GOAL");
      return;
    }
    const Point2 pose_xy{(*pose_)[0], (*pose_)[1]};
    auto start_cell = inflated_->world_to_cell(pose_xy);
    if (!start_cell.has_value()) {
      clear_path("START_OUTSIDE_MAP", std::nullopt);
      return;
    }
    const auto planning_started = monotonic_seconds();
    // Brushing past an obstacle puts the vehicle inside its own lethal
    // inflation. Dropping the route for that would arm the flight adapter's
    // land timer while the aircraft is merely close to something, so shift the
    // search start to the nearest cell outside the envelope instead.
    double start_offset = 0.0;
    if (!traversable(*inflated_, *start_cell)) {
      const auto recovery = recover_start(
        *grid_, *inflated_, *start_cell, occupied_threshold_, start_recovery_radius_);
      if (!recovery.has_value()) {
        clear_path("START_BLOCKED", std::nullopt);
        return;
      }
      start_cell = recovery->cell;
      start_offset = recovery->distance;
    }
    const auto selection = closest_reachable_goal(*inflated_, *start_cell, *goal_);
    if (!selection.has_value()) {
      clear_path("START_BLOCKED", std::nullopt);
      return;
    }
    const auto goal_cell = selection->cell;
    const auto [exact, terminal] = classify_goal(*grid_, *inflated_, *goal_, goal_cell);
    const auto raw_mode = goal_mode_from(exact, terminal);
    // Counted against distinct occupancy grids, so repeated planner ticks on
    // one map confirm nothing.
    auto decision = mode_.observe(raw_mode, grid_generation_);
    publish_goal_flags(exact, terminal);
    // The candidate's endpoint. /planner/effective_goal is published further
    // down with the endpoint of the route actually accepted, which during a
    // pending mode transition is deliberately not this one.
    const auto effective_goal = inflated_->cell_center(goal_cell);

    const PlanKey key{grid_generation_, goal_generation_, *start_cell, goal_cell};
    if (last_planned_key_.has_value() && *last_planned_key_ == key) {
      return;
    }
    last_planned_key_ = key;

    const auto selection_ms = (monotonic_seconds() - planning_started) * 1000.0;
    const auto remaining_ms = planning_timeout_ms_ - selection_ms;
    if (planning_timeout_ms_ > 0.0 && remaining_ms <= 0.0) {
      publish_float(planning_ms_pub_, selection_ms);
      std_msgs::msg::Int32 expanded;
      expanded.data = selection->reachable_cells;
      expanded_pub_->publish(expanded);
      clear_path(
        "TIMEOUT reachable=" + std::to_string(selection->reachable_cells) +
        " plan=" + format(selection_ms, 1) + "ms", effective_goal);
      return;
    }

    const auto result = astar(
      *inflated_, *start_cell, goal_cell, heuristic_weight_, cost_weight_,
      std::max(0.0, remaining_ms));
    const auto total_ms = (monotonic_seconds() - planning_started) * 1000.0;
    publish_float(planning_ms_pub_, total_ms);
    std_msgs::msg::Int32 expanded_message;
    expanded_message.data = result.expanded;
    expanded_pub_->publish(expanded_message);
    if (!result.found()) {
      accepted_vio_points_.clear();
      accepted_effective_goal_vio_.reset();
      path_map_generation_ = grid_generation_;
      path_pub_->publish(make_path({}, (*pose_)[2]));
      publish_path_map_generation();
      const auto status = result.reason + " expanded=" + std::to_string(result.expanded) +
        " plan=" + format(total_ms, 1) + "ms";
      publish_status(status);
      publish_markers(status, effective_goal);
      return;
    }

    const auto candidate_cells = simplify_path(
      *inflated_, result.cells, true, &(*grid_), lethal_radius_, occupied_threshold_);
    if (candidate_cells.empty()) {
      clear_path("UNSAFE_SEGMENT continuous clearance validation failed", effective_goal);
      return;
    }
    std::vector<Point2> candidate_points;
    candidate_points.reserve(candidate_cells.size());
    for (const auto & cell : candidate_cells) {
      candidate_points.push_back(inflated_->cell_center(cell));
    }
    const auto candidate_length = path_length(candidate_points);
    candidate_pub_->publish(make_path(candidate_points, (*pose_)[2]));

    // Everything below compares the retained route and the fresh candidate in
    // one frame: the map solution this grid belongs to.
    auto accepted_points = accepted_in_map();
    // Mode debounce is not collision debounce. The retained route is
    // revalidated against every new raw grid before it is allowed to survive.
    const bool had_retained_route = !accepted_points.empty();
    const bool retained_safe = path_valid(accepted_points);
    const auto projection = retained_safe
      ? path_projection(accepted_points, pose_xy)
      : std::nullopt;
    const auto goal_changed = accepted_goal_generation_ != goal_generation_;
    // Re-derive the accepted endpoint's cell in *this* grid. Comparing stored
    // indices across grids with different origins or dimensions is meaningless.
    std::optional<Cell> accepted_goal_cell;
    if (accepted_effective_goal_vio_.has_value()) {
      accepted_goal_cell = inflated_->world_to_cell(
        vio_point_to_map(*accepted_effective_goal_vio_, *correction_));
    }
    const auto effective_goal_changed =
      !accepted_goal_cell.has_value() || *accepted_goal_cell != goal_cell;
    PathReplacementInputs inputs;
    inputs.mode_transition_pending = decision.has_pending;
    inputs.retained_safe = retained_safe;
    inputs.goal_changed = goal_changed;
    inputs.effective_goal_changed = effective_goal_changed;
    inputs.projection = projection;
    inputs.candidate_length = candidate_length;
    inputs.retain_tolerance = path_retain_tolerance_;
    inputs.switch_improvement = switch_improvement_;
    const auto replacement = decide_path_replacement(inputs);
    Point2 accepted_effective_goal = effective_goal;
    if (replacement.replace) {
      accepted_points = candidate_points;
      accepted_goal_generation_ = goal_generation_;
      ++accepted_generation_;
      // The old route is gone, so there is no older meaning left to protect.
      mode_.commit(raw_mode);
      decision = mode_.decision();
      publish_goal_flags(exact, terminal);
    } else {
      accepted_points = trim_path_to(accepted_points, pose_xy, path_head_margin_);
      if (accepted_goal_cell.has_value()) {
        accepted_effective_goal = inflated_->cell_center(*accepted_goal_cell);
      }
    }
    store_accepted(accepted_points);
    accepted_effective_goal_vio_ = map_point_to_vio(accepted_effective_goal, *correction_);
    path_map_generation_ = grid_generation_;
    path_pub_->publish(make_path(accepted_points, (*pose_)[2]));
    publish_path_map_generation();
    // The endpoint of the route actually published, not of the newest candidate.
    geometry_msgs::msg::PointStamped accepted_goal_message;
    accepted_goal_message.header.stamp = now();
    accepted_goal_message.header.frame_id = frame_id_;
    accepted_goal_message.point.x = accepted_effective_goal.first;
    accepted_goal_message.point.y = accepted_effective_goal.second;
    accepted_goal_message.point.z = (*pose_)[2];
    effective_goal_pub_->publish(accepted_goal_message);
    const auto accepted_length = path_length(accepted_points);
    publish_float(path_length_pub_, accepted_length);

    const std::string mode = to_string(decision.stable);
    auto status = mode + mode_.pending_suffix() + " length=" + format(accepted_length, 2) +
      "m candidate=" + format(candidate_length, 2) +
      "m goal_distance=" + format(selection->distance, 2) +
      "m reachable=" + std::to_string(selection->reachable_cells) +
      " expanded=" + std::to_string(result.expanded) +
      " plan=" + format(total_ms, 1) +
      "ms map_age=" + format(now_s - map_received_, 2) +
      "s path_gen=" + std::to_string(accepted_generation_) +
      " map_gen=" + std::to_string(grid_generation_) +
      " raw_mode=" + to_string(raw_mode);
    if (had_retained_route && !retained_safe) {
      status += " retained_unsafe=1";
    }
    if (replacement.transition_hold) {
      status += " retained_through_pending_mode=1";
    }
    if (projection.has_value()) {
      status += " off_path=" + format(projection->distance, 2) + "m";
    }
    if (start_offset > 0.0) {
      status += " start_recovered=" + format(start_offset, 2) + "m";
    }
    publish_status(status);
    publish_markers(status, accepted_effective_goal);
  }

  struct PlanKey
  {
    long grid_generation;
    long goal_generation;
    Cell start;
    Cell goal;
    bool operator==(const PlanKey & other) const
    {
      return grid_generation == other.grid_generation &&
             goal_generation == other.goal_generation &&
             start == other.start && goal == other.goal;
    }
  };

  std::string map_topic_, pose_topic_, goal_topic_, frame_id_;
  double map_timeout_{}, pose_timeout_{}, lethal_radius_{}, inflation_radius_{};
  double inflation_cost_scaling_{}, start_recovery_radius_{}, heuristic_weight_{};
  double cost_weight_{}, planning_timeout_ms_{}, switch_improvement_{};
  double path_retain_tolerance_{}, path_head_margin_{};
  std::string correction_topic_;
  double correction_timeout_{}, max_correction_m_{}, max_correction_yaw_deg_{};
  int occupied_threshold_{65};
  int mode_confirmation_maps_{2};
  GoalModeHysteresis mode_{2};

  std::optional<GridMap> grid_;
  nav_msgs::msg::MapMetaData grid_info_;
  std::optional<CostGrid> inflated_;
  long grid_generation_{0};
  long inflated_generation_{-1};
  std::optional<std::array<double, 3>> pose_;
  std::optional<Point2> goal_;
  long goal_generation_{0};
  double map_received_{0.0};
  double pose_received_{0.0};
  // Canonical continuous-VIO storage: a pure coordinate re-expression is not a
  // new path and never resets progress.
  std::vector<Point2> accepted_vio_points_;
  long accepted_goal_generation_{-1};
  std::optional<Point2> accepted_effective_goal_vio_;
  long accepted_generation_{0};
  long path_map_generation_{0};
  bool correction_seen_{false}, correction_valid_{false};
  std::string correction_reason_;
  double correction_received_{0.0};
  std::optional<Correction4> correction_;
  std::optional<PlanKey> last_planned_key_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr correction_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, candidate_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, config_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr planning_ms_pub_, path_length_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr expanded_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
    map_generation_pub_, path_map_generation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_exact_pub_, goal_terminal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr effective_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  std::unique_ptr<px4_vio_bridge::ProcessSingleton> singleton;
  try {
    singleton = std::make_unique<px4_vio_bridge::ProcessSingleton>("global_planner_monitor");
  } catch (const std::exception & error) {
    std::fprintf(stderr, "FATAL: %s\n", error.what());
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
