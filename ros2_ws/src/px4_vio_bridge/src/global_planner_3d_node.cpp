// Observation-only OctoMap-backed 3D A* planner.
//
// This executable intentionally has no px4_msgs dependency and publishes no
// /fmu/in topics. It is the first acceptance-gate implementation described in
// HANDOFF_3D_NAVIGATION.md, not flight authority.

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "px4_vio_bridge/grid_planner_3d.hpp"
#include "px4_vio_bridge/flight_geometry_3d.hpp"
#include "px4_vio_bridge/octomap_grid.hpp"
#include "px4_vio_bridge/navigation_snapshot.hpp"

namespace px4_vio_bridge
{
namespace
{

double steady_seconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

bool finite(const Point3 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

struct MapMetadata
{
  std::int64_t generation{};
  std::int64_t stamp_ns{};
  double resolution{};
  std::string frame_id;
  std::string source_pose_generation;
  Pose3 map_from_odom;
};

struct GeofenceMask
{
  std::size_t width{};
  std::size_t height{};
  double resolution{};
  Point3 origin;
  Point3 center;
  double radius{};
  std::vector<std::uint8_t> inside;

  [[nodiscard]] bool matches(const VoxelGrid & grid, const Point3 & requested_center,
    double requested_radius) const
  {
    return width == grid.width() && height == grid.height() &&
      resolution == grid.resolution() && origin.x == grid.origin().x &&
      origin.y == grid.origin().y && center.x == requested_center.x &&
      center.y == requested_center.y && radius == requested_radius;
  }
};

std::shared_ptr<const GeofenceMask> make_geofence_mask(
  const VoxelGrid & grid, const Point3 & center, double radius)
{
  auto mask = std::make_shared<GeofenceMask>();
  mask->width = grid.width();
  mask->height = grid.height();
  mask->resolution = grid.resolution();
  mask->origin = grid.origin();
  mask->center = center;
  mask->radius = radius;
  mask->inside.resize(grid.width() * grid.height());
  const long double radius_squared = static_cast<long double>(radius) * radius;
  for (std::size_t y = 0; y < grid.height(); ++y) {
    const long double dy = static_cast<long double>(grid.origin().y) +
      (static_cast<long double>(y) + 0.5L) * grid.resolution() - center.y;
    for (std::size_t x = 0; x < grid.width(); ++x) {
      const long double dx = static_cast<long double>(grid.origin().x) +
        (static_cast<long double>(x) + 0.5L) * grid.resolution() - center.x;
      mask->inside[y * grid.width() + x] =
        dx * dx + dy * dy <= radius_squared ? 1U : 0U;
    }
  }
  return mask;
}

class ProcessSingleton
{
public:
  explicit ProcessSingleton(const std::string & role)
  {
    const auto * domain_value = std::getenv("ROS_DOMAIN_ID");
    std::string domain = domain_value == nullptr ? "0" : domain_value;
    for (auto & character : domain) {
      if (!std::isalnum(static_cast<unsigned char>(character)) && character != '_' &&
        character != '-' && character != '.')
      {
        character = '_';
      }
    }
    path_ = "/tmp/px4_vio_bridge_" + role + "_ros_domain_" + domain + ".lock";
    descriptor_ = ::open(path_.c_str(), O_RDWR | O_CREAT, 0644);
    if (descriptor_ < 0 || ::flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
      throw std::runtime_error("another 3D planner authority holds " + path_);
    }
    if (::ftruncate(descriptor_, 0) != 0) {
      throw std::runtime_error("cannot initialise singleton " + path_);
    }
    const auto pid = std::to_string(::getpid());
    if (::write(descriptor_, pid.data(), pid.size()) < 0) {
      throw std::runtime_error("cannot write singleton " + path_);
    }
  }

  ~ProcessSingleton()
  {
    if (descriptor_ >= 0) {
      ::flock(descriptor_, LOCK_UN);
      ::close(descriptor_);
    }
  }

private:
  std::string path_;
  int descriptor_{-1};
};

}  // namespace

class GlobalPlanner3DNode final : public rclcpp::Node
{
public:
  GlobalPlanner3DNode()
  : Node("global_planner_3d_monitor"), singleton_("planner3d_authority")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/rtabmap/octomap");
    map_metadata_topic_ = declare_parameter<std::string>(
      "map_metadata_topic", "/rtabmap/octomap_metadata");
    require_map_metadata_ = declare_parameter<bool>("require_map_metadata", true);
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/rtabmap/pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/waypoint/clicked");
    goal_height_mode_ = declare_parameter<std::string>("goal_height_mode", "clicked");
    current_goal_min_height_above_start_ =
      declare_parameter<double>("current_goal_min_height_above_start", 0.0);
    exploration_route_ = declare_parameter<bool>("exploration_route", false);
    if (goal_height_mode_ != "clicked" && goal_height_mode_ != "current") {
      throw std::invalid_argument("goal_height_mode must be clicked or current");
    }
    if (!std::isfinite(current_goal_min_height_above_start_) ||
      current_goal_min_height_above_start_ < 0.0)
    {
      throw std::invalid_argument("current_goal_min_height_above_start must be finite and nonnegative");
    }
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.25);
    safety_margin_ = declare_parameter<double>("safety_margin", 0.10);
    max_cross_track_ = declare_parameter<double>("max_cross_track", 0.05);
    inflation_extra_ = declare_parameter<double>("inflation_extra", 0.20);
    planning_radius_xy_ = declare_parameter<double>("planning_radius_xy", 3.0);
    min_z_ = declare_parameter<double>("min_z", 0.20);
    max_z_ = declare_parameter<double>("max_z", 2.00);
    map_timeout_ = declare_parameter<double>("map_timeout", 3.0);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 1.0);
    map_preparation_period_ = declare_parameter<double>("map_preparation_period", 2.0);
    planning_pose_drift_tolerance_ =
      declare_parameter<double>("planning_pose_drift_tolerance", 0.10);
    max_marker_voxels_ = static_cast<std::size_t>(
      std::max<int64_t>(0, declare_parameter<int64_t>("max_marker_voxels", 20000)));
    config_.lethal_radius = robot_radius_ + safety_margin_;
    config_.inflation_radius = config_.lethal_radius + inflation_extra_;
    config_.cost_scaling = declare_parameter<double>("inflation_cost_scaling", 3.0);
    config_.heuristic_weight = declare_parameter<double>("heuristic_weight", 1.0);
    config_.cost_weight = declare_parameter<double>("cost_weight", 2.0);
    config_.timeout_ms = declare_parameter<double>("planning_timeout_ms", 150.0);
    config_.start_recovery_radius = declare_parameter<double>("start_recovery_radius", 0.20);
    const auto rate_hz = declare_parameter<double>("planning_rate_hz", 2.0);
    validate_config();

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      map_topic_, map_qos,
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr message) {on_map(std::move(message));});
    metadata_sub_ = create_subscription<std_msgs::msg::String>(
      map_metadata_topic_, map_qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_metadata(*message);});
    snapshot_topic_ = declare_parameter<std::string>("pose_snapshot_topic", "");
    if (!snapshot_topic_.empty()) {
      snapshot_sub_ = create_subscription<std_msgs::msg::String>(snapshot_topic_, 10,
        [this](std_msgs::msg::String::ConstSharedPtr message) {on_snapshot(*message);});
    } else {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {on_pose(*message);});
    }
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {on_goal(*message);});

    path_snapshot_pub_ = create_publisher<std_msgs::msg::String>("/planner3d/path_snapshot", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner3d/path", 10);
    candidate_pub_ = create_publisher<nav_msgs::msg::Path>("/planner3d/candidate_path", 10);
    reference_pub_ = create_publisher<nav_msgs::msg::Path>("/planner3d/reference_path", 10);
    reference_snapshot_pub_ = create_publisher<std_msgs::msg::String>("/planner3d/reference_snapshot", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planner3d/status", 10);
    map_generation_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/planner3d/map_generation", map_qos);
    path_map_generation_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/planner3d/path_map_generation", map_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planner3d/markers", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.2, rate_hz)), [this]() {tick();});
    // Do not leave a completed sub-second job waiting for the next 2 Hz
    // planning tick: that can coincide with the next 1 Hz map and needlessly
    // discard useful work. This timer only wakes the normal single-threaded
    // result-consumption path once per job.
    worker_poll_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
        if (!worker_completion_notified_ && planning_future_.valid() &&
          planning_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
          worker_completion_notified_ = true;
          tick();
        }
      });
    RCLCPP_WARN(
      get_logger(),
      "3D planner is OBSERVATION ONLY: no PX4 publishers; unknown=blocked radius=%.2fm",
      config_.lethal_radius);
  }

  ~GlobalPlanner3DNode() override
  {
    if (planning_future_.valid()) {planning_future_.wait();}
  }

private:
  struct ForwardProbe
  {
    Point3 center;
    double step{};
    SphereBlockers3D blockers;
  };

  struct PlanningRequest
  {
    std::shared_ptr<const VoxelGrid> raw;
    std::shared_ptr<const GeofenceMask> geofence_mask;
    Point3 pose;
    Point3 goal;
    Planner3DConfig config;
    std::int64_t map_generation{};
    std::int64_t map_stamp_ns{};
    std::int64_t goal_generation{};
    Pose3 map_from_odom;
    bool exploration_route{};
    double grid_ms{};
  };

  struct PlanningOutput
  {
    std::shared_ptr<const VoxelGrid> raw;
    Point3 planned_pose;
    Point3 requested_goal;
    PlanResult3D result;
    ForwardProbe probe;
    std::vector<Point3> reference_path;
    nlohmann::ordered_json reference_info;
    nlohmann::ordered_json timing;
    std::int64_t map_generation{};
    std::int64_t map_stamp_ns{};
    std::int64_t goal_generation{};
    Pose3 map_from_odom;
    bool exploration_route{};
  };

  struct PlanningWorkerCache
  {
    std::int64_t map_generation{-1};
    std::int64_t map_stamp_ns{-1};
    bool exploration_route{};
    std::optional<PreparedPlanner3D> planner;
  };

  static ForwardProbe make_forward_probe(
    const VoxelGrid & raw, const Point3 & pose, const Point3 & goal, double radius)
  {
    const double dx = goal.x - pose.x, dy = goal.y - pose.y, dz = goal.z - pose.z;
    const double distance = std::hypot(std::hypot(dx, dy), dz);
    const double step = std::min(0.25, distance);
    const double scale = distance > 1e-12 ? step / distance : 0.0;
    const Point3 center{pose.x + dx * scale, pose.y + dy * scale, pose.z + dz * scale};
    return {center, step, sphere_blockers_3d(raw, center, radius)};
  }

  static PlanningOutput calculate_plan(
    PlanningRequest request, const std::shared_ptr<PlanningWorkerCache> & cache)
  {
    const auto worker_begin = steady_seconds();
    const bool cache_hit = cache->planner &&
      cache->map_generation == request.map_generation &&
      cache->map_stamp_ns == request.map_stamp_ns &&
      cache->exploration_route == request.exploration_route;
    const auto prepare_begin = steady_seconds();
    if (!cache_hit) {
      if (request.exploration_route) {
        // Unknown-inside-geofence is the common case, so construct the
        // optimistic view as free and write only hard obstacles. This avoids
        // copying the observed grid and then rewriting most of it.
        VoxelGrid optimistic(
          request.raw->width(), request.raw->height(), request.raw->depth(),
          request.raw->resolution(), request.raw->origin(), VoxelState::Free);
        for (int z = 0; z < static_cast<int>(optimistic.depth()); ++z) {
          for (int y = 0; y < static_cast<int>(optimistic.height()); ++y) {
            for (int x = 0; x < static_cast<int>(optimistic.width()); ++x) {
              const Voxel voxel{x, y, z};
              if (!request.geofence_mask->inside[
                  static_cast<std::size_t>(y) * optimistic.width() +
                  static_cast<std::size_t>(x)])
              {
                optimistic.set(voxel, VoxelState::Occupied);
              } else if (request.raw->at(voxel) == VoxelState::Occupied) {
                optimistic.set(voxel, VoxelState::Occupied);
              }
            }
          }
        }
        auto proposal_config = request.config;
        proposal_config.heuristic_weight = 2.5;
        proposal_config.project_blocked_goal = true;
        cache->planner.emplace(std::move(optimistic), proposal_config, request.raw.get());
      } else {
        cache->planner.emplace(*request.raw, request.config);
      }
      cache->map_generation = request.map_generation;
      cache->map_stamp_ns = request.map_stamp_ns;
      cache->exploration_route = request.exploration_route;
    }
    const auto plan_begin = steady_seconds();
    auto result = cache->planner->plan(request.pose, request.goal);
    std::vector<Point3> reference_path;
    nlohmann::ordered_json reference_info;
    if (request.exploration_route) {
      reference_path = result.path;
      auto verified = verified_prefix_3d(
        *request.raw, reference_path, request.config.lethal_radius);
      const bool complete = !verified.empty() && verified.size() == reference_path.size() &&
        verified.back().x == reference_path.back().x &&
        verified.back().y == reference_path.back().y &&
        verified.back().z == reference_path.back().z;
      const double prefix_length = path_length_3d(verified);
      reference_info = { {"state", result.reason},
        {"map_generation", request.map_generation},
        {"goal_generation", request.goal_generation},
        {"length_m", path_length_3d(reference_path)},
        {"fully_verified", complete},
        {"verified_prefix_m", prefix_length},
        {"verified_local_path_m", prefix_length},
        {"local_route_mode", "prefix"},
        {"known_fallback_used", false},
        {"goal_exact", result.goal && result.goal->exact},
        {"executable", false}};
      result.path = std::move(verified);
      if (!complete && result.goal) {
        result.goal->exact = false;
        result.goal->terminal = false;
      }
      if (!complete && !reference_path.empty()) {
        result.reason = result.path.empty() ? "REFERENCE_ONLY" : "EXPLORING";
      }
    }
    const auto plan_end = steady_seconds();
    auto probe = make_forward_probe(
      *request.raw, request.pose, request.goal, request.config.lethal_radius);
    PlanningOutput output;
    output.raw = std::move(request.raw);
    output.planned_pose = request.pose;
    output.requested_goal = request.goal;
    output.result = std::move(result);
    output.probe = std::move(probe);
    output.reference_path = std::move(reference_path);
    output.reference_info = std::move(reference_info);
    output.timing = {{"map_generation", request.map_generation},
      {"map_cache_hit", cache_hit}, {"grid_ms", request.grid_ms},
      {"prepare_ms", (plan_begin - prepare_begin) * 1000.0},
      {"plan_ms", (plan_end - plan_begin) * 1000.0},
      {"probe_ms", (steady_seconds() - plan_end) * 1000.0},
      {"worker_total_ms", (steady_seconds() - worker_begin) * 1000.0}};
    output.map_generation = request.map_generation;
    output.map_stamp_ns = request.map_stamp_ns;
    output.goal_generation = request.goal_generation;
    output.map_from_odom = request.map_from_odom;
    output.exploration_route = request.exploration_route;
    return output;
  }

  void add_probe_status(nlohmann::ordered_json & status, const ForwardProbe * probe) const
  {
    if (!probe) {return;}
    status["forward_probe"] = {{"center", {probe->center.x, probe->center.y, probe->center.z}},
      {"step_m", probe->step}, {"radius", config_.lethal_radius},
      {"inside_bounds", probe->blockers.inside_bounds},
      {"unknown_voxels", probe->blockers.unknown.size()},
      {"occupied_voxels", probe->blockers.occupied.size()},
      {"diagnostic_only", true}};
  }

  void validate_config()
  {
    if (!std::isfinite(voxel_size_) || voxel_size_ <= 0.0 ||
      !std::isfinite(robot_radius_) || robot_radius_ <= 0.0 ||
      !std::isfinite(safety_margin_) || safety_margin_ < 0.10 ||
      !std::isfinite(max_cross_track_) || max_cross_track_ < 0.0 ||
      safety_margin_ + 1.0e-12 < max_cross_track_ ||
      !std::isfinite(planning_radius_xy_) || planning_radius_xy_ <= 0.0 ||
      !std::isfinite(min_z_) || !std::isfinite(max_z_) || min_z_ >= max_z_ ||
      !std::isfinite(inflation_extra_) || inflation_extra_ < 0.0 ||
      !std::isfinite(map_preparation_period_) || map_preparation_period_ < 0.0 ||
      !std::isfinite(planning_pose_drift_tolerance_) ||
      planning_pose_drift_tolerance_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid 3D configuration (requires safety_margin>=0.10 and "
              "safety_margin>=max_cross_track)");
    }
  }

  void on_map(octomap_msgs::msg::Octomap::ConstSharedPtr message)
  {
    if (message->header.frame_id != frame_id_) {
      publish_status("MAP_FRAME_REJECTED");
      return;
    }
    if (!std::isfinite(message->resolution) || message->resolution <= 0.0 ||
      message->data.empty())
    {
      publish_status("MAP_GEOMETRY_REJECTED");
      return;
    }
    if (message->id != "OcTree" && message->id != "ColorOcTree")
    {
      publish_status("MAP_TYPE_REJECTED");
      return;
    }
    // Pair and retain the serialized latest map cheaply. Deserializing every
    // 1 Hz update consumed almost as much CPU as inflation even when that map
    // was intentionally skipped by the preparation throttle.
    pending_map_message_ = std::move(message);
    pending_map_stamp_ = rclcpp::Time(pending_map_message_->header.stamp);
    if (!require_map_metadata_) {
      pending_metadata_ = MapMetadata{
        map_generation_ + 1, pending_map_stamp_.nanoseconds(), pending_map_message_->resolution,
        pending_map_message_->header.frame_id, "unpaired", Pose3{}};
    }
    try_accept_map();
  }

  void on_metadata(const std_msgs::msg::String & message)
  {
    try {
      const auto json = nlohmann::json::parse(message.data);
      if (!json.at("ground_is_obstacle").get<bool>() || !json.at("ray_tracing").get<bool>()) {
        publish_status("MAP_CONTRACT_REJECTED");
        return;
      }
      pending_metadata_ = MapMetadata{
        json.at("generation").get<std::int64_t>(),
        json.at("stamp_ns").get<std::int64_t>(),
        json.at("resolution").get<double>(),
        json.at("frame_id").get<std::string>(),
        json.at("source_pose_generation").get<std::string>(),
        snapshot_pose(json.at("map_from_odom"))};
      try_accept_map();
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "map metadata rejected: %s", error.what());
      publish_status("MAP_METADATA_REJECTED");
    }
  }

  void try_accept_map()
  {
    if (!pending_map_message_ || !pending_metadata_) {return;}
    if (pending_metadata_->stamp_ns != pending_map_stamp_.nanoseconds() ||
      pending_metadata_->frame_id != frame_id_ ||
      std::abs(pending_metadata_->resolution - pending_map_message_->resolution) > 1.0e-9 ||
      pending_metadata_->generation <= map_generation_ ||
      !fresh_stamp(pending_metadata_->stamp_ns, now().nanoseconds(), map_timeout_))
    {
      publish_status("MAP_METADATA_MISMATCH");
      return;
    }
    map_message_ = std::move(pending_map_message_);
    map_stamp_ = pending_map_stamp_;
    map_received_ = steady_seconds();
    map_generation_ = pending_metadata_->generation;
    map_from_odom_ = pending_metadata_->map_from_odom;
    source_pose_generation_ = pending_metadata_->source_pose_generation;
    pending_metadata_.reset();
    std_msgs::msg::Int32 generation;
    generation.data = static_cast<std::int32_t>(map_generation_);
    map_generation_pub_->publish(generation);
    // Keep serving the previous source-tagged route while preparation of the
    // newest map is throttled. The flight adapter independently reframes and
    // collision-checks that route against every accepted OctoMap.
  }

  void on_snapshot(const std_msgs::msg::String & message)
  {
    try {
      const auto value = nlohmann::json::parse(message.data);
      const auto stamp = value.at("stamp_ns").get<std::int64_t>();
      if (value.at("frame_id") != frame_id_ ||
        !fresh_stamp(stamp, now().nanoseconds(), pose_timeout_) || stamp <= pose_stamp_) {return;}
      const auto pose = snapshot_pose(value.at("pose"));
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = frame_id_; msg.header.stamp = rclcpp::Time(stamp);
      msg.pose.position.x = pose.position.x; msg.pose.position.y = pose.position.y;
      msg.pose.position.z = pose.position.z;
      pose_generation_ = value.at("map_generation");
      pose_map_stamp_ = value.at("map_stamp_ns");
      pose_map_from_odom_ = snapshot_pose(value.at("correction"));
      on_pose(msg);
    } catch (const std::exception &) {publish_status("POSE_SNAPSHOT_REJECTED");}
  }

  void on_pose(const geometry_msgs::msg::PoseStamped & message)
  {
    const Point3 point{message.pose.position.x, message.pose.position.y, message.pose.position.z};
    if (message.header.frame_id != frame_id_ || !finite(point) ||
      !fresh_stamp(rclcpp::Time(message.header.stamp).nanoseconds(), now().nanoseconds(), pose_timeout_)) {
      publish_status("POSE_REJECTED");
      return;
    }
    pose_stamp_ = rclcpp::Time(message.header.stamp).nanoseconds();
    pose_ = point;
    if (snapshot_topic_.empty()) {
      // The legacy pose topic is already expressed in the current map frame.
      pose_map_from_odom_ = map_from_odom_;
    }
    pose_received_ = steady_seconds();
    if (!takeoff_center_) {
      takeoff_center_ = point;
      plan_dirty_ = true;
    }
    if (!last_planned_pose_ ||
      std::hypot(point.x - last_planned_pose_->x, point.y - last_planned_pose_->y) >=
      voxel_size_ * 0.5 || std::abs(point.z - last_planned_pose_->z) >= voxel_size_ * 0.5)
    {
      plan_dirty_ = true;
    }
  }

  void on_goal(const geometry_msgs::msg::PointStamped & message)
  {
    const Point3 point{message.point.x, message.point.y, message.point.z};
    if (message.header.frame_id != frame_id_ || !finite(point)) {
      publish_status("GOAL_FRAME_REJECTED");
      return;
    }
    clicked_goal_ = point;
    if (goal_height_mode_ == "current") {
      if (!current_goal_pose_ready()) {
        // A Foxglove click is a user command, not disposable sensor data. Map
        // publication briefly leads its matching pose snapshot on the live
        // graph, so retain the newest click and resolve its height as soon as a
        // coherent pose/map pair arrives. The 20260921T142531Z flight lost its
        // only post-hover click in exactly this window.
        pending_current_goal_ = point;
        goal_.reset();
        fail_closed("GOAL_HEIGHT_POSE_UNAVAILABLE");
        return;
      }
    }
    accept_goal(point);
  }

  bool current_goal_pose_ready() const
  {
    return pose_ && fresh_stamp(pose_stamp_, now().nanoseconds(), pose_timeout_) &&
      (snapshot_topic_.empty() || (pose_generation_ == map_generation_ &&
      pose_map_stamp_ == map_stamp_.nanoseconds()));
  }

  void accept_goal(Point3 resolved)
  {
    if (goal_height_mode_ == "current") {
      resolved.z = pose_->z;
      if (takeoff_center_) {
        // In the flight monitor, a floor click made before takeoff must describe
        // the hover-level XY destination, not a descent back toward the floor.
        // This floor is zero by default and explicitly set to 0.30 m by the
        // ROS/PX4 3D monitor launch.
        resolved.z = std::max(
          resolved.z, takeoff_center_->z + current_goal_min_height_above_start_);
      }
    }
    goal_ = resolved;
    pending_current_goal_.reset();
    ++goal_generation_;
    plan_dirty_ = true;
  }

  void tick()
  {
    const auto now = steady_seconds();
    // Transient freshness faults report status without withdrawing a route.
    // The worker below keeps this executor responsive while inflation and A*
    // run; consumers independently hold on stale map/pose/path snapshots.
    if (!map_message_ || now - map_received_ > map_timeout_ ||
      !fresh_stamp(map_stamp_.nanoseconds(), this->now().nanoseconds(), map_timeout_))
    {
      publish_status("MAP_STALE");
      return;
    }
    if (!pose_ || now - pose_received_ > pose_timeout_ ||
      !fresh_stamp(pose_stamp_, this->now().nanoseconds(), pose_timeout_))
    {
      publish_status("POSE_STALE");
      return;
    }
    if (!snapshot_topic_.empty() && (pose_generation_ != map_generation_ ||
      pose_map_stamp_ != map_stamp_.nanoseconds() ||
      !same_snapshot_transform(pose_map_from_odom_, map_from_odom_)))
    {
      publish_status("POSE_MAP_MISMATCH");
      return;
    }
    if (pending_current_goal_) {accept_goal(*pending_current_goal_);}
    if (!goal_) {fail_closed("WAITING_FOR_GOAL"); return;}
    if (!takeoff_center_) {fail_closed("WAITING_FOR_TAKEOFF_ORIGIN"); return;}
    if (!inside_geofence(*goal_)) {fail_closed("GOAL_OUTSIDE_GEOFENCE"); return;}
    if (!inside_start_envelope(*pose_)) {
      fail_closed("START_ENVELOPE_OUTSIDE_GEOFENCE");
      return;
    }
    const bool map_preparation_pending = raw_grid_generation_ != map_generation_ ||
      raw_grid_map_stamp_ns_ != map_stamp_.nanoseconds();
    const bool map_preparation_due = !raw_grid_ || (map_preparation_pending &&
      now - last_map_preparation_ >= map_preparation_period_);
    if (!plan_dirty_ && !map_preparation_due) {
      publish_path(accepted_path_, path_pub_);
      publish_reference();
      std_msgs::msg::Int32 path_generation;
      path_generation.data = static_cast<std::int32_t>(path_map_generation_);
      path_map_generation_pub_->publish(path_generation);
      return;
    }

    if (planning_future_.valid()) {
      if (planning_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
      {
        publish_status("PLANNING");
        return;
      }
      try {
        auto output = planning_future_.get();
        const auto pose_in_source = reframe_map_point_3d(
          *pose_, map_from_odom_, output.map_from_odom);
        const auto pose_drift = std::hypot(
          std::hypot(pose_in_source.x - output.planned_pose.x,
          pose_in_source.y - output.planned_pose.y),
          pose_in_source.z - output.planned_pose.z);
        const bool pose_matches =
          pose_drift <= planning_pose_drift_tolerance_ + 1.0e-12;
        const Point3 * route_entry = !output.result.path.empty() ?
          &output.result.path.front() : (!output.reference_path.empty() ?
          &output.reference_path.front() : nullptr);
        const bool connector_clear = !route_entry || swept_sphere_clear(
          *output.raw, pose_in_source, *route_entry, config_.lethal_radius);
        const bool source_is_current = output.map_generation == map_generation_ &&
          output.map_stamp_ns == map_stamp_.nanoseconds();
        const bool current = output.goal_generation == goal_generation_ && pose_matches &&
          connector_clear && (source_is_current || output.result.found());
        if (current) {
          // A completed plan remains usable after bounded motion only if the
          // verified connector is part of the route handed to consumers.
          if (pose_drift > 1.0e-9) {
            if (!output.result.path.empty()) {
              output.result.path.insert(output.result.path.begin(), pose_in_source);
            }
            if (!output.reference_path.empty()) {
              output.reference_path.insert(output.reference_path.begin(), pose_in_source);
              output.reference_info["length_m"] =
                output.reference_info.value("length_m", 0.0) + pose_drift;
              output.reference_info["verified_prefix_m"] =
                output.reference_info.value("verified_prefix_m", 0.0) + pose_drift;
              output.reference_info["verified_local_path_m"] =
                output.reference_info.value("verified_local_path_m", 0.0) + pose_drift;
            }
          }
          reference_path_ = std::move(output.reference_path);
          reference_info_ = std::move(output.reference_info);
          reference_map_generation_ = output.map_generation;
          reference_map_stamp_ns_ = output.map_stamp_ns;
          plan_timing_ = std::move(output.timing);
          publish_reference();
          publish_candidate(output.exploration_route ? reference_path_ : output.result.path);
          publish_markers(
            *output.raw, output.result, output.probe,
            output.map_generation, output.map_stamp_ns);
          last_planned_pose_ = *pose_;
          if (!output.result.found()) {
            fail_closed(output.result.reason, &output.probe);
            // The same immutable inputs cannot produce a different answer.
            // Map, goal, or meaningful pose motion will mark planning dirty.
            plan_dirty_ = false;
            return;
          }
          accepted_goal_exact_ = output.result.goal && output.result.goal->exact;
          accepted_goal_terminal_ = output.result.goal && output.result.goal->terminal;
          accepted_path_ = output.result.path;
          path_map_generation_ = output.map_generation;
          path_map_stamp_ns_ = output.map_stamp_ns;
          publish_path(accepted_path_, path_pub_);
          std_msgs::msg::Int32 path_generation;
          path_generation.data = static_cast<std::int32_t>(path_map_generation_);
          path_map_generation_pub_->publish(path_generation);
          plan_dirty_ = false;
          publish_result_status(
            output.result, output.raw->resolution(), output.probe);
          return;
        }
        // The completed job is obsolete. There is deliberately no queue: the
        // latest coherent map/pose/goal snapshot below replaces every update
        // which arrived while this job was running.
        if (!source_is_current) {
          // Do not spin on a negative or disconnected result from an older
          // prepared map. Keep the retained route (if any) and wait until the
          // bounded map-preparation interval permits a current-map refresh.
          plan_dirty_ = false;
          publish_status("WAITING_FOR_MAP_PREPARATION");
          return;
        }
        plan_dirty_ = true;
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "3D planning worker failed: %s", error.what());
        fail_closed("PLANNING_WORKER_FAILED");
        return;
      }
    }

    if (map_preparation_due) {
      const auto grid_begin = steady_seconds();
      auto grid = make_planning_grid();
      if (!grid) {fail_closed("MAP_CROP_INVALID"); return;}
      raw_grid_ = std::make_shared<const VoxelGrid>(std::move(*grid));
      raw_grid_generation_ = map_generation_;
      raw_grid_map_stamp_ns_ = map_stamp_.nanoseconds();
      raw_grid_map_from_odom_ = map_from_odom_;
      raw_grid_ms_ = (steady_seconds() - grid_begin) * 1000.0;
      last_map_preparation_ = now;
      ++map_preparation_count_;
    }
    if (!raw_grid_) {
      publish_status("WAITING_FOR_MAP_PREPARATION");
      return;
    }
    PlanningRequest request;
    request.raw = raw_grid_;
    request.geofence_mask = geofence_mask_;
    request.pose = reframe_map_point_3d(*pose_, map_from_odom_, raw_grid_map_from_odom_);
    request.goal = *goal_;
    request.config = config_;
    request.map_generation = raw_grid_generation_;
    request.map_stamp_ns = raw_grid_map_stamp_ns_;
    request.goal_generation = goal_generation_;
    request.map_from_odom = raw_grid_map_from_odom_;
    request.exploration_route = exploration_route_;
    request.grid_ms = raw_grid_ms_;
    auto cache = planning_worker_cache_;
    planning_future_ = std::async(
      std::launch::async, [request = std::move(request), cache]() mutable {
        return calculate_plan(std::move(request), cache);
      });
    worker_completion_notified_ = false;
    publish_status("PLANNING");
  }

  bool inside_geofence(const Point3 & point) const
  {
    return takeoff_center_ &&
           std::hypot(point.x - takeoff_center_->x, point.y - takeoff_center_->y) <=
           planning_radius_xy_ + 1.0e-12 && point.z >= min_z_ && point.z <= max_z_;
  }

  bool inside_start_envelope(const Point3 & point) const
  {
    const auto radius = config_.lethal_radius;
    return takeoff_center_ && point.z - radius >= min_z_ && point.z + radius <= max_z_ &&
      std::hypot(point.x - takeoff_center_->x, point.y - takeoff_center_->y) + radius <=
      planning_radius_xy_;
  }

  void add_geofence_status(nlohmann::ordered_json & status) const
  {
    status["geofence"] = {{"min_z", min_z_}, {"max_z", max_z_},
      {"planning_radius_xy", planning_radius_xy_},
      {"min_center_z", min_z_ + config_.lethal_radius},
      {"max_center_z", max_z_ - config_.lethal_radius}};
    status["lethal_radius"] = config_.lethal_radius;
    if (pose_) {status["start"] = {pose_->x, pose_->y, pose_->z};}
    if (goal_) {status["requested_goal"] = {goal_->x, goal_->y, goal_->z};}
    status["goal_height_mode"] = goal_height_mode_;
    status["current_goal_min_height_above_start"] = current_goal_min_height_above_start_;
    status["goal_height_pending"] = pending_current_goal_.has_value();
    status["exploration_route"] = exploration_route_;
    status["prepared_map_generation"] = raw_grid_generation_;
    status["prepared_map_stamp_ns"] = raw_grid_map_stamp_ns_;
    status["map_preparation_pending"] = raw_grid_generation_ != map_generation_ ||
      raw_grid_map_stamp_ns_ != map_stamp_.nanoseconds();
    status["map_preparation_period_s"] = map_preparation_period_;
    status["map_preparation_count"] = map_preparation_count_;
    status["planning_pose_drift_tolerance_m"] = planning_pose_drift_tolerance_;
    if (!reference_info_.empty() &&
      reference_info_["goal_generation"] == goal_generation_) {
      status["reference_route"] = reference_info_;
      status["reference_route"]["source_is_current"] =
        reference_map_generation_ == map_generation_ &&
        reference_map_stamp_ns_ == map_stamp_.nanoseconds();
    }
    if (!plan_timing_.empty()) {
      status["last_plan_timing"] = plan_timing_;
      status["last_plan_timing"]["source_is_current"] =
        plan_timing_.value("map_generation", std::int64_t(-1)) == map_generation_;
    }
    if (clicked_goal_) {status["clicked_goal"] = {clicked_goal_->x, clicked_goal_->y, clicked_goal_->z};}
  }

  std::optional<VoxelGrid> make_planning_grid()
  {
    std::unique_ptr<octomap::AbstractOcTree> octree;
    try {
      octree.reset(octomap_msgs::msgToMap(*map_message_));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "OctoMap deserialization failed: %s", error.what());
      return std::nullopt;
    }
    if (!octree || (dynamic_cast<octomap::OcTree *>(octree.get()) == nullptr &&
      dynamic_cast<octomap::ColorOcTree *>(octree.get()) == nullptr))
    {
      return std::nullopt;
    }
    const auto resolution = octree->getResolution();
    // Matching OctoMap's native resolution keeps every planner centre aligned
    // with an OctoMap key and avoids inventing sub-voxel free-space evidence.
    if (std::abs(resolution - voxel_size_) > 1.0e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "voxel_size %.3f differs from OctoMap %.3f; using OctoMap resolution",
        voxel_size_, resolution);
    }
    const auto align_origin = [resolution](double lower) {
        // OctoMap's depth-16 keys are cubes [n*r,(n+1)*r] with centres
        // (n+0.5)*r, which is exactly VoxelGrid's convention.
        return std::floor(lower / resolution) * resolution;
      };
    const Point3 origin{
      align_origin(takeoff_center_->x - planning_radius_xy_),
      align_origin(takeoff_center_->y - planning_radius_xy_),
      align_origin(min_z_)};
    const auto cells = [resolution](double lower, double upper) {
        return static_cast<std::size_t>(std::ceil((upper - lower) / resolution));
      };
    const auto width = cells(origin.x, takeoff_center_->x + planning_radius_xy_);
    const auto height = cells(origin.y, takeoff_center_->y + planning_radius_xy_);
    const auto depth = cells(origin.z, max_z_);
    if (width == 0 || height == 0 || depth == 0 ||
      width > 512 || height > 512 || depth > 256 ||
      width * height * depth > 6000000ULL)
    {
      return std::nullopt;
    }
    VoxelGrid grid(width, height, depth, resolution, origin);
    if (const auto * tree = dynamic_cast<const octomap::OcTree *>(octree.get())) {
      fill_native_octomap_grid(*tree, grid);
    } else if (const auto * tree = dynamic_cast<const octomap::ColorOcTree *>(octree.get())) {
      fill_native_octomap_grid(*tree, grid);
    }
    if (!geofence_mask_ || !geofence_mask_->matches(
        grid, *takeoff_center_, planning_radius_xy_))
    {
      geofence_mask_ = make_geofence_mask(grid, *takeoff_center_, planning_radius_xy_);
    }
    for (int y = 0; y < static_cast<int>(height); ++y) {
      for (int x = 0; x < static_cast<int>(width); ++x) {
        if (geofence_mask_->inside[static_cast<std::size_t>(y) * width +
          static_cast<std::size_t>(x)]) {continue;}
        // The cylinder is the takeoff-centred XY geofence. Cells outside it
        // stay unknown and are therefore hard obstacles.
        for (int z = 0; z < static_cast<int>(depth); ++z) {
          grid.set({x, y, z}, VoxelState::Unknown);
        }
      }
    }
    return grid;
  }

  void publish_path(
    const std::vector<Point3> & points,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher)
  {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }
    publisher->publish(message);
    if (publisher == path_pub_) {
      nlohmann::json points_json = nlohmann::json::array();
      for (const auto & point : points) {points_json.push_back({point.x, point.y, point.z});}
      std_msgs::msg::String snapshot;
      snapshot.data = nlohmann::json{{"stamp_ns", rclcpp::Time(message.header.stamp).nanoseconds()},
        {"frame_id", frame_id_}, {"map_generation", path_map_generation_},
        {"map_stamp_ns", path_map_stamp_ns_}, {"lethal_radius", config_.lethal_radius},
        {"points", points_json}, {"goal_generation", goal_generation_}, {"goal_exact", accepted_goal_exact_},
        {"goal_terminal", accepted_goal_terminal_}, {"executable", true}}.dump();
      path_snapshot_pub_->publish(snapshot);
    }
  }

  void publish_candidate(const std::vector<Point3> & points)
  {
    publish_path(points, candidate_pub_);
  }

  void publish_reference()
  {
    publish_path(reference_path_, reference_pub_);
    nlohmann::json points = nlohmann::json::array();
    for (const auto & p : reference_path_) {points.push_back({p.x, p.y, p.z});}
    std_msgs::msg::String message;
    message.data = nlohmann::json{{"stamp_ns", now().nanoseconds()}, {"frame_id", frame_id_},
      {"map_generation", reference_map_generation_},
      {"map_stamp_ns", reference_map_stamp_ns_},
      {"goal_generation", goal_generation_}, {"lethal_radius", config_.lethal_radius},
      {"executable", false}, {"unknown_policy", "penalized"},
      {"points", points}, {"verification", reference_info_}}.dump();
    reference_snapshot_pub_->publish(message);
  }

  void fail_closed(const std::string & reason, const ForwardProbe * probe = nullptr)
  {
    if (!probe) {
      reference_path_.clear(); reference_info_ = nlohmann::ordered_json::object();
      reference_map_generation_ = map_generation_;
      reference_map_stamp_ns_ = map_stamp_.nanoseconds();
      publish_reference();
    }
    plan_dirty_ = true;
    accepted_path_.clear();
    path_map_generation_ = map_generation_;
    path_map_stamp_ns_ = map_stamp_.nanoseconds();
    accepted_goal_exact_ = accepted_goal_terminal_ = false;
    publish_path({}, path_pub_);
    publish_status(reason, probe);
  }

  void publish_status(const std::string & reason, const ForwardProbe * probe = nullptr)
  {
    nlohmann::ordered_json status;
    status["state"] = reason;
    status["observation_only"] = true;
    status["map_generation"] = map_generation_;
    status["path_map_generation"] = path_map_generation_;
    status["goal_generation"] = goal_generation_;
    status["map_stamp_ns"] = map_stamp_.nanoseconds();
    status["source_pose_generation"] = source_pose_generation_;
    status["lethal_radius"] = config_.lethal_radius;
    std_msgs::msg::String message;
    add_geofence_status(status);
    add_probe_status(status, probe);
    message.data = status.dump();
    status_pub_->publish(message);
  }

  void publish_result_status(const PlanResult3D & result, double resolution, const ForwardProbe & probe)
  {
    nlohmann::ordered_json status;
    status["state"] = result.reason;
    status["observation_only"] = true;
    status["map_generation"] = map_generation_;
    status["path_map_generation"] = path_map_generation_;
    status["goal_generation"] = goal_generation_;
    status["map_stamp_ns"] = map_stamp_.nanoseconds();
    status["source_pose_generation"] = source_pose_generation_;
    status["resolution"] = resolution;
    status["expanded"] = result.search.expanded;
    status["planning_ms"] = result.search.elapsed_ms;
    status["path_length"] = path_length_3d(result.path);
    status["goal_exact"] = result.goal && result.goal->exact;
    status["goal_terminal"] = result.goal && result.goal->terminal;
    status["reachable_voxels"] = result.goal ? result.goal->reachable_voxels : 0;
    const auto & endpoint = result.path.back();
    const double remaining = std::hypot(std::hypot(goal_->x - endpoint.x, goal_->y - endpoint.y),
      goal_->z - endpoint.z);
    const double initial = std::hypot(std::hypot(goal_->x - pose_->x, goal_->y - pose_->y),
      goal_->z - pose_->z);
    status["goal_remaining_m"] = remaining;
    status["goal_progress_m"] = initial - remaining;
    status["short_partial_path"] = !status["goal_exact"].get<bool>() && path_length_3d(result.path) < 0.15;
    std_msgs::msg::String message;
    add_geofence_status(status);
    add_probe_status(status, &probe);
    message.data = status.dump();
    status_pub_->publish(message);
  }

  void publish_markers(
    const VoxelGrid & raw, const PlanResult3D & result, const ForwardProbe & probe,
    std::int64_t source_generation, std::int64_t source_stamp_ns)
  {
    visualization_msgs::msg::MarkerArray array;
    const bool refresh_occupancy = marker_map_generation_ != source_generation ||
      marker_map_stamp_ns_ != source_stamp_ns;
    if (refresh_occupancy) {
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      array.markers.push_back(clear);
    }
    if (!reference_path_.empty()) {
      visualization_msgs::msg::Marker route;
      route.header.stamp = now(); route.header.frame_id = frame_id_;
      route.ns = "reference_route"; route.id = 10;
      route.type = visualization_msgs::msg::Marker::LINE_STRIP;
      route.action = visualization_msgs::msg::Marker::ADD;
      route.pose.orientation.w = 1.0; route.scale.x = 0.025;
      route.color.r = 1.0F; route.color.g = 0.65F; route.color.b = 0.0F; route.color.a = 0.9F;
      route.lifetime.sec = 2;
      for (const auto & p : reference_path_) {
        geometry_msgs::msg::Point point; point.x = p.x; point.y = p.y; point.z = p.z;
        route.points.push_back(point);
      }
      array.markers.push_back(route);
      auto label = route;
      label.ns = "reference_route_label"; label.id = 11;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.points.clear(); label.scale.z = 0.07;
      label.pose.position.x = reference_path_.front().x;
      label.pose.position.y = reference_path_.front().y;
      label.pose.position.z = reference_path_.front().z + 0.65;
      label.text = reference_info_.value("fully_verified", false) ?
        "Mapped route" : "Proposed route: unknown allowed";
      array.markers.push_back(label);
    }
    visualization_msgs::msg::Marker occupied;
    occupied.header.stamp = now();
    occupied.header.frame_id = frame_id_;
    occupied.ns = "occupied";
    occupied.id = 1;
    occupied.type = visualization_msgs::msg::Marker::CUBE_LIST;
    occupied.action = visualization_msgs::msg::Marker::ADD;
    occupied.pose.orientation.w = 1.0;
    occupied.scale.x = occupied.scale.y = occupied.scale.z = raw.resolution();
    occupied.color.r = 0.9F;
    occupied.color.g = 0.15F;
    occupied.color.b = 0.10F;
    occupied.color.a = 0.45F;
    if (refresh_occupancy) {
      const auto occupied_total = static_cast<std::size_t>(std::count(
          raw.data().begin(), raw.data().end(), VoxelState::Occupied));
      const auto stride = std::max<std::size_t>(
        1, max_marker_voxels_ == 0 ? occupied_total + 1 :
        (occupied_total + max_marker_voxels_ - 1) / max_marker_voxels_);
      std::size_t seen = 0;
      for (int z = 0; z < static_cast<int>(raw.depth()); ++z) {
        for (int y = 0; y < static_cast<int>(raw.height()); ++y) {
          for (int x = 0; x < static_cast<int>(raw.width()); ++x) {
            const Voxel voxel{x, y, z};
            if (raw.at(voxel) != VoxelState::Occupied || seen++ % stride != 0) {continue;}
            const auto center = raw.voxel_center(voxel);
            geometry_msgs::msg::Point point;
            point.x = center.x; point.y = center.y; point.z = center.z;
            occupied.points.push_back(point);
          }
        }
      }
      array.markers.push_back(occupied);
      marker_map_generation_ = source_generation;
      marker_map_stamp_ns_ = source_stamp_ns;
    }
    visualization_msgs::msg::Marker path;
    path.header = occupied.header;
    path.ns = "clearance_path";
    path.id = 2;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 2.0 * config_.lethal_radius;
    path.color.r = 0.15F; path.color.g = 0.65F; path.color.b = 1.0F; path.color.a = 0.20F;
    for (const auto & item : result.path) {
      geometry_msgs::msg::Point point;
      point.x = item.x; point.y = item.y; point.z = item.z;
      path.points.push_back(point);
    }
    array.markers.push_back(path);
    if (!result.goal || !result.goal->exact) {
      auto unknown = occupied;
      unknown.ns = "forward_unknown"; unknown.id = 3; unknown.points.clear();
      unknown.lifetime = rclcpp::Duration::from_seconds(2.0);
      unknown.color.r = 0.80F; unknown.color.g = 0.25F; unknown.color.b = 1.0F; unknown.color.a = 0.7F;
      for (const auto & voxel : probe.blockers.unknown) {
        if (unknown.points.size() >= max_marker_voxels_) {break;}
        const auto center = raw.voxel_center(voxel);
        geometry_msgs::msg::Point point;
        point.x = center.x; point.y = center.y; point.z = center.z;
        unknown.points.push_back(point);
      }
      array.markers.push_back(unknown);
      auto envelope = path;
      envelope.ns = "forward_probe"; envelope.id = 4; envelope.type = visualization_msgs::msg::Marker::SPHERE;
      envelope.points.clear(); envelope.pose.position.x = probe.center.x;
      envelope.lifetime = rclcpp::Duration::from_seconds(2.0);
      envelope.pose.position.y = probe.center.y; envelope.pose.position.z = probe.center.z;
      envelope.scale.x = envelope.scale.y = envelope.scale.z = 2.0 * config_.lethal_radius;
      envelope.color.r = 1.0F; envelope.color.g = 0.65F; envelope.color.b = 0.0F; envelope.color.a = 0.18F;
      array.markers.push_back(envelope);
      auto label = envelope;
      label.ns = "forward_probe_label"; label.id = 5;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z += config_.lethal_radius + 0.15;
      label.scale.z = 0.09; label.color.a = 1.0F;
      label.text = "Forward clearance probe (not a route)\nUnknown: " +
        std::to_string(probe.blockers.unknown.size()) + "  Occupied: " +
        std::to_string(probe.blockers.occupied.size());
      array.markers.push_back(label);
    }
    markers_pub_->publish(array);
  }

  ProcessSingleton singleton_;
  std::string map_topic_;
  std::string map_metadata_topic_;
  std::string pose_topic_;
  std::string goal_topic_;
  std::string goal_height_mode_;
  double current_goal_min_height_above_start_{};
  std::string frame_id_;
  std::string snapshot_topic_;
  bool accepted_goal_exact_{false}, accepted_goal_terminal_{false};
  std::int64_t pose_stamp_{}, pose_generation_{}, pose_map_stamp_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr snapshot_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_snapshot_pub_;
  double voxel_size_{};
  double robot_radius_{};
  double safety_margin_{};
  double max_cross_track_{};
  double inflation_extra_{};
  double planning_radius_xy_{};
  double min_z_{};
  double max_z_{};
  double map_timeout_{};
  double pose_timeout_{};
  double map_preparation_period_{};
  double planning_pose_drift_tolerance_{};
  std::size_t max_marker_voxels_{};
  bool require_map_metadata_{};
  Planner3DConfig config_;
  bool exploration_route_{};
  std::shared_ptr<const VoxelGrid> raw_grid_;
  std::shared_ptr<const GeofenceMask> geofence_mask_;
  std::int64_t raw_grid_generation_{-1};
  std::int64_t raw_grid_map_stamp_ns_{-1};
  Pose3 raw_grid_map_from_odom_;
  double raw_grid_ms_{};
  double last_map_preparation_{-std::numeric_limits<double>::infinity()};
  std::uint64_t map_preparation_count_{};
  std::int64_t marker_map_generation_{-1};
  std::int64_t marker_map_stamp_ns_{-1};
  std::shared_ptr<PlanningWorkerCache> planning_worker_cache_{
    std::make_shared<PlanningWorkerCache>()};
  std::future<PlanningOutput> planning_future_;
  bool worker_completion_notified_{false};
  std::vector<Point3> reference_path_;
  std::int64_t reference_map_generation_{};
  std::int64_t reference_map_stamp_ns_{};
  nlohmann::ordered_json reference_info_;
  nlohmann::ordered_json plan_timing_;
  octomap_msgs::msg::Octomap::ConstSharedPtr map_message_;
  octomap_msgs::msg::Octomap::ConstSharedPtr pending_map_message_;
  std::optional<MapMetadata> pending_metadata_;
  rclcpp::Time pending_map_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<Point3> pose_;
  std::optional<Point3> goal_;
  std::optional<Point3> clicked_goal_;
  std::optional<Point3> pending_current_goal_;
  std::optional<Point3> takeoff_center_;
  std::optional<Point3> last_planned_pose_;
  std::vector<Point3> accepted_path_;
  double map_received_{};
  double pose_received_{};
  std::int64_t map_generation_{};
  std::int64_t path_map_generation_{};
  std::int64_t path_map_stamp_ns_{};
  std::int64_t goal_generation_{};
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME};
  Pose3 map_from_odom_;
  Pose3 pose_map_from_odom_;
  std::string source_pose_generation_;
  bool plan_dirty_{true};
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reference_snapshot_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr map_generation_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr path_map_generation_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr worker_poll_timer_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<px4_vio_bridge::GlobalPlanner3DNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "global_planner_3d_monitor: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
