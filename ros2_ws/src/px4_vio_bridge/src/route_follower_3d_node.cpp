// Observation-only 3D route follower. It publishes proposed commands and a
// fail-closed validity gate, never PX4 setpoints.

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nlohmann/json.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "px4_vio_bridge/grid_planner_3d.hpp"
#include "px4_vio_bridge/flight_geometry_3d.hpp"
#include "px4_vio_bridge/octomap_grid.hpp"
#include "px4_vio_bridge/navigation_snapshot.hpp"
#include "px4_vio_bridge/route_follower_3d.hpp"

namespace px4_vio_bridge
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double steady_seconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

struct MapMetadata
{
  std::int64_t generation{};
  std::int64_t stamp_ns{};
  double resolution{};
  std::string frame_id;
  std::optional<Pose3> correction;
};

struct Correction3D
{
  std::array<double, 9> rotation{};
  Point3 translation{};
  double roll{};
  double pitch{};
  double yaw{};
};

struct MapFrame
{
  std::int64_t generation{};
  std::int64_t stamp_ns{};
  Pose3 map_from_odom;
};

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
      throw std::runtime_error("another 3D follower holds " + path_);
    }
    if (::ftruncate(descriptor_, 0) != 0) {
      throw std::runtime_error("cannot truncate follower singleton");
    }
    const auto pid = std::to_string(::getpid());
    if (::write(descriptor_, pid.data(), pid.size()) < 0) {
      throw std::runtime_error("cannot write follower singleton");
    }
  }
  ~ProcessSingleton()
  {
    if (descriptor_ >= 0) {::flock(descriptor_, LOCK_UN); ::close(descriptor_);}
  }

private:
  std::string path_;
  int descriptor_{-1};
};

std::optional<Correction3D> correction_from_pose(
  const geometry_msgs::msg::PoseStamped & message)
{
  const auto & q = message.pose.orientation;
  const auto norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm < 1.0e-9) {return std::nullopt;}
  const auto w = q.w / norm;
  const auto x = q.x / norm;
  const auto y = q.y / norm;
  const auto z = q.z / norm;
  Correction3D correction;
  correction.rotation = {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)};
  correction.translation = {
    message.pose.position.x, message.pose.position.y, message.pose.position.z};
  correction.roll = std::atan2(
    2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  correction.pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  correction.yaw = std::atan2(
    2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return correction;
}

Point3 inverse_rotate(const Correction3D & correction, const Point3 & map_vector)
{
  // map<-vio rotation transposed: displacement is never translated.
  return {
    correction.rotation[0] * map_vector.x + correction.rotation[3] * map_vector.y +
    correction.rotation[6] * map_vector.z,
    correction.rotation[1] * map_vector.x + correction.rotation[4] * map_vector.y +
    correction.rotation[7] * map_vector.z,
    correction.rotation[2] * map_vector.x + correction.rotation[5] * map_vector.y +
    correction.rotation[8] * map_vector.z};
}

}  // namespace

class RouteFollower3DNode final : public rclcpp::Node
{
public:
  RouteFollower3DNode()
  : Node("route_follower_3d_monitor"), singleton_("planner3d_follower")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    vio_frame_id_ = declare_parameter<std::string>("vio_frame_id", "continuous_vio");
    path_snapshot_topic_ = declare_parameter<std::string>("path_snapshot_topic", "/planner3d/path_snapshot");
    map_topic_ = declare_parameter<std::string>("map_topic", "/rtabmap/octomap");
    metadata_topic_ = declare_parameter<std::string>(
      "map_metadata_topic", "/rtabmap/octomap_metadata");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/rtabmap/pose");
    raw_vio_topic_ = declare_parameter<std::string>("raw_vio_topic", "/rtabmap/vio_pose");
    correction_topic_ = declare_parameter<std::string>(
      "correction_topic", "/rtabmap/odom_correction");
    path_timeout_ = declare_parameter<double>("path_timeout", 3.0);
    map_timeout_ = declare_parameter<double>("map_timeout", 3.0);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 1.0);
    vio_timeout_ = declare_parameter<double>("vio_timeout", 0.5);
    correction_timeout_ = declare_parameter<double>("correction_timeout", 1.0);
    planning_radius_xy_ = declare_parameter<double>("planning_radius_xy", 3.0);
    min_z_ = declare_parameter<double>("min_z", 0.20);
    max_z_ = declare_parameter<double>("max_z", 2.00);
    const auto robot_radius = declare_parameter<double>("robot_radius", 0.25);
    const auto safety_margin = declare_parameter<double>("safety_margin", 0.10);
    required_clearance_ = robot_radius + safety_margin;
    config_.lookahead = declare_parameter<double>("lookahead", 0.35);
    config_.max_horizontal_speed = declare_parameter<double>("max_horizontal_speed", 0.10);
    config_.max_vertical_speed = declare_parameter<double>("max_vertical_speed", 0.05);
    config_.max_horizontal_acceleration = declare_parameter<double>(
      "max_horizontal_acceleration", 0.30);
    config_.max_vertical_acceleration = declare_parameter<double>(
      "max_vertical_acceleration", 0.20);
    config_.max_cross_track = declare_parameter<double>("max_cross_track", 0.05);
    config_.max_vertical_track = declare_parameter<double>("max_vertical_track", 0.05);
    config_.arrival_tolerance = declare_parameter<double>("arrival_tolerance", 0.10);
    max_correction_translation_ = declare_parameter<double>("max_correction_m", 0.50);
    max_correction_yaw_ = declare_parameter<double>("max_correction_yaw_deg", 15.0) * kPi / 180.0;
    max_correction_roll_pitch_ = declare_parameter<double>(
      "max_correction_roll_pitch_deg", 5.0) * kPi / 180.0;
    if (safety_margin < 0.10 || safety_margin < config_.max_cross_track ||
      safety_margin < config_.max_vertical_track || min_z_ >= max_z_)
    {
      throw std::invalid_argument("3D follower clearance/tracking invariant violated");
    }
    follower_ = std::make_unique<RouteFollower3D>(config_);
    const auto rate_hz = declare_parameter<double>("rate_hz", 20.0);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      map_topic_, qos,
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr message) {on_map(*message);});
    metadata_sub_ = create_subscription<std_msgs::msg::String>(
      metadata_topic_, qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_metadata(*message);});
    path_snapshot_sub_ = create_subscription<std_msgs::msg::String>(
      path_snapshot_topic_, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_path_snapshot(*message);});
    snapshot_topic_ = declare_parameter<std::string>("pose_snapshot_topic", "");
    if (!snapshot_topic_.empty()) {
      snapshot_sub_ = create_subscription<std_msgs::msg::String>(snapshot_topic_, 10,
        [this](std_msgs::msg::String::ConstSharedPtr message) {on_snapshot(*message);});
    } else {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {on_pose(*message);});
    vio_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      raw_vio_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
        if (message->header.frame_id != vio_frame_id_ ||
          !fresh_stamp(stamp, now().nanoseconds(), vio_timeout_)) {return;}
        vio_stamp_ = stamp;
        vio_received_ = steady_seconds();
      });
    correction_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      correction_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        on_correction(*message);
      });
    }
    displacement_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/planner3d/follower/displacement", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/planner3d/follower/velocity", 10);
    acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/planner3d/follower/acceleration", 10);
    carrot_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner3d/follower/carrot", 10);
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner3d/follower/lookahead", 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/planner3d/follower/valid", 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("/planner3d/follower/goal_reached", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planner3d/follower/status", 10);
    last_tick_ = steady_seconds();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz)), [this]() {tick();});
    RCLCPP_WARN(get_logger(), "3D follower is OBSERVATION ONLY; no PX4 publishers");
  }

private:
  void on_map(const octomap_msgs::msg::Octomap & message)
  {
    if (message.header.frame_id != frame_id_ || message.data.empty()) {return;}
    auto tree = std::unique_ptr<octomap::AbstractOcTree>(octomap_msgs::msgToMap(message));
    if (dynamic_cast<octomap::OcTree *>(tree.get()) == nullptr &&
      dynamic_cast<octomap::ColorOcTree *>(tree.get()) == nullptr)
    {
      return;
    }
    pending_octree_ = std::move(tree);
    pending_map_stamp_ = rclcpp::Time(message.header.stamp);
    try_accept_map();
  }

  void on_metadata(const std_msgs::msg::String & message)
  {
    try {
      const auto json = nlohmann::json::parse(message.data);
      if (!json.at("ground_is_obstacle").get<bool>() || !json.at("ray_tracing").get<bool>()) {
        return;
      }
      pending_metadata_ = MapMetadata{
        json.at("generation").get<std::int64_t>(), json.at("stamp_ns").get<std::int64_t>(),
        json.at("resolution").get<double>(), json.at("frame_id").get<std::string>(),
        json.contains("map_from_odom") ? std::optional<Pose3>(snapshot_pose(json.at("map_from_odom"))) :
        std::nullopt};
      try_accept_map();
    } catch (const std::exception &) {
      pending_metadata_.reset();
    }
  }

  void try_accept_map()
  {
    if (!pending_octree_ || !pending_metadata_) {return;}
    if (pending_metadata_->stamp_ns != pending_map_stamp_.nanoseconds() ||
      pending_metadata_->frame_id != frame_id_ ||
      std::abs(pending_metadata_->resolution - pending_octree_->getResolution()) > 1.0e-9 ||
      pending_metadata_->generation <= map_generation_ ||
      !fresh_stamp(pending_metadata_->stamp_ns, now().nanoseconds(), map_timeout_))
    {
      return;
    }
    octree_ = std::move(pending_octree_);
    map_generation_ = pending_metadata_->generation;
    map_stamp_ = pending_metadata_->stamp_ns;
    map_correction_ = pending_metadata_->correction;
    if (map_correction_) {
      map_frames_.push_back({map_generation_, map_stamp_, *map_correction_});
      while (map_frames_.size() > 64) {map_frames_.pop_front();}
    }
    pending_metadata_.reset();
    map_received_ = steady_seconds();
    // Revalidate the retained source route against this newest map before it
    // can produce another command. Do not require the planner to reinflate the
    // same generation just to keep the observation follower alive.
    active_path_generation_.reset();
    try_accept_path();
  }

  void on_path_snapshot(const std_msgs::msg::String & message)
  {
    try {
      const auto value = nlohmann::json::parse(message.data);
      const auto stamp = value.at("stamp_ns").get<std::int64_t>();
      // A delayed old packet cannot supersede or refresh an accepted path.
      if (stamp <= path_stamp_) {return;}
      if (!value.value("executable", true)) {throw std::invalid_argument("reference is not executable");}
      if (value.at("frame_id") != frame_id_ ||
        !fresh_stamp(stamp, now().nanoseconds(), path_timeout_))
      {
        follower_->clear(); active_path_generation_.reset(); pending_path_.clear();
        return;
      }
      std::vector<Point3> points;
      for (const auto & item : value.at("points")) {
        if (!item.is_array() || item.size() != 3) {throw std::invalid_argument("path point");}
        Point3 point{item[0], item[1], item[2]};
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          throw std::invalid_argument("nonfinite path point");
        }
        points.push_back(point);
      }
      pending_goal_exact_ = value.value("goal_exact", false);
      pending_goal_terminal_ = value.value("goal_terminal", false);
      if (pending_goal_exact_ && !pending_goal_terminal_) {
        throw std::invalid_argument("exact goal must be terminal");
      }
      pending_path_ = std::move(points);
      pending_path_generation_ = value.at("map_generation").get<std::int64_t>();
      path_map_stamp_ = value.at("map_stamp_ns");
      path_stamp_ = stamp;
      path_received_ = steady_seconds();
      // An empty path is an explicit veto. Never retain the previous route.
      const bool current_source = *pending_path_generation_ == map_generation_ &&
        path_map_stamp_ == map_stamp_;
      const bool historical_source = std::any_of(
        map_frames_.begin(), map_frames_.end(), [this](const MapFrame & frame) {
          return frame.generation == *pending_path_generation_ &&
                 frame.stamp_ns == path_map_stamp_;
        });
      if (pending_path_.size() < 2 || (!current_source && !historical_source)) {
        follower_->clear(); active_path_generation_.reset();
      }
      try_accept_path();
    } catch (const std::exception &) {
      follower_->clear(); active_path_generation_.reset(); pending_path_.clear();
    }
  }

  void on_snapshot(const std_msgs::msg::String & message)
  {
    try {
      const auto value = nlohmann::json::parse(message.data);
      const auto stamp = value.at("stamp_ns").get<std::int64_t>();
      if (value.at("frame_id") != frame_id_ || value.at("odom_frame_id") != vio_frame_id_ ||
        !fresh_stamp(stamp, now().nanoseconds(), std::min(pose_timeout_, vio_timeout_)) ||
        stamp <= pose_stamp_) {return;}
      const auto pose = snapshot_pose(value.at("pose"));
      snapshot_pose(value.at("raw_pose"));
      const auto correction = snapshot_pose(value.at("correction"));
      snapshot_correction_ = correction;
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = frame_id_; msg.header.stamp = rclcpp::Time(stamp);
      msg.pose.position.x = correction.position.x; msg.pose.position.y = correction.position.y;
      msg.pose.position.z = correction.position.z;
      msg.pose.orientation.x = correction.orientation.x; msg.pose.orientation.y = correction.orientation.y;
      msg.pose.orientation.z = correction.orientation.z; msg.pose.orientation.w = correction.orientation.w;
      pose_generation_ = value.at("map_generation"); pose_map_stamp_ = value.at("map_stamp_ns");
      on_correction(msg);
      vio_received_ = steady_seconds(); vio_stamp_ = stamp;
      msg.pose.position.x = pose.position.x; msg.pose.position.y = pose.position.y;
      msg.pose.position.z = pose.position.z;
      on_pose(msg);
    } catch (const std::exception &) {correction_.reset();}
  }

  void try_accept_path()
  {
    if (!pose_ || pending_path_.size() < 2 || !pending_path_generation_ || !octree_ ||
      (!snapshot_topic_.empty() && (pose_generation_ != map_generation_ || pose_map_stamp_ != map_stamp_)))
    {
      return;
    }
    if (accepted_path_stamp_ == path_stamp_ && active_path_generation_ &&
      *active_path_generation_ == map_generation_) {return;}
    if (!snapshot_topic_.empty()) {
      if (!correction_ || !map_correction_ || !snapshot_correction_ ||
        !same_snapshot_transform(*map_correction_, *snapshot_correction_)) {return;}
      if (configured_map_generation_ != map_generation_) {
        follower_ = std::make_unique<RouteFollower3D>(limits_for_map_tilt(config_, correction_->rotation[8]));
        configured_map_generation_ = map_generation_;
      }
    }
    std::vector<Point3> current_path;
    current_path.reserve(pending_path_.size());
    if (*pending_path_generation_ == map_generation_ && path_map_stamp_ == map_stamp_) {
      current_path = pending_path_;
    } else {
      if (!map_correction_) {return;}
      const auto source = std::find_if(
        map_frames_.begin(), map_frames_.end(), [this](const MapFrame & frame) {
          return frame.generation == *pending_path_generation_ &&
                 frame.stamp_ns == path_map_stamp_;
        });
      if (source == map_frames_.end()) {return;}
      for (const auto & point : pending_path_) {
        current_path.push_back(reframe_map_point_3d(
          point, source->map_from_odom, *map_correction_));
      }
    }
    for (std::size_t index = 1; index < current_path.size(); ++index) {
      if (!chord_clear(current_path[index - 1], current_path[index])) {
        follower_->clear();
        active_path_generation_.reset();
        return;
      }
    }
    if (active_path_generation_ && active_points_.size() == current_path.size() &&
      std::equal(active_points_.begin(), active_points_.end(), current_path.begin(),
        [](const Point3 & a, const Point3 & b) {return a.x == b.x && a.y == b.y && a.z == b.z;}))
    {
      active_goal_exact_ = pending_goal_exact_;
      active_goal_terminal_ = pending_goal_terminal_;
      accepted_path_stamp_ = path_stamp_;
      active_path_generation_ = map_generation_;
      return;
    }
    if (follower_->set_path(current_path, *pose_)) {
      active_points_ = std::move(current_path);
      active_goal_exact_ = pending_goal_exact_;
      active_goal_terminal_ = pending_goal_terminal_;
      accepted_path_stamp_ = path_stamp_;
      active_path_generation_ = map_generation_;
    }
  }

  void on_pose(const geometry_msgs::msg::PoseStamped & message)
  {
    const Point3 point{message.pose.position.x, message.pose.position.y, message.pose.position.z};
    if (message.header.frame_id != frame_id_ || !std::isfinite(point.x) ||
      !std::isfinite(point.y) || !std::isfinite(point.z) ||
      !fresh_stamp(rclcpp::Time(message.header.stamp).nanoseconds(), now().nanoseconds(), pose_timeout_))
    {
      return;
    }
    pose_stamp_ = rclcpp::Time(message.header.stamp).nanoseconds();
    pose_ = point;
    pose_received_ = steady_seconds();
    if (!takeoff_center_) {takeoff_center_ = point;}
    try_accept_path();
  }

  void on_correction(const geometry_msgs::msg::PoseStamped & message)
  {
    if (message.header.frame_id != frame_id_ ||
      !fresh_stamp(rclcpp::Time(message.header.stamp).nanoseconds(), now().nanoseconds(), correction_timeout_) ||
      !std::isfinite(message.pose.position.x) || !std::isfinite(message.pose.position.y) ||
      !std::isfinite(message.pose.position.z)) {correction_.reset(); return;}
    correction_stamp_ = rclcpp::Time(message.header.stamp).nanoseconds();
    correction_ = correction_from_pose(message);
    correction_received_ = steady_seconds();
  }

  void tick()
  {
    const auto tick_time = steady_seconds();
    const auto dt = std::clamp(tick_time - last_tick_, 0.001, 0.5);
    last_tick_ = tick_time;
    if (!octree_ || tick_time - map_received_ > map_timeout_ ||
      !fresh_stamp(map_stamp_, now().nanoseconds(), map_timeout_)) {invalid("MAP_STALE"); return;}
    if (!pose_ || tick_time - pose_received_ > pose_timeout_ ||
      !fresh_stamp(pose_stamp_, now().nanoseconds(), pose_timeout_)) {invalid("POSE_STALE"); return;}
    if (tick_time - vio_received_ > vio_timeout_ ||
      !fresh_stamp(vio_stamp_, now().nanoseconds(), vio_timeout_)) {invalid("VIO_STALE"); return;}
    if (!correction_ || tick_time - correction_received_ > correction_timeout_ ||
      !fresh_stamp(correction_stamp_, now().nanoseconds(), correction_timeout_)) {
      invalid("CORRECTION_STALE"); return;
    }
    if (!snapshot_topic_.empty() && (pose_generation_ != map_generation_ || pose_map_stamp_ != map_stamp_)) {
      invalid("POSE_MAP_MISMATCH"); return;
    }
    if (!active_path_generation_ || *active_path_generation_ != map_generation_ ||
      tick_time - path_received_ > path_timeout_ ||
      !fresh_stamp(path_stamp_, now().nanoseconds(), path_timeout_))
    {
      invalid("PATH_STALE"); return;
    }
    const auto translation = distance_3d(correction_->translation, {});
    if (!snapshot_topic_.empty() && (!map_correction_ || !snapshot_correction_ ||
      !same_snapshot_transform(*map_correction_, *snapshot_correction_)))
    {invalid("CORRECTION_MAP_MISMATCH"); return;}
    // Atomic mode already requires a fresh matched map, pose and path and
    // resets the follower on every map generation. A stable global frame
    // offset is not a persistent motion jump. Keep the legacy absolute guard
    // for independently delivered correction/pose topics.
    if (snapshot_topic_.empty() && (translation > max_correction_translation_ ||
      std::abs(correction_->yaw) > max_correction_yaw_ ||
      std::abs(correction_->roll) > max_correction_roll_pitch_ ||
      std::abs(correction_->pitch) > max_correction_roll_pitch_))
    {
      invalid("CORRECTION_LIMIT"); return;
    }
    const auto generation = map_generation_;
    const auto result = follower_->update(
      *pose_, dt, [this](const auto & start, const auto & end) {
        return chord_clear(start, end);
      });
    if (generation != map_generation_ || !result.valid) {
      invalid(generation != map_generation_ ? "MAP_GENERATION_RACE" : result.reason);
      return;
    }
    publish_vector(
      inverse_rotate(*correction_, result.displacement), displacement_pub_, vio_frame_id_);
    publish_vector(inverse_rotate(*correction_, result.velocity), velocity_pub_, vio_frame_id_);
    publish_vector(
      inverse_rotate(*correction_, result.acceleration), acceleration_pub_, vio_frame_id_);
    publish_pose(result.carrot, carrot_pub_);
    publish_pose(result.lookahead, lookahead_pub_);
    std_msgs::msg::Bool valid; valid.data = true; valid_pub_->publish(valid);
    std_msgs::msg::Bool reached;
    reached.data = result.reached && active_goal_exact_;
    reached_pub_->publish(reached);
    auto reason = result.reason;
    if (result.reached && !active_goal_exact_) {
      reason = active_goal_terminal_ ? "SAFE_APPROACH_REACHED" : "EXPLORATION_ENDPOINT_REACHED";
    }
    publish_status(reason, result);
  }

  bool chord_clear(const Point3 & start, const Point3 & end) const
  {
    if (!inside_limits(start) || !inside_limits(end)) {return false;}
    const auto resolution = octree_->getResolution();
    const auto padding = required_clearance_ + 2.0 * resolution;
    const auto align = [resolution](double value) {
        return std::floor(value / resolution) * resolution;
      };
    const Point3 origin{
      align(std::min(start.x, end.x) - padding),
      align(std::min(start.y, end.y) - padding),
      align(std::min(start.z, end.z) - padding)};
    const auto dimension = [resolution](double lower, double upper) {
        return static_cast<std::size_t>(std::ceil((upper - lower) / resolution));
      };
    const auto width = dimension(origin.x, std::max(start.x, end.x) + padding);
    const auto height = dimension(origin.y, std::max(start.y, end.y) + padding);
    const auto depth = dimension(origin.z, std::max(start.z, end.z) + padding);
    if (width * height * depth > 2000000ULL) {return false;}
    VoxelGrid grid(width, height, depth, resolution, origin);
    if (const auto * tree = dynamic_cast<const octomap::OcTree *>(octree_.get())) {
      fill_native_octomap_grid(*tree, grid);
    } else if (const auto * tree = dynamic_cast<const octomap::ColorOcTree *>(octree_.get())) {
      fill_native_octomap_grid(*tree, grid);
    }
    return swept_sphere_clear(grid, start, end, required_clearance_);
  }

  bool inside_limits(const Point3 & point) const
  {
    return takeoff_center_ && point.z >= min_z_ + required_clearance_ &&
           point.z <= max_z_ - required_clearance_ &&
           std::hypot(point.x - takeoff_center_->x, point.y - takeoff_center_->y) +
           required_clearance_ <= planning_radius_xy_;
  }

  std::optional<VoxelState> octomap_state(const Point3 & point) const
  {
    if (const auto * tree = dynamic_cast<const octomap::OcTree *>(octree_.get())) {
      const auto * node = tree->search(point.x, point.y, point.z);
      if (node == nullptr) {return std::nullopt;}
      return tree->isNodeOccupied(node) ? VoxelState::Occupied : VoxelState::Free;
    }
    if (const auto * tree = dynamic_cast<const octomap::ColorOcTree *>(octree_.get())) {
      const auto * node = tree->search(point.x, point.y, point.z);
      if (node == nullptr) {return std::nullopt;}
      return tree->isNodeOccupied(node) ? VoxelState::Occupied : VoxelState::Free;
    }
    return std::nullopt;
  }

  void invalid(const std::string & reason)
  {
    // Resuming after a watchdog must start from the current pose with zero
    // internal feedforward, not revive the pre-fault carrot/velocity.
    follower_->clear();
    active_path_generation_.reset();
    publish_vector({}, displacement_pub_, vio_frame_id_);
    publish_vector({}, velocity_pub_, vio_frame_id_);
    publish_vector({}, acceleration_pub_, vio_frame_id_);
    std_msgs::msg::Bool valid; valid.data = false; valid_pub_->publish(valid);
    std_msgs::msg::Bool reached; reached.data = false; reached_pub_->publish(reached);
    FollowResult3D empty;
    publish_status(reason, empty);
  }

  void publish_vector(
    const Point3 & point,
    const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr & publisher,
    const std::string & frame)
  {
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = now();
    message.header.frame_id = frame;
    message.vector.x = point.x; message.vector.y = point.y; message.vector.z = point.z;
    publisher->publish(message);
  }

  void publish_pose(
    const Point3 & point,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now(); message.header.frame_id = frame_id_;
    message.pose.position.x = point.x; message.pose.position.y = point.y;
    message.pose.position.z = point.z; message.pose.orientation.w = 1.0;
    publisher->publish(message);
  }

  void publish_status(const std::string & reason, const FollowResult3D & result)
  {
    nlohmann::ordered_json status;
    status["state"] = reason;
    status["observation_only"] = true;
    status["map_generation"] = map_generation_;
    status["map_stamp_ns"] = map_stamp_;
    status["pose_stamp_ns"] = pose_stamp_;
    status["pose_map_generation"] = snapshot_topic_.empty() ? map_generation_ : pose_generation_;
    status["path_stamp_ns"] = path_stamp_;
    status["pose"] = pose_ ? nlohmann::json{pose_->x, pose_->y, pose_->z} : nlohmann::json();
    status["carrot"] = {result.carrot.x, result.carrot.y, result.carrot.z};
    status["lookahead"] = {result.lookahead.x, result.lookahead.y, result.lookahead.z};
    status["lethal_radius"] = required_clearance_;
    status["correction_policy"] = snapshot_topic_.empty() ? "absolute_limits" : "matched_map_generation";
    status["path_map_generation"] = active_path_generation_.value_or(0);
    status["cross_track"] = result.projection.horizontal_distance;
    status["vertical_track"] = result.projection.vertical_distance;
    status["remaining"] = result.remaining;
    status["goal_exact"] = active_goal_exact_;
    status["goal_terminal"] = active_goal_terminal_;
    status["path_endpoint_reached"] = result.reached;
    status["requested_goal_reached"] = result.reached && active_goal_exact_;
    std_msgs::msg::String message; message.data = status.dump(); status_pub_->publish(message);
  }

  ProcessSingleton singleton_;
  std::string frame_id_;
  std::string vio_frame_id_;
  std::string path_snapshot_topic_;
  std::string map_topic_;
  std::string metadata_topic_;
  std::string pose_topic_;
  std::string snapshot_topic_;
  std::int64_t pose_stamp_{}, vio_stamp_{}, correction_stamp_{}, map_stamp_{};
  std::int64_t pose_generation_{}, pose_map_stamp_{}, path_stamp_{}, path_map_stamp_{}, accepted_path_stamp_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr snapshot_sub_, path_snapshot_sub_;
  std::string raw_vio_topic_;
  std::string correction_topic_;
  double path_timeout_{};
  double map_timeout_{};
  double pose_timeout_{};
  double vio_timeout_{};
  double correction_timeout_{};
  double planning_radius_xy_{};
  double min_z_{};
  double max_z_{};
  double required_clearance_{};
  std::optional<Pose3> map_correction_, snapshot_correction_;
  std::deque<MapFrame> map_frames_;
  std::int64_t configured_map_generation_{};
  double max_correction_translation_{};
  double max_correction_yaw_{};
  double max_correction_roll_pitch_{};
  Follower3DConfig config_;
  std::unique_ptr<RouteFollower3D> follower_;
  std::unique_ptr<octomap::AbstractOcTree> octree_;
  std::unique_ptr<octomap::AbstractOcTree> pending_octree_;
  std::optional<MapMetadata> pending_metadata_;
  rclcpp::Time pending_map_stamp_{0, 0, RCL_ROS_TIME};
  std::int64_t map_generation_{};
  std::optional<std::int64_t> pending_path_generation_;
  std::optional<std::int64_t> active_path_generation_;
  std::vector<Point3> pending_path_;
  std::vector<Point3> active_points_;
  bool pending_goal_exact_{false}, pending_goal_terminal_{false};
  bool active_goal_exact_{false}, active_goal_terminal_{false};
  std::optional<Point3> pose_;
  std::optional<Point3> takeoff_center_;
  std::optional<Correction3D> correction_;
  double map_received_{};
  double path_received_{};
  double pose_received_{};
  double vio_received_{};
  double correction_received_{};
  double last_tick_{};
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vio_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr correction_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr displacement_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<px4_vio_bridge::RouteFollower3DNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "route_follower_3d_monitor: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
