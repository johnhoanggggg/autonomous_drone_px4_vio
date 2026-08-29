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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "px4_vio_bridge/grid_planner_3d.hpp"
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
};

struct Correction3D
{
  std::array<double, 9> rotation{};
  Point3 translation{};
  double roll{};
  double pitch{};
  double yaw{};
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
    path_topic_ = declare_parameter<std::string>("path_topic", "/planner3d/path");
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
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {on_path(*message);});
    path_generation_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/planner3d/path_map_generation", qos,
      [this](std_msgs::msg::Int32::ConstSharedPtr message) {
        pending_path_generation_ = message->data;
        try_accept_path();
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {on_pose(*message);});
    vio_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      raw_vio_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {
        vio_received_ = steady_seconds();
      });
    correction_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      correction_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        on_correction(*message);
      });
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
        json.at("resolution").get<double>(), json.at("frame_id").get<std::string>()};
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
      std::abs(pending_metadata_->resolution - pending_octree_->getResolution()) > 1.0e-9)
    {
      return;
    }
    octree_ = std::move(pending_octree_);
    map_generation_ = pending_metadata_->generation;
    pending_metadata_.reset();
    map_received_ = steady_seconds();
    follower_->clear();
    active_path_generation_.reset();
    try_accept_path();
  }

  void on_path(const nav_msgs::msg::Path & message)
  {
    pending_path_.clear();
    if (message.header.frame_id != frame_id_ || message.poses.size() < 2) {return;}
    for (const auto & pose : message.poses) {
      const Point3 point{pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        pending_path_.clear();
        return;
      }
      pending_path_.push_back(point);
    }
    path_received_ = steady_seconds();
    try_accept_path();
  }

  void try_accept_path()
  {
    if (!pose_ || pending_path_.size() < 2 || !pending_path_generation_ ||
      *pending_path_generation_ != map_generation_ || !octree_)
    {
      return;
    }
    if (follower_->set_path(pending_path_, *pose_)) {
      active_path_generation_ = pending_path_generation_;
    }
  }

  void on_pose(const geometry_msgs::msg::PoseStamped & message)
  {
    const Point3 point{message.pose.position.x, message.pose.position.y, message.pose.position.z};
    if (message.header.frame_id != frame_id_ || !std::isfinite(point.x) ||
      !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      return;
    }
    pose_ = point;
    pose_received_ = steady_seconds();
    if (!takeoff_center_) {takeoff_center_ = point;}
    try_accept_path();
  }

  void on_correction(const geometry_msgs::msg::PoseStamped & message)
  {
    correction_ = correction_from_pose(message);
    correction_received_ = steady_seconds();
  }

  void tick()
  {
    const auto tick_time = steady_seconds();
    const auto dt = std::clamp(tick_time - last_tick_, 0.001, 0.5);
    last_tick_ = tick_time;
    if (!octree_ || tick_time - map_received_ > map_timeout_) {invalid("MAP_STALE"); return;}
    if (!pose_ || tick_time - pose_received_ > pose_timeout_) {invalid("POSE_STALE"); return;}
    if (tick_time - vio_received_ > vio_timeout_) {invalid("VIO_STALE"); return;}
    if (!correction_ || tick_time - correction_received_ > correction_timeout_) {
      invalid("CORRECTION_STALE"); return;
    }
    if (!active_path_generation_ || *active_path_generation_ != map_generation_ ||
      tick_time - path_received_ > path_timeout_)
    {
      invalid("PATH_STALE"); return;
    }
    const auto translation = distance_3d(correction_->translation, {});
    if (translation > max_correction_translation_ ||
      std::abs(correction_->yaw) > max_correction_yaw_ ||
      std::abs(correction_->roll) > max_correction_roll_pitch_ ||
      std::abs(correction_->pitch) > max_correction_roll_pitch_)
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
    std_msgs::msg::Bool reached; reached.data = result.reached; reached_pub_->publish(reached);
    publish_status(result.reason, result);
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
    for (int z = 0; z < static_cast<int>(depth); ++z) {
      for (int y = 0; y < static_cast<int>(height); ++y) {
        for (int x = 0; x < static_cast<int>(width); ++x) {
          const Voxel voxel{x, y, z};
          const auto state = octomap_state(grid.voxel_center(voxel));
          if (state) {grid.set(voxel, *state);}
        }
      }
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
    status["path_map_generation"] = active_path_generation_.value_or(0);
    status["cross_track"] = result.projection.horizontal_distance;
    status["vertical_track"] = result.projection.vertical_distance;
    status["remaining"] = result.remaining;
    std_msgs::msg::String message; message.data = status.dump(); status_pub_->publish(message);
  }

  ProcessSingleton singleton_;
  std::string frame_id_;
  std::string vio_frame_id_;
  std::string path_topic_;
  std::string map_topic_;
  std::string metadata_topic_;
  std::string pose_topic_;
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
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr path_generation_sub_;
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
