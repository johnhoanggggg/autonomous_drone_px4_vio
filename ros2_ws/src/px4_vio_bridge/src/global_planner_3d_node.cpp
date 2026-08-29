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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

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
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr message) {on_map(*message);});
    metadata_sub_ = create_subscription<std_msgs::msg::String>(
      map_metadata_topic_, map_qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_metadata(*message);});
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {on_pose(*message);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {on_goal(*message);});

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner3d/path", 10);
    candidate_pub_ = create_publisher<nav_msgs::msg::Path>("/planner3d/candidate_path", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planner3d/status", 10);
    map_generation_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/planner3d/map_generation", map_qos);
    path_map_generation_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/planner3d/path_map_generation", map_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planner3d/markers", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.2, rate_hz)), [this]() {tick();});
    RCLCPP_WARN(
      get_logger(),
      "3D planner is OBSERVATION ONLY: no PX4 publishers; unknown=blocked radius=%.2fm",
      config_.lethal_radius);
  }

private:
  void validate_config()
  {
    if (!std::isfinite(voxel_size_) || voxel_size_ <= 0.0 ||
      !std::isfinite(robot_radius_) || robot_radius_ <= 0.0 ||
      !std::isfinite(safety_margin_) || safety_margin_ < 0.10 ||
      !std::isfinite(max_cross_track_) || max_cross_track_ < 0.0 ||
      safety_margin_ + 1.0e-12 < max_cross_track_ ||
      !std::isfinite(planning_radius_xy_) || planning_radius_xy_ <= 0.0 ||
      !std::isfinite(min_z_) || !std::isfinite(max_z_) || min_z_ >= max_z_ ||
      !std::isfinite(inflation_extra_) || inflation_extra_ < 0.0)
    {
      throw std::invalid_argument(
              "invalid 3D configuration (requires safety_margin>=0.10 and "
              "safety_margin>=max_cross_track)");
    }
  }

  void on_map(const octomap_msgs::msg::Octomap & message)
  {
    if (message.header.frame_id != frame_id_) {
      publish_status("MAP_FRAME_REJECTED");
      return;
    }
    if (!std::isfinite(message.resolution) || message.resolution <= 0.0 ||
      message.data.empty())
    {
      publish_status("MAP_GEOMETRY_REJECTED");
      return;
    }
    std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(message));
    if (dynamic_cast<octomap::OcTree *>(abstract.get()) == nullptr &&
      dynamic_cast<octomap::ColorOcTree *>(abstract.get()) == nullptr)
    {
      publish_status("MAP_TYPE_REJECTED");
      return;
    }
    pending_octree_ = std::move(abstract);
    pending_map_stamp_ = rclcpp::Time(message.header.stamp);
    if (!require_map_metadata_) {
      pending_metadata_ = MapMetadata{
        map_generation_ + 1, pending_map_stamp_.nanoseconds(), message.resolution,
        message.header.frame_id, "unpaired"};
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
        json.at("source_pose_generation").get<std::string>()};
      try_accept_map();
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "map metadata rejected: %s", error.what());
      publish_status("MAP_METADATA_REJECTED");
    }
  }

  void try_accept_map()
  {
    if (!pending_octree_ || !pending_metadata_) {return;}
    if (pending_metadata_->stamp_ns != pending_map_stamp_.nanoseconds() ||
      pending_metadata_->frame_id != frame_id_ ||
      std::abs(pending_metadata_->resolution - pending_octree_->getResolution()) > 1.0e-9 ||
      pending_metadata_->generation <= 0)
    {
      publish_status("MAP_METADATA_MISMATCH");
      return;
    }
    octree_ = std::move(pending_octree_);
    map_stamp_ = pending_map_stamp_;
    map_received_ = steady_seconds();
    map_generation_ = pending_metadata_->generation;
    source_pose_generation_ = pending_metadata_->source_pose_generation;
    pending_metadata_.reset();
    std_msgs::msg::Int32 generation;
    generation.data = static_cast<std::int32_t>(map_generation_);
    map_generation_pub_->publish(generation);
    plan_dirty_ = true;
  }

  void on_pose(const geometry_msgs::msg::PoseStamped & message)
  {
    const Point3 point{message.pose.position.x, message.pose.position.y, message.pose.position.z};
    if (message.header.frame_id != frame_id_ || !finite(point)) {
      publish_status("POSE_REJECTED");
      return;
    }
    pose_ = point;
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
    goal_ = point;  // Preserve clicked Z exactly.
    ++goal_generation_;
    plan_dirty_ = true;
  }

  void tick()
  {
    const auto now = steady_seconds();
    if (!octree_ || now - map_received_ > map_timeout_) {fail_closed("MAP_STALE"); return;}
    if (!pose_ || now - pose_received_ > pose_timeout_) {fail_closed("POSE_STALE"); return;}
    if (!goal_) {fail_closed("WAITING_FOR_GOAL"); return;}
    if (!takeoff_center_) {fail_closed("WAITING_FOR_TAKEOFF_ORIGIN"); return;}
    if (!inside_geofence(*goal_)) {fail_closed("GOAL_OUTSIDE_GEOFENCE"); return;}
    if (!plan_dirty_) {
      publish_path(accepted_path_, path_pub_);
      std_msgs::msg::Int32 path_generation;
      path_generation.data = static_cast<std::int32_t>(path_map_generation_);
      path_map_generation_pub_->publish(path_generation);
      return;
    }

    const auto generation = map_generation_;
    auto raw = make_planning_grid();
    if (!raw) {fail_closed("MAP_CROP_INVALID"); return;}
    auto result = plan_path_3d(*raw, *pose_, *goal_, config_);
    if (generation != map_generation_) {fail_closed("MAP_GENERATION_RACE"); return;}
    publish_candidate(result.path);
    publish_markers(*raw, result);
    if (!result.found()) {
      fail_closed(result.reason);
      return;
    }
    publish_path(result.path, path_pub_);
    accepted_path_ = result.path;
    last_planned_pose_ = pose_;
    path_map_generation_ = generation;
    std_msgs::msg::Int32 path_generation;
    path_generation.data = static_cast<std::int32_t>(path_map_generation_);
    path_map_generation_pub_->publish(path_generation);
    plan_dirty_ = false;
    publish_result_status(result, raw->resolution());
  }

  bool inside_geofence(const Point3 & point) const
  {
    return takeoff_center_ &&
           std::hypot(point.x - takeoff_center_->x, point.y - takeoff_center_->y) <=
           planning_radius_xy_ + 1.0e-12 && point.z >= min_z_ && point.z <= max_z_;
  }

  std::optional<VoxelGrid> make_planning_grid() const
  {
    const auto resolution = octree_->getResolution();
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
    for (int z = 0; z < static_cast<int>(depth); ++z) {
      for (int y = 0; y < static_cast<int>(height); ++y) {
        for (int x = 0; x < static_cast<int>(width); ++x) {
          const Voxel voxel{x, y, z};
          const auto center = grid.voxel_center(voxel);
          // The cylinder is the takeoff-centred XY geofence. Cells outside it
          // stay unknown and are therefore hard obstacles.
          if (std::hypot(
              center.x - takeoff_center_->x, center.y - takeoff_center_->y) >
            planning_radius_xy_)
          {
            continue;
          }
          const auto state = octomap_state(center);
          if (state) {grid.set(voxel, *state);}
        }
      }
    }
    return grid;
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
  }

  void publish_candidate(const std::vector<Point3> & points)
  {
    publish_path(points, candidate_pub_);
  }

  void fail_closed(const std::string & reason)
  {
    accepted_path_.clear();
    publish_path({}, path_pub_);
    publish_status(reason);
  }

  void publish_status(const std::string & reason)
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
    message.data = status.dump();
    status_pub_->publish(message);
  }

  void publish_result_status(const PlanResult3D & result, double resolution)
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
    std_msgs::msg::String message;
    message.data = status.dump();
    status_pub_->publish(message);
  }

  void publish_markers(const VoxelGrid & raw, const PlanResult3D & result)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
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
    markers_pub_->publish(array);
  }

  ProcessSingleton singleton_;
  std::string map_topic_;
  std::string map_metadata_topic_;
  std::string pose_topic_;
  std::string goal_topic_;
  std::string frame_id_;
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
  std::size_t max_marker_voxels_{};
  bool require_map_metadata_{};
  Planner3DConfig config_;
  std::unique_ptr<octomap::AbstractOcTree> octree_;
  std::unique_ptr<octomap::AbstractOcTree> pending_octree_;
  std::optional<MapMetadata> pending_metadata_;
  rclcpp::Time pending_map_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<Point3> pose_;
  std::optional<Point3> goal_;
  std::optional<Point3> takeoff_center_;
  std::optional<Point3> last_planned_pose_;
  std::vector<Point3> accepted_path_;
  double map_received_{};
  double pose_received_{};
  std::int64_t map_generation_{};
  std::int64_t path_map_generation_{};
  std::int64_t goal_generation_{};
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME};
  std::string source_pose_generation_;
  bool plan_dirty_{true};
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr map_generation_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr path_map_generation_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
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
