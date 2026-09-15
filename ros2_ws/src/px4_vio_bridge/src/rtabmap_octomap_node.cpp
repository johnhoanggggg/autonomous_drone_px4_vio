#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "px4_vio_bridge/rtabmap_octomap.hpp"
#include "px4_vio_bridge/navigation_snapshot.hpp"
#include "px4_vio_bridge/test_start_clear.hpp"

namespace px4_vio_bridge
{

class RtabmapOctomapNode final : public rclcpp::Node
{
public:
  RtabmapOctomapNode()
  : Node("rtabmap_octomap")
  {
    map_data_topic_ = declare_parameter<std::string>("map_data_topic", "/rtabmap/mapData");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const auto octomap_topic = declare_parameter<std::string>(
      "octomap_topic", "/rtabmap/octomap");
    const auto metadata_topic = declare_parameter<std::string>(
      "metadata_topic", "/rtabmap/octomap_metadata");
    const auto markers_topic = declare_parameter<std::string>(
      "markers_topic", "/rtabmap/octomap_markers");
    const auto ground_markers_topic = declare_parameter<std::string>(
      "ground_markers_topic", "/rtabmap/octomap_ground_markers");
    const auto obstacle_markers_topic = declare_parameter<std::string>(
      "obstacle_markers_topic", "/rtabmap/octomap_obstacle_markers");
    const auto max_marker_voxels = declare_parameter<int>("max_marker_voxels", 0);
    pose_update_tolerance_m_ = declare_parameter<double>("pose_update_tolerance_m", 0.01);
    pose_update_tolerance_rad_ = declare_parameter<double>("pose_update_tolerance_rad", 0.01);
    use_checkpoints_ = declare_parameter<bool>("octomap_use_checkpoints", true);
    assembler_.set_use_checkpoints(use_checkpoints_);
    if (max_marker_voxels < 0) {
      throw std::invalid_argument("max_marker_voxels cannot be negative");
    }
    if (!std::isfinite(pose_update_tolerance_m_) || pose_update_tolerance_m_ < 0.0 ||
      !std::isfinite(pose_update_tolerance_rad_) || pose_update_tolerance_rad_ < 0.0)
    {
      throw std::invalid_argument("pose update tolerances must be finite and non-negative");
    }
    max_marker_voxels_ = static_cast<std::size_t>(max_marker_voxels);
    test_clear_radius_ = declare_parameter<double>("test_start_clear_radius", 0.0);
    test_offset_ = {declare_parameter<double>("camera_position_frd_x", 0.100),
      declare_parameter<double>("camera_position_frd_y", -0.036),
      declare_parameter<double>("camera_position_frd_z", 0.056)};
    if (!std::isfinite(test_clear_radius_) || test_clear_radius_ < 0.0 || test_clear_radius_ > 0.5 ||
      !std::isfinite(test_offset_.x) || !std::isfinite(test_offset_.y) || !std::isfinite(test_offset_.z) ||
      std::hypot(test_offset_.x, test_offset_.y, test_offset_.z) > 1.0)
    {throw std::invalid_argument("invalid test start clearing configuration");}
    if (test_clear_radius_ > 0.0) {
      RCLCPP_WARN(get_logger(), "TEST MAP: forcing a %.2f m start sphere free; re-anchor on each fresh waypoint", test_clear_radius_);
      test_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/rtabmap/vio_odometry", rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (stamp <= test_latest_stamp_) {return;}
          test_latest_raw_.reset();
          if (msg->header.frame_id != "rtabmap_odom" || msg->child_frame_id != "oak" ||
            !fresh_stamp(stamp, now().nanoseconds(), 0.5) ||
            !std::isfinite(msg->pose.covariance[0]) || msg->pose.covariance[0] < 0.0 ||
            msg->pose.covariance[0] >= 9999.0) {return;}
          try {
            const auto & p = msg->pose.pose.position; const auto & q = msg->pose.pose.orientation;
            test_latest_raw_ = camera_to_body(snapshot_pose({p.x, p.y, p.z, q.x, q.y, q.z, q.w}), test_offset_);
            test_latest_stamp_ = stamp;
            if (!test_start_raw_) {
              test_start_raw_ = test_latest_raw_;
              test_start_stamp_ = stamp;
              test_anchor_sequence_ = 1;
            }
          } catch (const std::exception &) {return;}
        });
      test_goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/waypoint/clicked", 10,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (msg->header.frame_id != frame_id_ || !std::isfinite(msg->point.x) ||
            !std::isfinite(msg->point.y) || !std::isfinite(msg->point.z) ||
            stamp <= test_goal_stamp_ || !fresh_stamp(stamp, now().nanoseconds(), 0.5)) {return;}
          if (!test_latest_raw_ || !fresh_stamp(test_latest_stamp_, now().nanoseconds(), 0.5)) {
            RCLCPP_WARN(get_logger(), "Test start sphere unchanged: waypoint has no fresh valid body pose");
            return;
          }
          // One bounded test volume per waypoint request, fixed between clicks.
          // The next source map publishes a new generation using this anchor;
          // old artificial clearing is not accumulated in the sensor tree.
          test_start_raw_ = test_latest_raw_;
          test_start_stamp_ = test_latest_stamp_;
          test_goal_stamp_ = stamp;
          ++test_anchor_sequence_;
        });
    }
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    // ROS RTAB-Map's live MapData publisher is volatile, while the deterministic
    // fixture is transient-local. A volatile reliable subscription is compatible
    // with both offered durability policies.
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(octomap_topic, output_qos);
    metadata_pub_ = create_publisher<std_msgs::msg::String>(metadata_topic, output_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      markers_topic, output_qos);
    ground_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      ground_markers_topic, output_qos);
    obstacle_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      obstacle_markers_topic, output_qos);
    test_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planner3d/test_start_clear", output_qos);
    map_data_sub_ = create_subscription<rtabmap_msgs::msg::MapData>(
      map_data_topic_, input_qos,
      [this](rtabmap_msgs::msg::MapData::ConstSharedPtr message) {on_map_data(*message);});
    RCLCPP_INFO(
      get_logger(), "waiting for RTAB-Map keyframe grids on %s", map_data_topic_.c_str());
  }

private:
  void on_map_data(const rtabmap_msgs::msg::MapData & message)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    auto stage_start = processing_start;
    nlohmann::ordered_json timing;
    const auto stage = [&](const std::string & name) {
        const auto end = std::chrono::steady_clock::now();
        timing[name] = std::chrono::duration<double, std::milli>(end - stage_start).count();
        stage_start = end;
      };
    const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
    if (stamp <= last_stamp_) {return;}
    // Map and raw odometry stamps may differ slightly for the same frame.
    // The raw anchor lives in continuous odometry coordinates; apply this
    // map's correction even when its stamp is just before the anchor's stamp.
    if (test_clear_radius_ > 0.0 && !test_start_raw_) {return;}
    const auto & t = message.graph.map_to_odom;
    const auto norm = std::hypot(std::hypot(t.rotation.x, t.rotation.y),
      std::hypot(t.rotation.z, t.rotation.w));
    if (message.header.frame_id != frame_id_ ||
      !std::isfinite(t.translation.x) || !std::isfinite(t.translation.y) ||
      !std::isfinite(t.translation.z) || !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "MapData frame/correction rejected");
      return;
    }
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    std::map<int, rtabmap::Signature> signatures;
    rtabmap::Transform map_to_odom;
    rtabmap_conversions::mapDataFromROS(message, poses, links, signatures, map_to_odom);
    stage("conversion_ms");
    // CoreWrapper publishes the complete optimized pose graph but may attach
    // only newly added/retrieved node data to each MapData update. Cache just
    // the uncompressed local grids (not RGB-D descriptors) so every rebuild can
    // reapply all observations at the latest loop-corrected poses.
    for (const auto & item : signatures) {
      const auto & data = item.second.sensorData();
      cv::Mat ground;
      cv::Mat obstacles;
      cv::Mat empty;
      data.uncompressDataConst(nullptr, nullptr, nullptr, nullptr, &ground, &obstacles, &empty);
      if (ground.empty() && obstacles.empty() && empty.empty()) {
        continue;
      }
      RCLCPP_DEBUG(
        get_logger(),
        "node %d local grids: ground type=%d %dx%d, obstacles type=%d %dx%d, empty type=%d %dx%d",
        item.first, ground.type(), ground.rows, ground.cols,
        obstacles.type(), obstacles.rows, obstacles.cols,
        empty.type(), empty.rows, empty.cols);
      observations_[item.first] = {
        item.first, ground.clone(), obstacles.clone(), empty.clone(),
        data.gridCellSize(), data.gridViewPoint()};
    }
    if (!poses.empty()) {
      optimized_poses_ = std::move(poses);
    }
    std::vector<LocalGridObservation> observations;
    observations.reserve(optimized_poses_.size());
    for (const auto & pose : optimized_poses_) {
      const auto observation = observations_.find(pose.first);
      if (observation != observations_.end()) {
        observations.push_back(observation->second);
      }
    }
    std::string error;
    stage("local_grids_ms");
    if (!assembler_.update(
        observations, optimized_poses_, static_cast<float>(pose_update_tolerance_m_),
        static_cast<float>(pose_update_tolerance_rad_), &error))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "OctoMap generation rejected: %s", error.c_str());
      return;
    }
    stage("assembly_ms");
    octomap_msgs::msg::Octomap octomap_message;
    octomap_message.header = message.header;
    octomap_message.header.frame_id = frame_id_;
    std::unique_ptr<rtabmap::RtabmapColorOcTree> test_tree;
    const auto * output_tree = assembler_.tree();
    nlohmann::ordered_json test_clear = {{"enabled", false}};
    if (test_clear_radius_ > 0.0) {
      const auto & t = message.graph.map_to_odom;
      const auto corrected = apply_correction(snapshot_pose({t.translation.x, t.translation.y,
        t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w}), *test_start_raw_);
      const Point3 center{corrected.position.x, corrected.position.y, corrected.position.z};
      test_tree = clone_rtabmap_tree(*assembler_.tree());
      const auto cells = clear_test_start_sphere(*test_tree, center, test_clear_radius_);
      output_tree = test_tree.get();
      test_clear = {{"enabled", true}, {"radius", test_clear_radius_},
        {"center", {center.x, center.y, center.z}}, {"anchor_frame", "rtabmap_odom"},
        {"anchor_stamp_ns", test_start_stamp_}, {"raw_body_pose", pose_json(*test_start_raw_)},
        {"anchor_sequence", test_anchor_sequence_},
        {"anchor_reason", test_goal_stamp_ > 0 ? "waypoint" : "startup"},
        {"goal_stamp_ns", test_goal_stamp_},
        {"cleared_voxels", cells}, {"observed_free", false}};
    }
    stage("test_clear_ms");
    if (!octomap_msgs::binaryMapToMsg(*output_tree, octomap_message)) {
      RCLCPP_ERROR(get_logger(), "failed to serialize RTAB-Map OctoMap");
      return;
    }
    stage("serialization_ms");
    last_stamp_ = stamp;
    ++generation_;
    const auto & built = assembler_.metadata();
    const auto displayed_marker_voxels = publish_markers(octomap_message.header, output_tree);
    visualization_msgs::msg::MarkerArray test_markers;
    visualization_msgs::msg::Marker bubble;
    bubble.header = octomap_message.header;
    bubble.ns = "test_start_clear";
    bubble.id = 0;
    bubble.action = test_clear_radius_ > 0.0 ? bubble.ADD : bubble.DELETEALL;
    if (test_clear_radius_ > 0.0) {
      bubble.type = bubble.SPHERE;
      bubble.pose.position.x = test_clear["center"][0];
      bubble.pose.position.y = test_clear["center"][1];
      bubble.pose.position.z = test_clear["center"][2];
      bubble.pose.orientation.w = 1.0;
      bubble.scale.x = bubble.scale.y = bubble.scale.z = 2.0 * test_clear_radius_;
      bubble.color.r = 0.2F; bubble.color.g = 0.8F; bubble.color.b = 1.0F; bubble.color.a = 0.2F;
    }
    test_markers.markers.push_back(bubble);
    test_marker_pub_->publish(test_markers);
    stage("markers_ms");
    nlohmann::ordered_json metadata;
    metadata["generation"] = generation_;
    metadata["source_pose_generation"] = std::to_string(built.source_pose_generation);
    metadata["stamp_ns"] = rclcpp::Time(octomap_message.header.stamp).nanoseconds();
    metadata["frame_id"] = frame_id_;
    const auto & transform = message.graph.map_to_odom;
    metadata["map_from_odom"] = {
      transform.translation.x, transform.translation.y, transform.translation.z,
      transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w};
    metadata["resolution"] = built.resolution;
    double min_x, min_y, min_z, max_x, max_y, max_z;
    output_tree->getMetricMin(min_x, min_y, min_z);
    output_tree->getMetricMax(max_x, max_y, max_z);
    metadata["bounds"] = {
      {"min", {min_x, min_y, min_z}},
      {"max", {max_x, max_y, max_z}}};
    metadata["source_nodes"] = built.source_nodes;
    metadata["ground_cells"] = built.ground_cells;
    metadata["obstacle_cells"] = built.obstacle_cells;
    metadata["empty_cells"] = built.empty_cells;
    metadata["displayed_marker_voxels"] = displayed_marker_voxels;
    metadata["ground_is_obstacle"] = true;
    metadata["ray_tracing"] = true;
    metadata["test_start_clear"] = test_clear;
    metadata["processing_ms"] = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    metadata["timing"] = timing;
    metadata["checkpoint_restores"] = assembler_.checkpoint_restores();
    metadata["full_rebuilds"] = assembler_.full_rebuilds();
    metadata["checkpoints_enabled"] = use_checkpoints_;
    std_msgs::msg::String metadata_message;
    metadata_message.data = metadata.dump();
    // The planner pairs by stamp and accepts either arrival order.
    metadata_pub_->publish(metadata_message);
    octomap_pub_->publish(octomap_message);
  }

  std::size_t publish_markers(const std_msgs::msg::Header & header,
    const rtabmap::RtabmapColorOcTree * tree)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::MarkerArray ground_array;
    visualization_msgs::msg::MarkerArray obstacle_array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    ground_array.markers.push_back(clear);
    obstacle_array.markers.push_back(clear);
    if (max_marker_voxels_ == 0 || tree == nullptr) {
      markers_pub_->publish(array);
      ground_markers_pub_->publish(ground_array);
      obstacle_markers_pub_->publish(obstacle_array);
      return 0;
    }

    std::size_t occupied_count = 0;
    for (auto iter = tree->begin_leafs(); iter != tree->end_leafs(); ++iter)
    {
      occupied_count += tree->isNodeOccupied(*iter) ? 1U : 0U;
    }
    const auto stride = std::max<std::size_t>(
      1, (occupied_count + max_marker_voxels_ - 1) / max_marker_voxels_);
    const auto make_marker = [&](int id, const std::string & name) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = name;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z =
          tree->getResolution() * 0.98;
        marker.color.a = 0.72F;
        return marker;
      };
    auto ground = make_marker(1, "octomap_ground");
    ground.color.r = 0.55F;
    ground.color.g = 0.34F;
    ground.color.b = 0.12F;
    auto obstacles = make_marker(2, "octomap_obstacles");
    obstacles.color.r = 0.92F;
    obstacles.color.g = 0.16F;
    obstacles.color.b = 0.10F;
    std::size_t displayed = 0;
    std::size_t occupied_index = 0;
    for (auto iter = tree->begin_leafs(); iter != tree->end_leafs(); ++iter)
    {
      if (!tree->isNodeOccupied(*iter)) {
        continue;
      }
      if (occupied_index++ % stride != 0) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = iter.getX();
      point.y = iter.getY();
      point.z = iter.getZ();
      const auto is_ground =
        iter->getOccupancyType() == rtabmap::RtabmapColorOcTreeNode::kTypeGround;
      (is_ground ? ground.points : obstacles.points).push_back(point);
      ++displayed;
    }
    ground_array.markers.push_back(ground);
    obstacle_array.markers.push_back(obstacles);
    array.markers.push_back(std::move(ground));
    array.markers.push_back(std::move(obstacles));
    markers_pub_->publish(array);
    ground_markers_pub_->publish(ground_array);
    obstacle_markers_pub_->publish(obstacle_array);
    return displayed;
  }

  std::string map_data_topic_;
  std::string frame_id_;
  std::size_t max_marker_voxels_{};
  double pose_update_tolerance_m_{};
  double pose_update_tolerance_rad_{};
  std::int64_t generation_{};
  std::int64_t last_stamp_{};
  double test_clear_radius_{};
  bool use_checkpoints_{};
  Vec3 test_offset_;
  std::optional<Pose3> test_start_raw_;
  std::optional<Pose3> test_latest_raw_;
  std::int64_t test_latest_stamp_{};
  std::int64_t test_goal_stamp_{};
  std::int64_t test_anchor_sequence_{};
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr test_goal_sub_;
  std::int64_t test_start_stamp_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr test_odom_sub_;
  std::map<int, LocalGridObservation> observations_;
  std::map<int, rtabmap::Transform> optimized_poses_;
  RtabmapOctomapAssembler assembler_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr map_data_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ground_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr test_marker_pub_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::RtabmapOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
