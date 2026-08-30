#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nlohmann/json.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "px4_vio_bridge/rtabmap_octomap.hpp"

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
    const auto max_marker_voxels = declare_parameter<int>("max_marker_voxels", 50000);
    if (max_marker_voxels < 0) {
      throw std::invalid_argument("max_marker_voxels cannot be negative");
    }
    max_marker_voxels_ = static_cast<std::size_t>(max_marker_voxels);
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    // ROS RTAB-Map's live MapData publisher is volatile, while the deterministic
    // fixture is transient-local. A volatile reliable subscription is compatible
    // with both offered durability policies.
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(octomap_topic, output_qos);
    metadata_pub_ = create_publisher<std_msgs::msg::String>(metadata_topic, output_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      markers_topic, output_qos);
    map_data_sub_ = create_subscription<rtabmap_msgs::msg::MapData>(
      map_data_topic_, input_qos,
      [this](rtabmap_msgs::msg::MapData::ConstSharedPtr message) {on_map_data(*message);});
    RCLCPP_INFO(
      get_logger(), "waiting for RTAB-Map keyframe grids on %s", map_data_topic_.c_str());
  }

private:
  void on_map_data(const rtabmap_msgs::msg::MapData & message)
  {
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    std::map<int, rtabmap::Signature> signatures;
    rtabmap::Transform map_to_odom;
    rtabmap_conversions::mapDataFromROS(message, poses, links, signatures, map_to_odom);
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
    if (!assembler_.rebuild(observations, optimized_poses_, &error)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "OctoMap generation rejected: %s", error.c_str());
      return;
    }
    octomap_msgs::msg::Octomap octomap_message;
    octomap_message.header = message.header;
    octomap_message.header.frame_id = frame_id_;
    if (!octomap_msgs::binaryMapToMsg(*assembler_.tree(), octomap_message)) {
      RCLCPP_ERROR(get_logger(), "failed to serialize RTAB-Map OctoMap");
      return;
    }
    ++generation_;
    const auto & built = assembler_.metadata();
    const auto displayed_marker_voxels = publish_markers(octomap_message.header);
    nlohmann::ordered_json metadata;
    metadata["generation"] = generation_;
    metadata["source_pose_generation"] = std::to_string(built.source_pose_generation);
    metadata["stamp_ns"] = rclcpp::Time(octomap_message.header.stamp).nanoseconds();
    metadata["frame_id"] = frame_id_;
    metadata["resolution"] = built.resolution;
    metadata["bounds"] = {
      {"min", {built.min_x, built.min_y, built.min_z}},
      {"max", {built.max_x, built.max_y, built.max_z}}};
    metadata["source_nodes"] = built.source_nodes;
    metadata["ground_cells"] = built.ground_cells;
    metadata["obstacle_cells"] = built.obstacle_cells;
    metadata["empty_cells"] = built.empty_cells;
    metadata["displayed_marker_voxels"] = displayed_marker_voxels;
    metadata["ground_is_obstacle"] = true;
    metadata["ray_tracing"] = true;
    std_msgs::msg::String metadata_message;
    metadata_message.data = metadata.dump();
    // The planner pairs by stamp and accepts either arrival order.
    metadata_pub_->publish(metadata_message);
    octomap_pub_->publish(octomap_message);
  }

  std::size_t publish_markers(const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    if (max_marker_voxels_ == 0 || assembler_.tree() == nullptr) {
      markers_pub_->publish(array);
      return 0;
    }

    std::size_t occupied_count = 0;
    for (auto iter = assembler_.tree()->begin_leafs();
      iter != assembler_.tree()->end_leafs(); ++iter)
    {
      occupied_count += assembler_.tree()->isNodeOccupied(*iter) ? 1U : 0U;
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
          assembler_.tree()->getResolution() * 0.98;
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
    for (auto iter = assembler_.tree()->begin_leafs();
      iter != assembler_.tree()->end_leafs(); ++iter)
    {
      if (!assembler_.tree()->isNodeOccupied(*iter)) {
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
    array.markers.push_back(std::move(ground));
    array.markers.push_back(std::move(obstacles));
    markers_pub_->publish(array);
    return displayed;
  }

  std::string map_data_topic_;
  std::string frame_id_;
  std::size_t max_marker_voxels_{};
  std::int64_t generation_{};
  std::map<int, LocalGridObservation> observations_;
  std::map<int, rtabmap::Transform> optimized_poses_;
  RtabmapOctomapAssembler assembler_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr map_data_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::RtabmapOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
