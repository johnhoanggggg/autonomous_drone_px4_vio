#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <std_msgs/msg/string.hpp>

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
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>("/rtabmap/octomap", qos);
    metadata_pub_ = create_publisher<std_msgs::msg::String>("/rtabmap/octomap_metadata", qos);
    map_data_sub_ = create_subscription<rtabmap_msgs::msg::MapData>(
      map_data_topic_, qos,
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
    std::vector<LocalGridObservation> observations;
    observations.reserve(signatures.size());
    for (const auto & item : signatures) {
      const auto & data = item.second.sensorData();
      cv::Mat ground;
      cv::Mat obstacles;
      cv::Mat empty;
      data.uncompressDataConst(nullptr, nullptr, nullptr, nullptr, &ground, &obstacles, &empty);
      if (ground.empty() && obstacles.empty() && empty.empty()) {
        continue;
      }
      observations.push_back({
        item.first, ground, obstacles, empty, data.gridCellSize(), data.gridViewPoint()});
    }
    std::string error;
    if (!assembler_.rebuild(observations, poses, &error)) {
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
    metadata["ground_is_obstacle"] = true;
    metadata["ray_tracing"] = true;
    std_msgs::msg::String metadata_message;
    metadata_message.data = metadata.dump();
    // The planner pairs by stamp and accepts either arrival order.
    metadata_pub_->publish(metadata_message);
    octomap_pub_->publish(octomap_message);
  }

  std::string map_data_topic_;
  std::string frame_id_;
  std::int64_t generation_{};
  RtabmapOctomapAssembler assembler_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr map_data_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::RtabmapOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
