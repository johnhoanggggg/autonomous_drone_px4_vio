// Deterministic occupied/free keyframe fixture for monitor and replay tests.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/map_data.hpp>

namespace px4_vio_bridge
{
namespace
{

cv::Mat to_mat(const std::vector<cv::Point3f> & points)
{
  // RTAB-Map local grids are one-row laser scans. A one-point Nx1 test can
  // hide this contract, but OctoMap's assembler sizes its buffers from cols.
  cv::Mat result(1, static_cast<int>(points.size()), CV_32FC3);
  for (std::size_t index = 0; index < points.size(); ++index) {
    result.at<cv::Point3f>(0, static_cast<int>(index)) = points[index];
  }
  return result;
}

}  // namespace

class Rtabmap3DFixtureNode final : public rclcpp::Node
{
public:
  Rtabmap3DFixtureNode()
  : Node("rtabmap_3d_fixture")
  {
    resolution_ = declare_parameter<double>("resolution", 0.10);
    loop_correction_after_ = declare_parameter<double>("loop_correction_after", 5.0);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = create_publisher<rtabmap_msgs::msg::MapData>("/rtabmap/mapData", qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rtabmap/pose", 10);
    vio_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rtabmap/vio_pose", 10);
    correction_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/rtabmap/odom_correction", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/waypoint/clicked", qos);
    build_observation();
    started_ = now();
    map_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_map();});
    pose_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {publish_pose();});
    publish_map();
  }

private:
  void build_observation()
  {
    std::vector<cv::Point3f> ground;
    std::vector<cv::Point3f> obstacles;
    std::vector<cv::Point3f> empty;
    const auto r = static_cast<float>(resolution_);
    for (int iz = 0; iz < 16; ++iz) {
      const auto z = 0.25F + static_cast<float>(iz) * r;
      for (int iy = 0; iy < 40; ++iy) {
        const auto y = -1.95F + static_cast<float>(iy) * r;
        for (int ix = 0; ix < 40; ++ix) {
          const auto x = -1.95F + static_cast<float>(ix) * r;
          if (iz == 0) {
            ground.emplace_back(x, y, z);
          } else if (
            std::abs(x - 0.45F) < r * 0.5F && std::abs(y) <= 0.3F && z <= 1.5F)
          {
            obstacles.emplace_back(x, y, z);
          } else {
            empty.emplace_back(x, y, z);
          }
        }
      }
    }
    rtabmap::SensorData data;
    data.setId(1);
    data.setStamp(1.0);
    data.setOccupancyGrid(
      to_mat(ground), to_mat(obstacles), to_mat(empty), r, cv::Point3f(-1.0F, 0.0F, 0.8F));
    signature_ = std::make_unique<rtabmap::Signature>(
      1, 0, 0, 1.0, "fixture", rtabmap::Transform::getIdentity(),
      rtabmap::Transform(), data);
  }

  bool corrected() const
  {
    return (now() - started_).seconds() >= loop_correction_after_;
  }

  void publish_map()
  {
    const auto y = corrected() ? 0.20F : 0.0F;
    const std::map<int, rtabmap::Transform> poses{
      {1, rtabmap::Transform(0.0F, y, 0.0F, 0.0F, 0.0F, 0.0F)}};
    const std::map<int, rtabmap::Signature> signatures{{1, *signature_}};
    rtabmap_msgs::msg::MapData message;
    rtabmap_conversions::mapDataToROS(
      poses, {}, signatures, rtabmap::Transform::getIdentity(), message);
    message.header.stamp = now();
    message.header.frame_id = "world";
    map_pub_->publish(message);
  }

  void publish_pose()
  {
    const auto is_corrected = corrected();
    const auto y = is_corrected ? 0.20 : 0.0;
    geometry_msgs::msg::PoseStamped vio;
    vio.header.stamp = now(); vio.header.frame_id = "world";
    vio.pose.position.x = -1.0; vio.pose.position.y = 0.0; vio.pose.position.z = 0.8;
    vio.pose.orientation.w = 1.0;
    vio_pub_->publish(vio);
    auto pose = vio;
    pose.pose.position.y = y;
    pose_pub_->publish(pose);
    geometry_msgs::msg::PoseStamped correction;
    correction.header = pose.header;
    correction.pose.position.y = y;
    correction.pose.orientation.w = 1.0;
    correction_pub_->publish(correction);
    // A clicked waypoint is an event, not a pose stream. Re-issue it only when
    // the fixture applies its synthetic loop correction so the planner gets one
    // deterministic replan per map generation instead of 20 replans/second.
    if (!goal_published_ || is_corrected != goal_corrected_) {
      geometry_msgs::msg::PointStamped goal;
      goal.header = pose.header;
      goal.point.x = 1.0; goal.point.y = y; goal.point.z = 0.8;
      goal_pub_->publish(goal);
      goal_published_ = true;
      goal_corrected_ = is_corrected;
    }
  }

  double resolution_{};
  double loop_correction_after_{};
  bool goal_published_{false};
  bool goal_corrected_{false};
  rclcpp::Time started_;
  std::unique_ptr<rtabmap::Signature> signature_;
  rclcpp::Publisher<rtabmap_msgs::msg::MapData>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vio_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr correction_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::Rtabmap3DFixtureNode>());
  rclcpp::shutdown();
  return 0;
}
