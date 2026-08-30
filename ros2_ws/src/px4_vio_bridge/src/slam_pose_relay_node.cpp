// Rebuild the loop-corrected pose the on-device RTABMapSLAM used to publish.
//
// rtabmap_slam exposes its correction as the map->odom TF and publishes
// localization_pose only on a localization event, which never fires in SLAM
// mode. This node composes that TF with the continuous VIO pose so
// /rtabmap/pose and /rtabmap/odom_correction keep the meaning and the rate the
// planner and follower expect.

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "px4_vio_bridge/slam_pose_relay.hpp"

namespace px4_vio_bridge
{
namespace
{

Pose3 from_msg(const geometry_msgs::msg::Pose & pose)
{
  Pose3 out;
  out.position = Vec3{pose.position.x, pose.position.y, pose.position.z};
  out.orientation = Quat{
    pose.orientation.x, pose.orientation.y, pose.orientation.z,
    pose.orientation.w};
  return out;
}

geometry_msgs::msg::Pose to_msg(const Pose3 & pose)
{
  geometry_msgs::msg::Pose out;
  out.position.x = pose.position.x;
  out.position.y = pose.position.y;
  out.position.z = pose.position.z;
  out.orientation.x = pose.orientation.x;
  out.orientation.y = pose.orientation.y;
  out.orientation.z = pose.orientation.z;
  out.orientation.w = pose.orientation.w;
  return out;
}

}  // namespace

class SlamPoseRelay : public rclcpp::Node
{
public:
  SlamPoseRelay()
  : rclcpp::Node("slam_pose_relay")
  {
    vio_pose_topic_ = declare_parameter("vio_pose_topic", "/rtabmap/vio_pose");
    pose_topic_ = declare_parameter("pose_topic", "/rtabmap/pose");
    correction_topic_ =
      declare_parameter("correction_topic", "/rtabmap/odom_correction");
    map_frame_ = declare_parameter("map_frame", "rtabmap3d_map");
    odom_frame_ = declare_parameter("odom_frame", "rtabmap3d_odom");
    // The on-device node published both topics in 'world'.
    output_frame_ = declare_parameter("output_frame", "world");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    correction_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(correction_topic_, 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      vio_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        onPose(*msg);
      });

    RCLCPP_INFO(
      get_logger(), "Relaying %s with %s<-%s into %s and %s",
      vio_pose_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str(),
      pose_topic_.c_str(), correction_topic_.c_str());
  }

private:
  void onPose(const geometry_msgs::msg::PoseStamped & msg)
  {
    // Identity until the mapper publishes a correction, which is exactly what
    // the on-device node emitted before its first loop closure.
    Pose3 correction;

    try {
      // The correction only changes on loop closure, so the latest available
      // transform is the right one; an exact-stamp lookup would mostly fail.
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_, odom_frame_, tf2::TimePointZero);
      const Quat rotation{
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w};
      if (is_unit_quaternion(rotation)) {
        correction.position = Vec3{
          tf.transform.translation.x, tf.transform.translation.y,
          tf.transform.translation.z};
        correction.orientation = rotation;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "map correction has a non-unit quaternion; rejected");
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no %s<-%s transform yet, relaying uncorrected pose (%s)",
        map_frame_.c_str(), odom_frame_.c_str(), ex.what());
    }

    geometry_msgs::msg::PoseStamped corrected;
    corrected.header.stamp = msg.header.stamp;
    corrected.header.frame_id = output_frame_;
    corrected.pose = to_msg(apply_correction(correction, from_msg(msg.pose)));
    pose_pub_->publish(corrected);

    geometry_msgs::msg::PoseStamped correction_msg;
    correction_msg.header.stamp = msg.header.stamp;
    correction_msg.header.frame_id = output_frame_;
    correction_msg.pose = to_msg(correction);
    correction_pub_->publish(correction_msg);
  }

  std::string vio_pose_topic_;
  std::string pose_topic_;
  std::string correction_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string output_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr correction_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::SlamPoseRelay>());
  rclcpp::shutdown();
  return 0;
}
