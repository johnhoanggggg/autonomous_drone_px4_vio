// Observation-only ROS-native odometry -> planner poses. The correction comes
// from the exact graph used to assemble the OctoMap, never an unrelated latest TF.
#include <memory>
#include <optional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "px4_vio_bridge/navigation_snapshot.hpp"

namespace px4_vio_bridge
{
class RosPlannerPose final : public rclcpp::Node
{
public:
  RosPlannerPose() : Node("ros_planner_pose")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "rtabmap_odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "oak");
    map_frame_ = declare_parameter<std::string>("map_frame", "world");
    pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
    map_timeout_ = declare_parameter<double>("map_timeout", 3.0);
    offset_ = {declare_parameter<double>("camera_position_frd_x", 0.100),
      declare_parameter<double>("camera_position_frd_y", -0.036),
      declare_parameter<double>("camera_position_frd_z", 0.056)};
    if (!std::isfinite(offset_.x) || !std::isfinite(offset_.y) || !std::isfinite(offset_.z) ||
      std::hypot(offset_.x, offset_.y, offset_.z) > 1.0 ||
      !std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0 ||
      !std::isfinite(map_timeout_) || map_timeout_ <= 0.0)
    {
      throw std::invalid_argument("invalid pose adapter limits/offset");
    }
    raw_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rtabmap/vio_pose", 10);
    corrected_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rtabmap/pose", 10);
    correction_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rtabmap/odom_correction", 10);
    snapshot_pub_ = create_publisher<std_msgs::msg::String>("/planner3d/pose_snapshot", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planner3d/pose_status", 10);
    metadata_sub_ = create_subscription<std_msgs::msg::String>(
      "/rtabmap/octomap_metadata", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        try {
          auto value = nlohmann::json::parse(msg->data);
          if (value.at("frame_id") != map_frame_ ||
            !value.at("ground_is_obstacle").get<bool>() || !value.at("ray_tracing").get<bool>() ||
            value.at("generation").get<std::int64_t>() <= generation_ ||
            !fresh_stamp(value.at("stamp_ns"), now().nanoseconds(), map_timeout_))
          {status("MAP_REJECTED"); return;}
          const auto correction = snapshot_pose(value.at("map_from_odom"));
          generation_ = value.at("generation");
          map_stamp_ = value.at("stamp_ns");
          correction_ = correction;
        } catch (const std::exception &) {correction_.reset(); status("MAP_REJECTED");}
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/rtabmap/vio_odometry", rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odometry(*msg);});
  }

private:
  void status(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = nlohmann::json{{"state", state}, {"observation_only", true},
      {"map_generation", generation_}}.dump();
    status_pub_->publish(msg);
  }

  void publish_pose(const Pose3 & pose, const builtin_interfaces::msg::Time & stamp,
    const std::string & frame, const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp; msg.header.frame_id = frame;
    msg.pose.position.x = pose.position.x; msg.pose.position.y = pose.position.y;
    msg.pose.position.z = pose.position.z;
    msg.pose.orientation.x = pose.orientation.x; msg.pose.orientation.y = pose.orientation.y;
    msg.pose.orientation.z = pose.orientation.z; msg.pose.orientation.w = pose.orientation.w;
    pub->publish(msg);
  }

  void on_odometry(const nav_msgs::msg::Odometry & msg)
  {
    const auto stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id != child_frame_ ||
      !fresh_stamp(stamp, now().nanoseconds(), pose_timeout_) || stamp <= last_stamp_)
    {status("ODOMETRY_REJECTED"); return;}
    Pose3 raw;
    try {
      const auto & p = msg.pose.pose.position; const auto & q = msg.pose.pose.orientation;
      raw = snapshot_pose({p.x, p.y, p.z, q.x, q.y, q.z, q.w});
      if (!std::isfinite(msg.pose.covariance[0]) || msg.pose.covariance[0] < 0.0 ||
        msg.pose.covariance[0] >= 9999.0) {throw std::invalid_argument("lost odometry");}
    } catch (const std::exception &) {status("ODOMETRY_REJECTED"); return;}
    last_stamp_ = stamp;
    publish_pose(raw, msg.header.stamp, odom_frame_, raw_pub_);
    if (!correction_ || stamp < map_stamp_ || !fresh_stamp(map_stamp_, now().nanoseconds(), map_timeout_)) {
      status("MAP_STALE"); return;
    }
    const auto corrected = apply_correction(*correction_, raw);
    publish_pose(corrected, msg.header.stamp, map_frame_, corrected_pub_);
    publish_pose(*correction_, msg.header.stamp, map_frame_, correction_pub_);
    const auto body = camera_to_body(raw, offset_);
    std_msgs::msg::String snapshot;
    snapshot.data = nlohmann::json{
      {"stamp_ns", stamp}, {"map_stamp_ns", map_stamp_}, {"map_generation", generation_},
      {"frame_id", map_frame_}, {"odom_frame_id", odom_frame_},
      {"pose", pose_json(apply_correction(*correction_, body))},
      {"raw_pose", pose_json(body)}, {"correction", pose_json(*correction_)},
      {"camera_position_frd", {offset_.x, offset_.y, offset_.z}}}.dump();
    snapshot_pub_->publish(snapshot);
    status("POSE_VALID");
  }

  std::string odom_frame_, child_frame_, map_frame_;
  double pose_timeout_, map_timeout_;
  Vec3 offset_;
  std::int64_t last_stamp_{}, generation_{}, map_stamp_{};
  std::optional<Pose3> correction_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pub_, corrected_pub_, correction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr snapshot_pub_, status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub_;
};
}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::RosPlannerPose>());
  rclcpp::shutdown();
}
