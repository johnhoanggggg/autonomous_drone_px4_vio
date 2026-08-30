#include "px4_vio_bridge/grid_clearance.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace px4_vio_bridge
{
namespace
{

double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool finite(const Point2 & point)
{
  return std::isfinite(point.first) && std::isfinite(point.second);
}

rclcpp::QoS px4_input_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS px4_output_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
}

}  // namespace

class OffboardGlobalPlannerCppShadow final : public rclcpp::Node
{
public:
  OffboardGlobalPlannerCppShadow()
  : Node("cpp_clearance_shadow")
  {
    const auto rate_hz = declare_parameter<double>("rate_hz", 20.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const auto map_pose_topic =
      declare_parameter<std::string>("map_pose_topic", "/rtabmap/body_pose");
    map_timeout_ = declare_parameter<double>("map_timeout", 3.0);
    pose_timeout_ = declare_parameter<double>("map_pose_timeout", 1.0);
    correction_timeout_ = declare_parameter<double>("correction_timeout", 1.0);
    local_position_timeout_ = declare_parameter<double>("local_position_timeout", 0.30);
    setpoint_timeout_ = declare_parameter<double>("setpoint_timeout", 0.30);
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      throw std::invalid_argument("rate_hz must be finite and positive");
    }
    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/rtabmap/grid", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {on_map(*msg);});
    map_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      map_pose_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_map_pose(*msg);});
    correction_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/rtabmap/odom_correction", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_correction(*msg);});
    follower_config_sub_ = create_subscription<std_msgs::msg::String>(
      "/planner/follower/config", map_qos,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {on_follower_config(*msg);});
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_output_qos(),
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
        on_local_position(*msg);
      });
    setpoint_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_input_qos(),
      [this](px4_msgs::msg::TrajectorySetpoint::ConstSharedPtr msg) {on_setpoint(*msg);});

    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/planner/flight/cpp_shadow/clearance_valid", 10);
    endpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner/flight/cpp_shadow/endpoint", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/planner/flight/cpp_shadow/status", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {tick();});
    RCLCPP_WARN(
      get_logger(),
      "C++ flight-adapter shadow only: %.1f Hz, publishes NO PX4 commands; "
      "clearance comes from /planner/follower/config",
      rate_hz);
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    if (msg.header.frame_id != frame_id_) {
      publish_status("WAITING_FOR_MAP frame mismatch");
      return;
    }
    const auto & q = msg.info.origin.orientation;
    if (std::abs(q.x) > 1.0e-6 || std::abs(q.y) > 1.0e-6 ||
      std::abs(q.z) > 1.0e-6 || std::abs(q.w - 1.0) > 1.0e-6)
    {
      publish_status("WAITING_FOR_MAP rotated origin unsupported");
      return;
    }
    GridMap next;
    next.width = msg.info.width;
    next.height = msg.info.height;
    next.resolution = msg.info.resolution;
    next.origin_x = msg.info.origin.position.x;
    next.origin_y = msg.info.origin.position.y;
    next.data.assign(msg.data.begin(), msg.data.end());
    if (!next.valid()) {
      publish_status("WAITING_FOR_MAP invalid geometry");
      return;
    }
    for (const auto value : next.data) {
      if (value < -1 || value > 100) {
        publish_status("WAITING_FOR_MAP invalid occupancy value");
        return;
      }
    }
    grid_ = std::move(next);
    map_received_ = now();
  }

  void on_map_pose(const geometry_msgs::msg::PoseStamped & msg)
  {
    const Point2 point{msg.pose.position.x, msg.pose.position.y};
    if (msg.header.frame_id == frame_id_ && finite(point)) {
      map_pose_ = point;
      map_pose_received_ = now();
    }
  }

  void on_correction(const geometry_msgs::msg::PoseStamped & msg)
  {
    const auto yaw = quaternion_yaw(msg.pose.orientation);
    if (std::isfinite(yaw)) {
      correction_yaw_ = yaw;
      correction_received_ = now();
    }
  }

  void on_follower_config(const std_msgs::msg::String & msg)
  {
    try {
      const auto config = nlohmann::json::parse(msg.data);
      if (config.at("frame_id").get<std::string>() != frame_id_) {
        throw std::invalid_argument("frame_id mismatch");
      }
      const auto radius = config.at("robot_radius").get<double>();
      const auto margin = config.at("safety_margin").get<double>();
      const auto threshold = config.at("occupied_threshold").get<int>();
      const auto clearance = radius + margin;
      if (!std::isfinite(radius) || !std::isfinite(margin) || radius < 0.0 ||
        margin < 0.0 || !std::isfinite(clearance) || clearance <= 0.0 ||
        threshold < 0 || threshold > 100)
      {
        throw std::invalid_argument("invalid clearance or occupied threshold");
      }
      required_clearance_ = clearance;
      occupied_threshold_ = threshold;
      config_valid_ = true;
    } catch (const std::exception & error) {
      config_valid_ = false;
      publish_status(std::string("WAITING_FOR_CONFIG ") + error.what());
    }
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    const Point2 point{msg.x, msg.y};
    if (msg.xy_valid && finite(point)) {
      local_position_ = point;
      local_position_received_ = now();
    }
  }

  void on_setpoint(const px4_msgs::msg::TrajectorySetpoint & msg)
  {
    const Point2 point{msg.position[0], msg.position[1]};
    if (finite(point)) {
      setpoint_ = point;
      setpoint_received_ = now();
    }
  }

  bool stale(const rclcpp::Time & stamp, double timeout) const
  {
    return stamp.nanoseconds() == 0 || (now() - stamp).seconds() > timeout;
  }

  void tick()
  {
    if (!config_valid_) {
      publish_status("WAITING_FOR_CONFIG");
      return;
    }
    if (!grid_) {
      publish_status("WAITING_FOR_MAP");
      return;
    }
    if (stale(map_received_, map_timeout_)) {
      publish_status("STALE_MAP");
      return;
    }
    if (!map_pose_ || stale(map_pose_received_, pose_timeout_)) {
      publish_status("STALE_MAP_POSE");
      return;
    }
    if (!correction_yaw_ || stale(correction_received_, correction_timeout_)) {
      publish_status("STALE_CORRECTION");
      return;
    }
    if (!local_position_ || stale(local_position_received_, local_position_timeout_)) {
      publish_status("STALE_LOCAL_POSITION");
      return;
    }
    if (!setpoint_ || stale(setpoint_received_, setpoint_timeout_)) {
      publish_status("STALE_TRAJECTORY_SETPOINT");
      return;
    }

    // Python publishes NED displacement = (vio_y, vio_x). Invert that swap,
    // then apply the map<-VIO correction rotation used by planner_flight.py.
    const Point2 ned_displacement{
      setpoint_->first - local_position_->first,
      setpoint_->second - local_position_->second};
    const Point2 vio_displacement{ned_displacement.second, ned_displacement.first};
    const auto cosine = std::cos(*correction_yaw_);
    const auto sine = std::sin(*correction_yaw_);
    const Point2 map_displacement{
      cosine * vio_displacement.first - sine * vio_displacement.second,
      sine * vio_displacement.first + cosine * vio_displacement.second};
    const Point2 endpoint{
      map_pose_->first + map_displacement.first,
      map_pose_->second + map_displacement.second};
    const auto safe = segment_has_clearance(
      *grid_, *map_pose_, endpoint, required_clearance_, occupied_threshold_);

    std_msgs::msg::Bool valid;
    valid.data = safe;
    valid_pub_->publish(valid);
    geometry_msgs::msg::PoseStamped endpoint_msg;
    endpoint_msg.header.stamp = now();
    endpoint_msg.header.frame_id = frame_id_;
    endpoint_msg.pose.position.x = endpoint.first;
    endpoint_msg.pose.position.y = endpoint.second;
    endpoint_msg.pose.position.z = 0.0;
    endpoint_msg.pose.orientation.w = 1.0;
    endpoint_pub_->publish(endpoint_msg);

    std::ostringstream status;
    status << (safe ? "SAFE" : "BLOCKED") << " clearance=" << std::fixed
           << std::setprecision(2) << required_clearance_ << "m segment="
           << std::hypot(
                endpoint.first - map_pose_->first,
                endpoint.second - map_pose_->second)
           << "m";
    publish_status(status.str());
  }

  void publish_status(const std::string & text)
  {
    if (text == last_status_) {
      return;
    }
    last_status_ = text;
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  std::string frame_id_;
  double required_clearance_{0.30};
  int occupied_threshold_{65};
  bool config_valid_{false};
  double map_timeout_{};
  double pose_timeout_{};
  double correction_timeout_{};
  double local_position_timeout_{};
  double setpoint_timeout_{};
  std::optional<GridMap> grid_;
  std::optional<Point2> map_pose_;
  std::optional<double> correction_yaw_;
  std::optional<Point2> local_position_;
  std::optional<Point2> setpoint_;
  rclcpp::Time map_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time map_pose_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time correction_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_position_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time setpoint_received_{0, 0, RCL_ROS_TIME};
  std::string last_status_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr map_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr correction_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follower_config_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr endpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_vio_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_vio_bridge::OffboardGlobalPlannerCppShadow>());
  rclcpp::shutdown();
  return 0;
}
