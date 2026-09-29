#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace px4_vio_bridge
{

// depthai-ros publishes oak_imu_frame with a hard-coded "RDF" rotation, but the
// OAK-D-Lite's BMI270 samples are that frame turned 180 deg about its x axis:
// at rest the accelerometer's reaction vector pointed DOWN in the camera frame
// (Madgwick+TF reported the camera 176 deg tilted). A 2026-09-28 tilt test
// against PX4's attitude fitted the true mounting from the accelerometer and,
// independently, the gyro; negating y and z matched both within 2.6 / 4.3 deg
// (the rest is the camera-to-FC mount), while the other flips were ~179 deg
// off. Re-express every vector in the published frame, keeping the header.
class OakImuFrameFix : public rclcpp::Node
{
public:
  explicit OakImuFrameFix(const rclcpp::NodeOptions & options)
  : Node("oak_imu_frame_fix", options)
  {
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_in", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::UniquePtr message) {
        message->linear_acceleration.y = -message->linear_acceleration.y;
        message->linear_acceleration.z = -message->linear_acceleration.z;
        message->angular_velocity.y = -message->angular_velocity.y;
        message->angular_velocity.z = -message->angular_velocity.z;
        // Raw OAK samples carry no orientation (identity, unknown covariance);
        // mark it unknown so no consumer mistakes it for an estimate.
        message->orientation_covariance[0] = -1.0;
        publisher_->publish(std::move(message));
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

}  // namespace px4_vio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(px4_vio_bridge::OakImuFrameFix)
