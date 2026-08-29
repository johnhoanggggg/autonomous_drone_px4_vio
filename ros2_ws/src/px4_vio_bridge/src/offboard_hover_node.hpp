#pragma once

/// C++ counterpart of offboard_hover.py: the generic offboard sequencer,
/// safety watchdogs, keyboard/Foxglove kill+land controls and the PX4 command
/// plumbing that the global-planner adapter builds on.

#include <array>
#include <chrono>
#include <optional>
#include <string>

#include <termios.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "px4_vio_bridge/path_geometry.hpp"

namespace px4_vio_bridge
{

/// Publisher QoS for /fmu/in/* and the PX4 best-effort volatile streams.
rclcpp::QoS px4_pub_qos();
/// Subscription QoS for /fmu/out/*: this uXRCE agent publishes transient-local.
rclcpp::QoS px4_sub_qos();

class OffboardHover : public rclcpp::Node
{
public:
  explicit OffboardHover(const std::string & node_name = "offboard_hover");
  ~OffboardHover() override;

  void on_shutdown();
  void restore_terminal();

protected:
  // --- state machine ------------------------------------------------------
  enum class State
  {
    WaitPos, Stream, Engage, ClimbHold, Route, Land, Abort, Kill, Done
  };
  static const char * state_name(State state);
  void set_state(State state);

  /// Run the flight-specific portion of the sequence. Returns true when the
  /// current state was handled here.
  virtual bool handle_flight_state();
  virtual void publish_setpoint(double z_up, std::optional<double> yaw = std::nullopt);
  virtual void arm();
  /// (x, y) NED that the horizontal-error watchdog measures against.
  virtual Point2 hold_point() const;
  /// Distance from hold_point that trips the watchdog, in meters.
  virtual double horizontal_error_limit() const;
  virtual void on_local_position(const px4_msgs::msg::VehicleLocalPosition & msg);

  void tick();
  double monotonic_time() const;
  bool stale(std::optional<double> stamp, double timeout) const;

  // --- PX4 command plumbing ----------------------------------------------
  std::uint64_t now_us() const;
  void publish_offboard_mode();
  void send_command(std::uint16_t command, double p1 = 0.0, double p2 = 0.0);
  void request_offboard();
  void force_disarm();
  void request_land();
  void trigger_landing(const std::string & reason);
  void trigger_kill(const std::string & reason = "KILL KEY PRESSED");

  bool is_armed() const;
  bool is_offboard() const;
  bool pos_valid() const;
  bool check_flight_position();

  /// (setpoint, feedforward) pairs; feedforward is NaN when disabled.
  std::pair<double, double> ramp_z(double target_up);
  std::pair<double, double> ramp_yaw(double target);

  std::optional<std::string> tracking_loss_reason();
  std::optional<std::string> vio_fault_reason();

  // --- keyboard / teleop --------------------------------------------------
  void setup_keyboard_controls();
  void poll_keyboard_controls();
  static std::optional<std::string> decode_foxglove_teleop(
    const geometry_msgs::msg::Twist & msg);

  // --- parameters ---------------------------------------------------------
  double hover_height_{};
  double hold_time_{};
  double commanded_yaw_rate_{};
  bool yaw_feedforward_{};
  double commanded_climb_rate_{};
  double climb_leash_{};
  double climb_release_{};
  bool climb_feedforward_{};
  double rate_hz_{};
  double stream_time_{};
  double engage_timeout_{};
  double climb_timeout_{};
  double reach_tol_{};
  double max_flight_time_{};
  bool auto_arm_{};
  bool keyboard_kill_{};
  bool keyboard_land_{};
  bool foxglove_teleop_{};
  std::string foxglove_teleop_topic_;
  bool tracking_loss_land_{};
  double vio_pose_timeout_{};
  double vio_feature_timeout_{};
  double vio_odometry_timeout_{};
  int min_vio_features_{};
  double vio_feature_loss_time_{};
  double max_vio_yaw_error_{};
  double vio_yaw_error_time_{};
  double max_yaw_rate_{};
  double yaw_rate_loss_time_{};
  double max_horizontal_error_{};
  double horizontal_error_time_{};
  double tracking_arm_grace_{};
  double vio_reset_persist_{};
  double dt_{};

  // --- state --------------------------------------------------------------
  std::optional<px4_msgs::msg::VehicleLocalPosition> pos_;
  std::optional<px4_msgs::msg::VehicleControlMode> vcm_;
  std::optional<double> x0_, y0_, yaw0_;
  std::optional<double> yaw_cmd_;
  std::optional<double> z_cmd_;

  State state_{State::WaitPos};
  double t_{0.0};
  double armed_t_{0.0};
  double hold_t_{0.0};
  bool reached_{false};
  double last_cmd_t_{0.0};

  std::optional<double> last_vio_pose_time_;
  std::optional<double> last_vio_feature_time_;
  std::optional<double> last_vio_odometry_time_;
  std::optional<int> vio_feature_count_;
  std::optional<double> low_features_since_;
  std::optional<double> vio_at_origin_since_;
  std::optional<double> vio_yaw_error_;
  std::optional<double> vio_yaw_error_since_;
  std::optional<double> measured_yaw_rate_;
  std::optional<double> excessive_yaw_rate_since_;
  std::optional<double> horizontal_error_;
  std::optional<double> horizontal_error_since_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  void on_control_mode(const px4_msgs::msg::VehicleControlMode & msg);
  void on_vio_pose(const geometry_msgs::msg::PoseStamped & msg);
  void on_vio_feature_count(const std_msgs::msg::Int32 & msg);
  void on_vio_odometry(const px4_msgs::msg::VehicleOdometry & msg);
  void on_sensor_combined(const px4_msgs::msg::SensorCombined & msg);
  void on_foxglove_teleop(const geometry_msgs::msg::Twist & msg);

  int stdin_fd_{-1};
  bool stdin_fd_owned_{false};
  std::optional<termios> stdin_termios_;
  std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vio_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vio_feature_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vio_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
};

}  // namespace px4_vio_bridge
