#include "offboard_hover_node.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

namespace px4_vio_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr float kNanF = std::numeric_limits<float>::quiet_NaN();

double degrees(double radians) {return radians * 180.0 / kPi;}
double radians(double degrees_value) {return degrees_value * kPi / 180.0;}

std::string format(const char * fmt, ...) __attribute__((format(printf, 1, 2)));
std::string format(const char * fmt, ...)
{
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return std::string(buffer);
}
}  // namespace

rclcpp::QoS px4_pub_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS px4_sub_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
}

const char * OffboardHover::state_name(State state)
{
  switch (state) {
    case State::WaitPos: return "WAIT_POS";
    case State::Stream: return "STREAM";
    case State::Engage: return "ENGAGE";
    case State::ClimbHold: return "CLIMB_HOLD";
    case State::Route: return "ROUTE";
    case State::Land: return "LAND";
    case State::Abort: return "ABORT";
    case State::Kill: return "KILL";
    case State::Done: return "DONE";
  }
  return "?";
}

OffboardHover::OffboardHover(const std::string & node_name)
: rclcpp::Node(node_name)
{
  hover_height_ = declare_parameter<double>("hover_height", 0.30);
  hold_time_ = declare_parameter<double>("hold_time", 10.0);
  // Configured yaw-setpoint slew rate. Immutable after init: the measured gyro
  // rate lives in measured_yaw_rate_ and must NEVER be written here (a past
  // collision fed the gyro back into ramp_yaw -> runaway yaw).
  commanded_yaw_rate_ = radians(declare_parameter<double>("yaw_rate_deg", 5.0));
  yaw_feedforward_ = declare_parameter<bool>("yaw_feedforward", false);
  commanded_climb_rate_ = declare_parameter<double>("climb_rate", 0.25);
  climb_leash_ = declare_parameter<double>("climb_leash", 0.12);
  climb_release_ = declare_parameter<double>("climb_release", 0.05);
  climb_feedforward_ = declare_parameter<bool>("climb_feedforward", true);
  rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
  stream_time_ = declare_parameter<double>("stream_time", 1.0);
  engage_timeout_ = declare_parameter<double>("engage_timeout", 5.0);
  climb_timeout_ = declare_parameter<double>("climb_timeout", 15.0);
  reach_tol_ = declare_parameter<double>("reach_tol", 0.07);
  max_flight_time_ = declare_parameter<double>("max_flight_time", 40.0);
  auto_arm_ = declare_parameter<bool>("auto_arm", false);
  keyboard_kill_ = declare_parameter<bool>("keyboard_kill", true);
  keyboard_land_ = declare_parameter<bool>("keyboard_land", true);
  foxglove_teleop_ = declare_parameter<bool>("foxglove_teleop", false);
  foxglove_teleop_topic_ =
    declare_parameter<std::string>("foxglove_teleop_topic", "/planner/flight/teleop");
  tracking_loss_land_ = declare_parameter<bool>("tracking_loss_land", true);
  const auto vio_pose_topic =
    declare_parameter<std::string>("vio_pose_topic", "/rtabmap/vio_pose");
  const auto vio_feature_topic =
    declare_parameter<std::string>("vio_feature_topic", "/rtabmap/vio_feature_count");
  const auto vio_odometry_topic =
    declare_parameter<std::string>("vio_odometry_topic", "/fmu/in/vehicle_visual_odometry");
  vio_pose_timeout_ = declare_parameter<double>("vio_pose_timeout", 0.75);
  vio_feature_timeout_ = declare_parameter<double>("vio_feature_timeout", 0.5);
  vio_odometry_timeout_ = declare_parameter<double>("vio_odometry_timeout", 0.5);
  min_vio_features_ = declare_parameter<int>("min_vio_features", 160);
  vio_feature_loss_time_ = declare_parameter<double>("vio_feature_loss_time", 0.25);
  max_vio_yaw_error_ = radians(declare_parameter<double>("max_vio_yaw_error_deg", 20.0));
  vio_yaw_error_time_ = declare_parameter<double>("vio_yaw_error_time", 0.20);
  max_yaw_rate_ = radians(declare_parameter<double>("max_yaw_rate_deg", 60.0));
  yaw_rate_loss_time_ = declare_parameter<double>("yaw_rate_loss_time", 0.10);
  max_horizontal_error_ = declare_parameter<double>("max_horizontal_error", 0.35);
  horizontal_error_time_ = declare_parameter<double>("horizontal_error_time", 0.25);
  tracking_arm_grace_ = declare_parameter<double>("tracking_arm_grace", 1.0);
  vio_reset_persist_ = declare_parameter<double>("vio_reset_persist", 0.2);

  if (!std::isfinite(rate_hz_) || rate_hz_ <= 0.0) {
    throw std::invalid_argument("rate_hz must be finite and positive");
  }
  dt_ = 1.0 / rate_hz_;

  ocm_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", px4_pub_qos());
  sp_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", px4_pub_qos());
  cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", px4_pub_qos());

  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position_v1", px4_sub_qos(),
    [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
      on_local_position(*msg);
    });
  // This PX4 build's dds_topics.yaml publishes vehicle_control_mode (not
  // vehicle_status), so arm/offboard state is confirmed from its flags.
  control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", px4_sub_qos(),
    [this](px4_msgs::msg::VehicleControlMode::ConstSharedPtr msg) {on_control_mode(*msg);});
  vio_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    vio_pose_topic, 10,
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_vio_pose(*msg);});
  vio_feature_sub_ = create_subscription<std_msgs::msg::Int32>(
    vio_feature_topic, 10,
    [this](std_msgs::msg::Int32::ConstSharedPtr msg) {on_vio_feature_count(*msg);});
  vio_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    vio_odometry_topic, px4_pub_qos(),
    [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {on_vio_odometry(*msg);});
  sensor_combined_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
    "/fmu/out/sensor_combined", px4_pub_qos(),
    [this](px4_msgs::msg::SensorCombined::ConstSharedPtr msg) {on_sensor_combined(*msg);});

  if (foxglove_teleop_) {
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      foxglove_teleop_topic_, 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_foxglove_teleop(*msg);});
    RCLCPP_WARN(
      get_logger(), "FOXGLOVE FLIGHT CONTROLS: %s linear.z=-1 LAND, linear.z=-2 "
      "EMERGENCY KILL", foxglove_teleop_topic_.c_str());
  }
}

OffboardHover::~OffboardHover()
{
  restore_terminal();
}

double OffboardHover::monotonic_time() const
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_).count();
}

bool OffboardHover::stale(std::optional<double> stamp, double timeout) const
{
  return !stamp || monotonic_time() - *stamp > timeout;
}

// --- keyboard flight controls ---------------------------------------------

void OffboardHover::setup_keyboard_controls()
{
  if (!keyboard_kill_ && !keyboard_land_) {
    RCLCPP_WARN(get_logger(), "keyboard kill and land controls disabled by parameters");
    return;
  }
  if (::isatty(STDIN_FILENO)) {
    stdin_fd_ = STDIN_FILENO;
  } else {
    // ros2 launch does not forward its stdin to child nodes. Opening the
    // controlling terminal keeps K/L available from that shell.
    stdin_fd_ = ::open("/dev/tty", O_RDONLY | O_NONBLOCK);
    stdin_fd_owned_ = true;
  }
  termios attributes{};
  if (stdin_fd_ < 0 || ::tcgetattr(stdin_fd_, &attributes) != 0) {
    if (stdin_fd_owned_ && stdin_fd_ >= 0) {
      ::close(stdin_fd_);
    }
    stdin_fd_ = -1;
    stdin_fd_owned_ = false;
    RCLCPP_ERROR(
      get_logger(), "could not enable K/L keyboard controls: %s; keep the RC kill ready",
      std::strerror(errno));
    return;
  }
  stdin_termios_ = attributes;
  // cbreak: character-at-a-time, echo off, signals still delivered.
  termios raw = attributes;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (::tcsetattr(stdin_fd_, TCSANOW, &raw) != 0) {
    restore_terminal();
    RCLCPP_ERROR(
      get_logger(), "could not enable K/L keyboard controls: %s; keep the RC kill ready",
      std::strerror(errno));
    return;
  }
  RCLCPP_WARN(
    get_logger(),
    "KEYBOARD CONTROLS: press L for AUTO.LAND; press K to FORCE-DISARM immediately");
}

void OffboardHover::restore_terminal()
{
  if (stdin_fd_ >= 0 && stdin_termios_) {
    ::tcsetattr(stdin_fd_, TCSADRAIN, &*stdin_termios_);
  }
  if (stdin_fd_owned_ && stdin_fd_ >= 0) {
    ::close(stdin_fd_);
  }
  stdin_fd_ = -1;
  stdin_fd_owned_ = false;
  stdin_termios_.reset();
}

void OffboardHover::poll_keyboard_controls()
{
  if (stdin_fd_ < 0) {
    return;
  }
  fd_set readable;
  FD_ZERO(&readable);
  FD_SET(stdin_fd_, &readable);
  timeval timeout{0, 0};
  const int ready = ::select(stdin_fd_ + 1, &readable, nullptr, nullptr, &timeout);
  if (ready < 0) {
    RCLCPP_ERROR(get_logger(), "keyboard control read failed: %s", std::strerror(errno));
    restore_terminal();
    return;
  }
  if (ready == 0) {
    return;
  }
  char keys[64];
  const auto count = ::read(stdin_fd_, keys, sizeof(keys));
  if (count < 0) {
    RCLCPP_ERROR(get_logger(), "keyboard control read failed: %s", std::strerror(errno));
    restore_terminal();
    return;
  }
  bool kill_pressed = false;
  bool land_pressed = false;
  for (ssize_t index = 0; index < count; ++index) {
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(keys[index])));
    kill_pressed = kill_pressed || key == 'k';
    land_pressed = land_pressed || key == 'l';
  }
  // K takes precedence if both keys are present in one read.
  if (keyboard_kill_ && kill_pressed) {
    trigger_kill();
  } else if (keyboard_land_ && land_pressed) {
    trigger_landing("LAND KEY PRESSED");
  }
}

std::optional<std::string> OffboardHover::decode_foxglove_teleop(
  const geometry_msgs::msg::Twist & msg)
{
  const double value = msg.linear.z;
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  if (value <= -1.5) {
    return std::string("KILL");
  }
  if (value <= -0.5) {
    return std::string("LAND");
  }
  return std::nullopt;
}

void OffboardHover::on_foxglove_teleop(const geometry_msgs::msg::Twist & msg)
{
  const auto command = decode_foxglove_teleop(msg);
  if (!command) {
    return;
  }
  if (*command == "KILL") {
    // Do not let a retained/stale browser command terminate a newly started dry
    // run before it can reach the normal arming gates.
    if (!is_armed()) {
      RCLCPP_WARN(get_logger(), "FOXGLOVE EMERGENCY KILL ignored because vehicle is disarmed");
      return;
    }
    trigger_kill("FOXGLOVE EMERGENCY KILL PRESSED");
  } else {
    trigger_landing("FOXGLOVE LAND PRESSED");
  }
}

void OffboardHover::trigger_kill(const std::string & reason)
{
  if (state_ == State::Kill) {
    return;
  }
  RCLCPP_FATAL(get_logger(), "%s -> PX4 FORCED DISARM; MOTORS STOPPING", reason.c_str());
  set_state(State::Kill);
  // Send an immediate burst as well as repeating in tick(), since this is a
  // BEST_EFFORT link and a single command must not be relied upon.
  for (int index = 0; index < 5; ++index) {
    force_disarm();
  }
}

// --- subscriptions ---------------------------------------------------------

void OffboardHover::on_local_position(const px4_msgs::msg::VehicleLocalPosition & msg)
{
  pos_ = msg;
  if (!x0_ || !y0_) {
    return;
  }
  const double now = monotonic_time();
  const auto hold = hold_point();
  horizontal_error_ = std::hypot(msg.x - hold.first, msg.y - hold.second);
  if (*horizontal_error_ > horizontal_error_limit()) {
    if (!horizontal_error_since_) {
      horizontal_error_since_ = now;
    }
  } else {
    horizontal_error_since_.reset();
  }
}

Point2 OffboardHover::hold_point() const
{
  return {x0_.value_or(0.0), y0_.value_or(0.0)};
}

double OffboardHover::horizontal_error_limit() const
{
  return max_horizontal_error_;
}

void OffboardHover::on_control_mode(const px4_msgs::msg::VehicleControlMode & msg)
{
  vcm_ = msg;
}

void OffboardHover::on_vio_pose(const geometry_msgs::msg::PoseStamped & msg)
{
  const double now = monotonic_time();
  last_vio_pose_time_ = now;
  // A relocalization reset writes exactly (0, 0, 0); real poses are always
  // noisy and never land on bit-exact zero, so this alone flags the reset.
  const auto & p = msg.pose.position;
  if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) {
    if (!vio_at_origin_since_) {
      vio_at_origin_since_ = now;
    }
  } else {
    vio_at_origin_since_.reset();
  }
}

void OffboardHover::on_vio_feature_count(const std_msgs::msg::Int32 & msg)
{
  const double now = monotonic_time();
  last_vio_feature_time_ = now;
  vio_feature_count_ = static_cast<int>(msg.data);
  if (*vio_feature_count_ < min_vio_features_) {
    if (!low_features_since_) {
      low_features_since_ = now;
    }
  } else {
    low_features_since_.reset();
  }
}

void OffboardHover::on_vio_odometry(const px4_msgs::msg::VehicleOdometry & msg)
{
  last_vio_odometry_time_ = monotonic_time();
  if (!pos_ || !std::isfinite(pos_->heading)) {
    vio_yaw_error_.reset();
    vio_yaw_error_since_.reset();
    return;
  }
  const std::array<double, 4> q{msg.q[0], msg.q[1], msg.q[2], msg.q[3]};
  for (const double value : q) {
    if (!std::isfinite(value)) {
      vio_yaw_error_.reset();
      vio_yaw_error_since_.reset();
      return;
    }
  }
  vio_yaw_error_ = std::abs(
    wrap_pi(pos_->heading - px4_yaw_from_quaternion(q[0], q[1], q[2], q[3])));
  if (*vio_yaw_error_ > max_vio_yaw_error_) {
    if (!vio_yaw_error_since_) {
      vio_yaw_error_since_ = monotonic_time();
    }
  } else {
    vio_yaw_error_since_.reset();
  }
}

void OffboardHover::on_sensor_combined(const px4_msgs::msg::SensorCombined & msg)
{
  measured_yaw_rate_ = std::abs(static_cast<double>(msg.gyro_rad[2]));
  if (*measured_yaw_rate_ > max_yaw_rate_) {
    if (!excessive_yaw_rate_since_) {
      excessive_yaw_rate_since_ = monotonic_time();
    }
  } else {
    excessive_yaw_rate_since_.reset();
  }
}

bool OffboardHover::is_armed() const {return vcm_ && vcm_->flag_armed;}
bool OffboardHover::is_offboard() const
{
  return vcm_ && vcm_->flag_control_offboard_enabled;
}
bool OffboardHover::pos_valid() const {return pos_ && pos_->xy_valid && pos_->z_valid;}

// --- publishers ------------------------------------------------------------

std::uint64_t OffboardHover::now_us() const
{
  return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

void OffboardHover::publish_offboard_mode()
{
  px4_msgs::msg::OffboardControlMode m;
  m.timestamp = now_us();
  m.position = true;
  m.velocity = false;
  m.acceleration = false;
  m.attitude = false;
  m.body_rate = false;
  ocm_pub_->publish(m);
}

void OffboardHover::publish_setpoint(double z_up, std::optional<double> yaw)
{
  const double target = yaw.value_or(yaw0_.value_or(0.0));
  const auto [yaw_sp, yawspeed] = ramp_yaw(target);
  const auto [z_sp, vz_sp] = ramp_z(z_up);
  px4_msgs::msg::TrajectorySetpoint m;
  m.timestamp = now_us();
  // NED, z down-negative-up.
  m.position = {static_cast<float>(*x0_), static_cast<float>(*y0_),
    static_cast<float>(-z_sp)};
  m.velocity = {kNanF, kNanF, static_cast<float>(vz_sp)};
  m.acceleration = {kNanF, kNanF, kNanF};
  m.yaw = static_cast<float>(yaw_sp);
  m.yawspeed = static_cast<float>(yawspeed);
  sp_pub_->publish(m);
}

std::pair<double, double> OffboardHover::ramp_z(double target_up)
{
  // See offboard_hover.ramp_z: PX4's takeoff ramp hands the position controller
  // a collective ~0.08 below hover, so a hard z step climbs on the integrator
  // alone (20.4 s for 0.30 m, ULog 209). The feedforward puts the full
  // commanded rate into the velocity loop; the ramp keeps the position setpoint
  // honest while that happens.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (commanded_climb_rate_ <= 0.0) {
    z_cmd_ = target_up;
    return {target_up, nan};
  }
  if (!z_cmd_) {
    z_cmd_ = target_up;
  }
  // Advance toward the target, clamped to the remaining distance so the ramp
  // lands exactly on it and never overshoots.
  const double step = commanded_climb_rate_ * dt_;
  *z_cmd_ += std::clamp(target_up - *z_cmd_, -step, step);
  // The leash is applied on every path, including the first call.
  if (climb_leash_ > 0.0 && pos_) {
    const double z_now = -static_cast<double>(pos_->z);
    z_cmd_ = std::clamp(*z_cmd_, z_now - climb_leash_, z_now + climb_leash_);
  }
  if (!climb_feedforward_) {
    return {*z_cmd_, nan};
  }
  // Release on the VEHICLE's remaining distance, not the ramp's: the ramp can
  // reach the target while the vehicle is still low but inside the leash.
  const double z_ref = pos_ ? -static_cast<double>(pos_->z) : *z_cmd_;
  const double remaining = target_up - z_ref;
  // Taper inside the release band rather than switching off, so there is no
  // discontinuity and noise near the target cannot chatter the feedforward.
  const double span = std::max(climb_release_, 1.0e-6);
  const double rate =
    commanded_climb_rate_ * std::min(1.0, std::abs(remaining) / span);
  if (rate < 1.0e-3) {
    return {*z_cmd_, 0.0};
  }
  // Deliberately the commanded rate, not d(z_cmd)/dt: while the leash holds the
  // setpoint back the vehicle is failing to keep up, which is exactly when the
  // velocity loop needs a real error to wind against. NED, so climbing is -vz.
  return {*z_cmd_, -std::copysign(rate, remaining)};
}

std::pair<double, double> OffboardHover::ramp_yaw(double target)
{
  // Sending PX4 a bounded yaw setpoint avoids the step-yaw torque spike that
  // saturates the mixer when a new heading is commanded. Uses ONLY
  // commanded_yaw_rate_ -- never the measured gyro rate.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (!yaw_cmd_) {
    yaw_cmd_ = target;
  }
  if (commanded_yaw_rate_ <= 0.0) {
    yaw_cmd_ = target;
    return {target, nan};
  }
  const double step = commanded_yaw_rate_ * dt_;
  const double delta = wrap_pi(target - *yaw_cmd_);
  if (std::abs(delta) <= step) {
    yaw_cmd_ = target;
    return {target, yaw_feedforward_ ? 0.0 : nan};
  }
  yaw_cmd_ = wrap_pi(*yaw_cmd_ + std::copysign(step, delta));
  if (!yaw_feedforward_) {
    return {*yaw_cmd_, nan};
  }
  return {*yaw_cmd_, std::copysign(commanded_yaw_rate_, delta)};
}

void OffboardHover::send_command(std::uint16_t command, double p1, double p2)
{
  px4_msgs::msg::VehicleCommand m;
  m.timestamp = now_us();
  m.command = command;
  m.param1 = static_cast<float>(p1);
  m.param2 = static_cast<float>(p2);
  m.target_system = 1;
  m.target_component = 1;
  m.source_system = 1;
  m.source_component = 1;
  m.from_external = true;
  cmd_pub_->publish(m);
}

void OffboardHover::request_offboard()
{
  // DO_SET_MODE: base_mode custom(1), custom_main_mode OFFBOARD(6)
  send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
}

void OffboardHover::arm()
{
  send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void OffboardHover::force_disarm()
{
  // PX4/MAVLink magic value 21196 bypasses the normal in-air disarm denial.
  send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0);
}

void OffboardHover::request_land()
{
  send_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

void OffboardHover::trigger_landing(const std::string & reason)
{
  if (state_ == State::Land || state_ == State::Kill || state_ == State::Done) {
    return;
  }
  if (!is_armed()) {
    RCLCPP_WARN(get_logger(), "%s, but vehicle is not armed; LAND ignored", reason.c_str());
    return;
  }
  RCLCPP_ERROR(get_logger(), "%s -> AUTO.LAND", reason.c_str());
  request_land();
  set_state(State::Land);
}

bool OffboardHover::check_flight_position()
{
  if (auto_arm_ && !pos_valid()) {
    trigger_landing("lost local position in flight");
    return false;
  }
  return true;
}

std::optional<std::string> OffboardHover::tracking_loss_reason()
{
  if (!tracking_loss_land_ || !is_armed()) {
    return std::nullopt;
  }
  if (armed_t_ < tracking_arm_grace_) {
    return std::nullopt;
  }
  if (const auto reason = vio_fault_reason()) {
    return reason;
  }
  const double now = monotonic_time();
  if (excessive_yaw_rate_since_ &&
    now - *excessive_yaw_rate_since_ >= yaw_rate_loss_time_)
  {
    return format(
      "yaw rate exceeded %.0f deg/s for %.2fs (latest=%.0f deg/s)",
      degrees(max_yaw_rate_), yaw_rate_loss_time_, degrees(*measured_yaw_rate_));
  }
  if (horizontal_error_since_ &&
    now - *horizontal_error_since_ >= horizontal_error_time_)
  {
    return format(
      "horizontal hold error exceeded %.2fm for %.2fs (latest=%.2fm)",
      horizontal_error_limit(), horizontal_error_time_, *horizontal_error_);
  }
  return std::nullopt;
}

std::optional<std::string> OffboardHover::vio_fault_reason()
{
  // Kept separate from tracking_loss_reason so a dry run can observe that the
  // checks fire without arming, even though LAND only happens when armed.
  const double now = monotonic_time();
  if (!last_vio_pose_time_ || now - *last_vio_pose_time_ > vio_pose_timeout_) {
    return format("RTAB-Map VIO pose stale for >%.2fs", vio_pose_timeout_);
  }
  if (!last_vio_feature_time_ || now - *last_vio_feature_time_ > vio_feature_timeout_) {
    return format("RTAB-Map feature data stale for >%.2fs", vio_feature_timeout_);
  }
  if (!last_vio_odometry_time_ || now - *last_vio_odometry_time_ > vio_odometry_timeout_) {
    return format("PX4 visual odometry input stale for >%.2fs", vio_odometry_timeout_);
  }
  if (low_features_since_ && now - *low_features_since_ >= vio_feature_loss_time_) {
    return format(
      "RTAB-Map tracking features stayed below %d for %.2fs (latest=%d)",
      min_vio_features_, vio_feature_loss_time_, vio_feature_count_.value_or(-1));
  }
  if (vio_at_origin_since_ && now - *vio_at_origin_since_ >= vio_reset_persist_) {
    return format(
      "RTAB-Map VIO pose reset to exact origin for %.2fs", vio_reset_persist_);
  }
  if (vio_yaw_error_since_ && now - *vio_yaw_error_since_ >= vio_yaw_error_time_) {
    return format(
      "VIO yaw disagreed with PX4 by more than %.0fdeg for %.2fs (latest=%.1fdeg)",
      degrees(max_vio_yaw_error_), vio_yaw_error_time_, degrees(*vio_yaw_error_));
  }
  return std::nullopt;
}

// --- state machine ---------------------------------------------------------

void OffboardHover::set_state(State state)
{
  RCLCPP_WARN(get_logger(), "-> %s", state_name(state));
  state_ = state;
  t_ = 0.0;
}

bool OffboardHover::handle_flight_state()
{
  if (state_ != State::ClimbHold) {
    return false;
  }
  publish_setpoint(hover_height_);
  if (auto_arm_ && !pos_valid()) {
    RCLCPP_ERROR(get_logger(), "lost local position in flight -> LAND");
    set_state(State::Land);
    return true;
  }
  if (!reached_ && pos_) {
    if (std::abs(-static_cast<double>(pos_->z) - hover_height_) <= reach_tol_) {
      reached_ = true;
      RCLCPP_WARN(get_logger(), "reached hover altitude, starting hold clock");
    }
  }
  if (reached_ || t_ > climb_timeout_) {
    hold_t_ += dt_;
    if (hold_t_ >= hold_time_) {
      set_state(State::Land);
    }
  }
  return true;
}

void OffboardHover::tick()
{
  t_ += dt_;
  poll_keyboard_controls();

  if (state_ == State::Kill) {
    // Stop offboard streaming and repeat for one second for transport
    // robustness. Forced disarm is intentionally effective in the air.
    force_disarm();
    if (t_ >= 1.0) {
      set_state(State::Done);
    }
    return;
  }

  if (is_armed()) {
    armed_t_ += dt_;
  }

  const auto tracking_loss = tracking_loss_reason();
  const bool terminal =
    state_ == State::Land || state_ == State::Kill || state_ == State::Done;
  if (tracking_loss && !terminal) {
    trigger_landing(*tracking_loss);
  } else if (tracking_loss_land_ && !is_armed() &&
    state_ != State::WaitPos && state_ != State::Stream && state_ != State::Engage &&
    !terminal && state_ != State::Abort)
  {
    // Dry run: surface that the detector fires, without arming or landing.
    if (const auto dry_fault = vio_fault_reason()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[dry run] tracking loss detected (disarmed, no LAND): %s", dry_fault->c_str());
    }
  }

  // global armed watchdog
  if (is_armed() && armed_t_ > max_flight_time_ &&
    state_ != State::Land && state_ != State::Done)
  {
    RCLCPP_ERROR(get_logger(), "max_flight_time exceeded -> LAND");
    set_state(State::Land);
  }

  if (state_ == State::WaitPos) {
    if (pos_valid() && vcm_) {
      x0_ = pos_->x;
      y0_ = pos_->y;
      yaw0_ = pos_->heading;
      yaw_cmd_ = pos_->heading;
      RCLCPP_WARN(
        get_logger(), "latched hold x=%.2f y=%.2f yaw=%.0fdeg",
        *x0_, *y0_, degrees(*yaw0_));
      set_state(State::Stream);
    } else if (t_ > 10.0) {
      RCLCPP_ERROR(get_logger(), "no valid local position in 10s -> ABORT");
      set_state(State::Abort);
    }
    return;
  }

  // From STREAM onward, ALWAYS keep the offboard heartbeat + setpoint flowing.
  publish_offboard_mode();

  if (handle_flight_state()) {
    return;
  }

  if (state_ == State::Stream) {
    publish_setpoint(0.0);  // hold on ground
    if (!pos_valid()) {
      RCLCPP_ERROR(get_logger(), "lost local position validity -> ABORT");
      set_state(State::Abort);
      return;
    }
    if (t_ >= stream_time_) {
      request_offboard();
      if (auto_arm_) {
        arm();
      }
      last_cmd_t_ = 0.0;
      set_state(State::Engage);
    }
    return;
  }

  if (state_ == State::Engage) {
    publish_setpoint(0.0);
    last_cmd_t_ += dt_;
    if (last_cmd_t_ >= 0.5) {  // resend requests periodically
      request_offboard();
      if (auto_arm_) {
        arm();
      }
      last_cmd_t_ = 0.0;
      RCLCPP_INFO(
        get_logger(), "ENGAGE t=%.1fs offboard=%d armed=%d",
        t_, static_cast<int>(is_offboard()), static_cast<int>(is_armed()));
    }
    if (is_offboard() && (is_armed() || !auto_arm_)) {
      set_state(State::ClimbHold);
    } else if (t_ > engage_timeout_) {
      RCLCPP_ERROR(
        get_logger(), "engage timeout (offboard=%d armed=%d) -> ABORT",
        static_cast<int>(is_offboard()), static_cast<int>(is_armed()));
      set_state(State::Abort);
    }
    return;
  }

  if (state_ == State::Land) {
    // keep streaming a couple cycles then hand to AUTO.LAND
    if (x0_) {
      publish_setpoint(hover_height_);
    }
    if (t_ < 0.2) {
      request_land();
    }
    if (!is_armed() && t_ > 1.0) {
      RCLCPP_WARN(get_logger(), "disarmed on ground -> DONE");
      set_state(State::Done);
    } else if (t_ > 20.0) {
      RCLCPP_WARN(get_logger(), "land phase timeout -> DONE");
      set_state(State::Done);
    }
    return;
  }

  if (state_ == State::Abort) {
    // never flew (or engage failed). If somehow armed, land; else just stop.
    if (is_armed()) {
      request_land();
    }
    if (t_ > 1.0) {
      set_state(State::Done);
    }
    return;
  }

  if (state_ == State::Done) {
    timer_->cancel();
    RCLCPP_WARN(get_logger(), "sequence complete");
    rclcpp::shutdown();
    return;
  }
}

void OffboardHover::on_shutdown()
{
  restore_terminal();
  if (is_armed()) {
    RCLCPP_ERROR(get_logger(), "shutdown while armed -> commanding AUTO.LAND");
    for (int index = 0; index < 5; ++index) {
      request_land();
    }
  }
}

}  // namespace px4_vio_bridge
