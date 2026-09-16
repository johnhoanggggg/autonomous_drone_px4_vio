// Explicit 3D flight authority. Reuses the existing PX4 sequencer and watchdogs.
// Unknown is traversable only with allow_unknown=true; occupied remains blocked.
#include <array>
#include <cstdio>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <std_msgs/msg/string.hpp>
#include "offboard_hover_node.hpp"
#include "px4_vio_bridge/flight_geometry_3d.hpp"

namespace px4_vio_bridge
{
class OffboardPlanner3D final : public OffboardHover
{
public:
  OffboardPlanner3D() : OffboardHover("cpp_flight_adapter_3d")
  {
    allow_unknown_=declare_parameter<bool>("allow_unknown",false);
    frame_=declare_parameter<std::string>("map_frame","world");
    odom_frame_=declare_parameter<std::string>("odom_frame","rtabmap_odom");
    path_topic_=declare_parameter<std::string>("path_snapshot_topic",
      allow_unknown_ ? "/planner3d/reference_snapshot" : "/planner3d/path_snapshot");
    radius_=declare_parameter<double>("required_clearance",.35);
    geofence_=declare_parameter<double>("geofence_radius",3.0);
    min_height_=declare_parameter<double>("min_height",.15);
    max_height_=declare_parameter<double>("max_height",1.25);
    map_timeout_=declare_parameter<double>("map_timeout",3.0);
    pose_timeout_=declare_parameter<double>("pose_timeout",.5);
    path_timeout_=declare_parameter<double>("path_timeout",2.0);
    px4_timeout_=declare_parameter<double>("px4_timeout",.5);
    fault_land_time_=declare_parameter<double>("planner_fault_land_time",3.0);
    goal_hold_time_=declare_parameter<double>("goal_hold_time",3.0);
    fresh_goal_=declare_parameter<bool>("require_goal_after_hover",true);
    yaw_track_=declare_parameter<bool>("yaw_follows_heading",true);
    yaw_align_=declare_parameter<double>("yaw_align_error_deg",40.0)*M_PI/180;
    yaw_resume_=declare_parameter<double>("yaw_resume_error_deg",15.0)*M_PI/180;
    yaw_offset_=declare_parameter<double>("vio_yaw_offset_deg",0.0)*M_PI/180;
    leash_xy_=declare_parameter<double>("command_leash_xy",.30);
    leash_z_=declare_parameter<double>("command_leash_z",.15);
    config_.max_horizontal_speed=declare_parameter<double>("command_speed",.10);
    config_.max_vertical_speed=declare_parameter<double>("command_vertical_speed",.05);
    config_.max_horizontal_acceleration=declare_parameter<double>("command_acceleration",.30);
    config_.max_vertical_acceleration=declare_parameter<double>("command_vertical_acceleration",.20);
    config_.lookahead=declare_parameter<double>("lookahead",.25);
    config_.max_cross_track=declare_parameter<double>("max_cross_track",.15);
    config_.max_vertical_track=declare_parameter<double>("max_vertical_track",.15);
    config_.stop_at_vertices=true;
    config_.arrival_tolerance=declare_parameter<double>("arrival_tolerance",.10);
    for (double v : {radius_,geofence_,max_height_,map_timeout_,pose_timeout_,path_timeout_,px4_timeout_,
      fault_land_time_,goal_hold_time_,leash_xy_,leash_z_,config_.max_horizontal_speed,
      config_.max_vertical_speed,config_.max_horizontal_acceleration,config_.max_vertical_acceleration,
      config_.lookahead,config_.max_cross_track,config_.max_vertical_track,config_.arrival_tolerance}) {
      if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("invalid 3D flight limits");}
    }
    if (!std::isfinite(min_height_) || min_height_<0 || min_height_>=hover_height_ ||
      hover_height_>max_height_ || geofence_<=radius_ || !std::isfinite(yaw_offset_) ||
      !std::isfinite(yaw_align_) || !std::isfinite(yaw_resume_) || yaw_resume_<0 || yaw_align_<yaw_resume_)
    {throw std::invalid_argument("invalid 3D flight bounds or yaw limits");}
    limiter_=std::make_unique<FlightPositionLimiter3D>(config_);
    const auto durable=rclcpp::QoS(1).reliable().transient_local();
    map_sub_=create_subscription<octomap_msgs::msg::Octomap>("/rtabmap/octomap",durable,
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr m){on_map(*m);});
    metadata_sub_=create_subscription<std_msgs::msg::String>("/rtabmap/octomap_metadata",durable,
      [this](std_msgs::msg::String::ConstSharedPtr m){on_metadata(*m);});
    pose_sub_=create_subscription<std_msgs::msg::String>("/planner3d/pose_snapshot",10,
      [this](std_msgs::msg::String::ConstSharedPtr m){on_pose(*m);});
    path_sub_=create_subscription<std_msgs::msg::String>(path_topic_,10,
      [this](std_msgs::msg::String::ConstSharedPtr m){on_path(*m);});
    info_sub_=create_subscription<rtabmap_msgs::msg::OdomInfo>("/rtabmap/vio_odom_info",rclcpp::SensorDataQoS(),
      [this](rtabmap_msgs::msg::OdomInfo::ConstSharedPtr m){
        const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
        if (!fresh_stamp(stamp,now().nanoseconds(),vio_feature_timeout_) || stamp<=info_stamp_) {return;}
        info_stamp_=stamp; info_lost_=m->lost;
        last_vio_feature_time_=monotonic_time(); vio_feature_count_=m->inliers;
        if (m->lost || m->inliers<min_vio_features_) {
          if (!low_features_since_) {low_features_since_=monotonic_time();}
        } else {low_features_since_.reset();}
      });
    mode_sub_=create_subscription<px4_msgs::msg::VehicleControlMode>("/fmu/out/vehicle_control_mode",px4_sub_qos(),
      [this](px4_msgs::msg::VehicleControlMode::ConstSharedPtr m){
        vcm_=*m; mode_received_=monotonic_time();});
    ev_sub_=create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",px4_pub_qos(),
      [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr m){
        if (ev_reset_ && *ev_reset_!=m->reset_counter && is_armed()) {fatal_="VIO_RESET";}
        ev_reset_=m->reset_counter;
        ev_valid_=m->pose_frame==px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED &&
          fresh_stamp(static_cast<std::int64_t>(m->timestamp_sample)*1000,now().nanoseconds(),vio_odometry_timeout_);
        for (auto v:m->position) {ev_valid_=ev_valid_ && std::isfinite(v);}
        const Quat q{m->q[1],m->q[2],m->q[3],m->q[0]};
        ev_valid_=ev_valid_ && is_unit_quaternion(q);
      });
    status_pub_=create_publisher<std_msgs::msg::String>("/planner3d/flight/status",10);
    config_pub_=create_publisher<std_msgs::msg::String>("/planner3d/flight/config",durable);
    std_msgs::msg::String cfg;
    cfg.data=nlohmann::json{{"allow_unknown",allow_unknown_},{"path_snapshot_topic",path_topic_},
      {"required_clearance",radius_},{"geofence_radius",geofence_},{"min_height",min_height_},
      {"max_height",max_height_},{"hover_height",hover_height_},{"auto_arm",auto_arm_},
      {"require_goal_after_hover",fresh_goal_},{"vio_yaw_offset_deg",yaw_offset_*180/M_PI},
      {"command_speed",config_.max_horizontal_speed},{"command_vertical_speed",config_.max_vertical_speed},
      {"command_acceleration",config_.max_horizontal_acceleration},
      {"command_vertical_acceleration",config_.max_vertical_acceleration},
      {"feature_health","rtabmap/vio_odom_info inliers and lost"},{"min_inliers",min_vio_features_},
      {"position_control",true},{"feedforward",false}}.dump(); config_pub_->publish(cfg);
    setup_keyboard_controls();
    timer_=create_wall_timer(std::chrono::duration<double>(dt_),[this](){tick();});
    RCLCPP_WARN(get_logger(),"3D FLIGHT: unknown=%s, occupied blocked, radius=%.2f m, auto_arm=%d",
      allow_unknown_?"allowed":"blocked",radius_,auto_arm_);
  }
protected:
  Point2 hold_point() const override
  {return command_ ? Point2{command_->x,command_->y} : OffboardHover::hold_point();}
  void on_local_position(const px4_msgs::msg::VehicleLocalPosition & m) override
  {
    if (!m.xy_valid || !m.z_valid || !std::isfinite(m.x) || !std::isfinite(m.y) ||
      !std::isfinite(m.z) || !std::isfinite(m.heading)) {local_valid_=false; return;}
    const std::array<unsigned,3> counters{m.xy_reset_counter,m.z_reset_counter,m.heading_reset_counter};
    if (resets_ && *resets_!=counters && is_armed()) {fatal_="PX4_ESTIMATOR_RESET";}
    if (m.timestamp<=local_stamp_) {return;}
    resets_=counters;local_stamp_=m.timestamp;local_valid_=true;local_received_=monotonic_time();
    OffboardHover::on_local_position(m);
    if (!ground_) {ground_=Point3{m.x,m.y,m.z}; limiter_->reset(*ground_);command_=*ground_;}
  }
  void arm() override
  {
    const auto reason=health(false);
    if (!reason.empty()) {publish_status("ARM_INHIBITED",reason); return;}
    const Point3 hover{ground_->x,ground_->y,ground_->z-hover_height_};
    const auto body=point3(pose_->position);
    const auto target=add_3d(body,ned_delta_to_map(subtract_3d(hover,local_point()),correction_->orientation,yaw_offset_));
    if (!chord(body,target)) {publish_status("ARM_INHIBITED","TAKEOFF_OBSTACLE");return;}
    OffboardHover::arm();
  }
  void publish_setpoint(double, std::optional<double> yaw=std::nullopt) override
  {
    if (!command_) {return;}
    if ((state_==State::Land || state_==State::Abort) && !terminal_hold_) {
      latch_hold();terminal_hold_=true;
    }
    const auto ramp=ramp_yaw(yaw.value_or(yaw_target_.value_or(yaw0_.value_or(0.0))));
    px4_msgs::msg::TrajectorySetpoint m;
    m.timestamp=now_us();m.position={static_cast<float>(command_->x),static_cast<float>(command_->y),static_cast<float>(command_->z)};
    const float nan=std::numeric_limits<float>::quiet_NaN();
    m.velocity={nan,nan,nan};m.acceleration={nan,nan,nan};m.jerk={nan,nan,nan};
    m.yaw=static_cast<float>(ramp.first);m.yawspeed=nan;
    last_setpoint_us_=m.timestamp;sp_pub_->publish(m);
  }
  bool handle_flight_state() override
  {
    clearance_checked_=false;
    if (state_==State::Land || state_==State::Abort || state_==State::Done) {return false;}
    const bool routing=state_==State::Route;
    const auto reason=health(routing);
    if (!reason.empty()) {fault(reason);return true;}
    if (state_==State::Stream || state_==State::Engage) {
      command_=*ground_;limiter_->reset(*command_);return false;
    }
    if (state_==State::ClimbHold) {
      Point3 target{ground_->x,ground_->y,ground_->z-hover_height_};
      if (auto error=move_to(target,true);!error.empty()) {fault(error);return true;}
      clear_fault();publish_setpoint(hover_height_);
      const double height=ground_->z-pos_->z;
      if (std::abs(height-hover_height_)<=reach_tol_ &&
        std::hypot(pos_->x-ground_->x,pos_->y-ground_->y)<=.15) {
        if (!hover_ready_) {
          hover_ready_=true;minimum_goal_=fresh_goal_?latest_goal_+1:0;
        }
        if (path_goal_>=minimum_goal_ && health(true).empty()) {
          set_state(State::Route);follower_.reset();publish_status("ROUTE_START","");
        } else {publish_status("WAITING_FOR_GOAL","click a waypoint at hover height");}
      } else if (t_>climb_timeout_) {trigger_landing("3D takeoff did not reach stable hover");}
      else {publish_status("CLIMB","");}
      return true;
    }
    if (!routing) {return false;}
    if (path_goal_<minimum_goal_) {fault("GOAL_PREDATES_HOVER");return true;}
    const auto body=point3(pose_->position);
    if (!follower_ || follower_path_stamp_!=path_stamp_) {
      if ((path_points_.size()!=active_points_.size() || !std::equal(path_points_.begin(),path_points_.end(),active_points_.begin(),
        [](const Point3&a,const Point3&b){return a.x==b.x && a.y==b.y && a.z==b.z;})) || follower_generation_!=generation_ || !follower_) {
        const auto up=rotate(correction_->orientation,{0,0,1});
        follower_=std::make_unique<RouteFollower3D>(limits_for_map_tilt(config_,up.z));
        if (!follower_->set_path(path_points_,body)) {fault("INVALID_ROUTE");return true;}
        for (std::size_t i=1;i<path_points_.size();++i) {
          if (!chord(path_points_[i-1],path_points_[i]) || !map_point_in_bounds(path_points_[i],false)) {
            fault("ROUTE_OBSTACLE_OR_GEOFENCE");return true;
          }
        }
        active_points_=path_points_;follower_generation_=generation_;
      }
      follower_path_stamp_=path_stamp_;
    }
    const auto preview=follower_->update(body,dt_,[this](const Point3&a,const Point3&b){return chord(a,b);});
    if (!preview.valid) {fault(preview.reason);return true;}
    if (yaw_track_) {
      const auto heading=map_delta_to_ned(subtract_3d(preview.lookahead,body),correction_->orientation,yaw_offset_);
      if (std::hypot(heading.x,heading.y)>.05) {yaw_target_=std::atan2(heading.y,heading.x);}
      const double error=std::abs(wrap_pi(yaw_target_.value_or(pos_->heading)-pos_->heading));
      if (error>yaw_align_) {yaw_holding_=true;}
      if (error<=yaw_resume_) {yaw_holding_=false;}
      if (yaw_holding_) {
        latch_hold();follower_.reset();fault_since_.reset();publish_setpoint(hover_height_);
        publish_status("YAW_ALIGN","");return true;
      }
    }
    const auto target=add_3d(local_point(),map_delta_to_ned(preview.displacement,correction_->orientation,yaw_offset_));
    if (auto error=move_to(target,false);!error.empty()) {fault(error);return true;}
    clear_fault();publish_setpoint(hover_height_);
    if (preview.reached) {
      publish_status(path_exact_?"GOAL_REACHED":(path_terminal_?"SAFE_APPROACH_REACHED":"EXPLORATION_ENDPOINT_REACHED"),"");
      if (!goal_since_) {goal_since_=monotonic_time();}
      if (path_terminal_ && is_armed() && monotonic_time()-*goal_since_>=goal_hold_time_) {
        trigger_landing(path_exact_?"3D goal reached":"3D safe approach reached");
      }
    } else {goal_since_.reset();publish_status("FOLLOWING","");}

    return true;
  }
private:
  using Json=nlohmann::json;
  static std::int64_t stamp_of(const builtin_interfaces::msg::Time & t)
  {return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;}
  void on_map(const octomap_msgs::msg::Octomap & m)
  {
    if (m.header.frame_id!=frame_ || !fresh_stamp(stamp_of(m.header.stamp),now().nanoseconds(),map_timeout_) ||
      stamp_of(m.header.stamp)<=map_stamp_) {return;}
    try {
      std::unique_ptr<octomap::AbstractOcTree> tree(m.binary?octomap_msgs::binaryMsgToMap(m):octomap_msgs::fullMsgToMap(m));
      if (!tree || tree->size()==0 || (!dynamic_cast<octomap::OcTree*>(tree.get()) &&
        !dynamic_cast<octomap::ColorOcTree*>(tree.get()))) {throw std::invalid_argument("invalid tree");}
      pending_tree_=std::move(tree);pending_stamp_=stamp_of(m.header.stamp);pair_map();
    } catch (...) {tree_.reset();pending_tree_.reset();}
  }
  void on_metadata(const std_msgs::msg::String & m)
  {
    try {
      auto j=Json::parse(m.data);
      if (j.at("frame_id")!=frame_ || j.at("generation").get<std::int64_t>()<=generation_) {return;}
      snapshot_pose(j.at("map_from_odom"));pending_metadata_=std::move(j);pair_map();
    } catch (...) {tree_.reset();pending_metadata_=Json();}
  }
  void pair_map()
  {
    if (!pending_tree_ || pending_metadata_.is_null() || pending_metadata_.at("stamp_ns")!=pending_stamp_) {return;}
    if (!fresh_stamp(pending_stamp_,now().nanoseconds(),map_timeout_) ||
      std::abs(pending_tree_->getResolution()-pending_metadata_.at("resolution").get<double>())>1e-9) {return;}
    tree_=std::move(pending_tree_);map_stamp_=pending_stamp_;generation_=pending_metadata_.at("generation");
    map_correction_=snapshot_pose(pending_metadata_.at("map_from_odom"));
    artificial_clear_=pending_metadata_.value("test_start_clear",Json::object()).value("enabled",false);
    map_received_=monotonic_time();pending_metadata_=Json();
  }
  void on_pose(const std_msgs::msg::String & m)
  {
    try {
      const auto j=Json::parse(m.data);const auto stamp=j.at("stamp_ns").get<std::int64_t>();
      if (stamp<=pose_stamp_) {return;}
      if (j.at("frame_id")!=frame_ || j.at("odom_frame_id")!=odom_frame_ ||
        !fresh_stamp(stamp,now().nanoseconds(),pose_timeout_)) {throw std::invalid_argument("invalid pose time/frame");}
      const auto raw=snapshot_pose(j.at("raw_pose")),corr=snapshot_pose(j.at("correction")),pose=snapshot_pose(j.at("pose"));
      if (!same_snapshot_transform(apply_correction(corr,raw),pose)) {throw std::invalid_argument("inconsistent body pose");}
      pose_=pose;correction_=corr;pose_stamp_=stamp;pose_generation_=j.at("map_generation");
      pose_map_stamp_=j.at("map_stamp_ns");pose_received_=monotonic_time();
    } catch (...) {pose_.reset();}
  }
  void on_path(const std_msgs::msg::String & m)
  {
    try {
      const auto j=Json::parse(m.data);const auto stamp=j.at("stamp_ns").get<std::int64_t>();
      if (stamp<=path_stamp_) {return;}
      if (j.at("frame_id")!=frame_ || !fresh_stamp(stamp,now().nanoseconds(),path_timeout_) ||
        j.at("lethal_radius").get<double>()+1e-9<radius_) {throw std::invalid_argument("invalid path contract");}
      const bool executable=j.value("executable",true);
      if (allow_unknown_) {
        if (executable || j.at("unknown_policy")!="penalized") {throw std::invalid_argument("expected explicit unknown proposal");}
      } else if (!executable) {throw std::invalid_argument("unknown proposal disabled");}
      const auto & info=allow_unknown_?j.at("verification"):j;
      const std::int64_t goal=j.value("goal_generation",std::int64_t(0));
      if (goal<latest_goal_) {return;}
      latest_goal_=goal;
      std::vector<Point3> points;
      if (j.at("points").size()>20000) {throw std::invalid_argument("oversized path");}
      for (const auto & v:j.at("points")) {
        if (!v.is_array() || v.size()!=3) {throw std::invalid_argument("invalid point");}
        Point3 p{v[0],v[1],v[2]};if (!finite_flight_point(p)) {throw std::invalid_argument("nonfinite path");}
        points.push_back(p);
      }
      if (path_goal_!=goal) {goal_since_.reset();}
      path_terminal_=allow_unknown_ ? (info.value("goal_exact",false) || info.value("state",std::string())=="SAFE_APPROACH") : info.value("goal_terminal",false);
      path_points_=std::move(points);path_stamp_=stamp;path_map_stamp_=j.at("map_stamp_ns");
      path_generation_=j.at("map_generation");path_goal_=goal;path_exact_=info.value("goal_exact",false);
      path_received_=monotonic_time();
    } catch (...) {path_points_.clear();follower_.reset();}
  }
  Point3 local_point() const {return {pos_->x,pos_->y,pos_->z};}
  bool chord(const Point3 & a,const Point3 & b) const
  {
    try {
      if (const auto * t=dynamic_cast<const octomap::OcTree*>(tree_.get())) {return flight_chord_clear(*t,a,b,radius_,allow_unknown_);}
      if (const auto * t=dynamic_cast<const octomap::ColorOcTree*>(tree_.get())) {return flight_chord_clear(*t,a,b,radius_,allow_unknown_);}
    } catch (...) {}
    return false;
  }
  bool map_point_in_bounds(const Point3 & p,bool climb) const
  {
    const auto ned=add_3d(local_point(),map_delta_to_ned(subtract_3d(p,point3(pose_->position)),correction_->orientation,yaw_offset_));
    return within_flight_bounds(ned,*ground_,geofence_-radius_,climb?-.05:min_height_,max_height_);
  }
  std::string health(bool require_path) const
  {
    const auto ns=now().nanoseconds();const auto clock=monotonic_time();
    if (!fatal_.empty()) {return fatal_;}
    if (!local_valid_ || !pos_ || !ground_ || stale(local_received_,px4_timeout_) || stale(mode_received_,px4_timeout_)) {return "PX4_STATE_STALE_OR_INVALID";}
    if (!tree_ || stale(map_received_,map_timeout_) || !fresh_stamp(map_stamp_,ns,map_timeout_)) {return "MAP_STALE";}
    if (!pose_ || !correction_ || stale(pose_received_,pose_timeout_) || !fresh_stamp(pose_stamp_,ns,pose_timeout_)) {return "POSE_STALE";}
    if (pose_generation_!=generation_ || pose_map_stamp_!=map_stamp_ ||
      !same_snapshot_transform(*correction_,map_correction_)) {return "POSE_MAP_MISMATCH";}
    if (!within_flight_bounds(local_point(),*ground_,geofence_,-.15,max_height_+.15)) {return "GEOFENCE";}
    if (info_lost_ || !fresh_stamp(info_stamp_,ns,vio_feature_timeout_) || !ev_valid_) {return "VIO_HEALTH";}
    // Inherited checks use monotonic receipts; atomic pose/info also enforce source age.
    if (stale(last_vio_pose_time_,vio_pose_timeout_) || stale(last_vio_feature_time_,vio_feature_timeout_) ||
      stale(last_vio_odometry_time_,vio_odometry_timeout_)) {return "VIO_STALE";}
    if (low_features_since_ && clock-*low_features_since_>=vio_feature_loss_time_) {return "LOW_INLIERS";}
    if (vio_at_origin_since_ && clock-*vio_at_origin_since_>=vio_reset_persist_) {return "VIO_ORIGIN_RESET";}
    if (vio_yaw_error_since_ && clock-*vio_yaw_error_since_>=vio_yaw_error_time_) {return "VIO_YAW_MISMATCH";}
    if (require_path) {
      if (path_points_.size()<2 || stale(path_received_,path_timeout_) || !fresh_stamp(path_stamp_,ns,path_timeout_)) {return "PATH_STALE";}
      if (path_generation_!=generation_ || path_map_stamp_!=map_stamp_) {return "PATH_MAP_MISMATCH";}
    }
    return {};
  }
  std::string move_to(Point3 target,bool climb)
  {
    const auto actual=local_point();auto d=subtract_3d(target,actual);
    const auto h=std::hypot(d.x,d.y);
    if (h>leash_xy_) {d.x*=leash_xy_/h;d.y*=leash_xy_/h;}
    d.z=std::clamp(d.z,-leash_z_,leash_z_);target=add_3d(actual,d);
    const auto previous=command_.value_or(limiter_->position());const auto limited=limiter_->update(target,dt_);
    // Audit the exact float-valued position published in TrajectorySetpoint.
    const Point3 candidate{static_cast<float>(limited.x),static_cast<float>(limited.y),static_cast<float>(limited.z)};
    if (!within_flight_bounds(candidate,*ground_,geofence_-radius_,climb?-.05:min_height_,max_height_)) {return "COMMAND_GEOFENCE";}
    const auto body=point3(pose_->position);
    const auto mapped=add_3d(body,ned_delta_to_map(subtract_3d(candidate,actual),correction_->orientation,yaw_offset_));
    const auto previous_mapped=add_3d(body,ned_delta_to_map(subtract_3d(previous,actual),correction_->orientation,yaw_offset_));
    if (!chord(body,mapped) || !chord(previous_mapped,mapped)) {return "FINAL_COMMAND_OBSTACLE";}
    command_=candidate;last_map_command_=mapped;previous_map_command_=previous_mapped;
    command_origin_=actual;clearance_checked_=true;return {};
  }
  void latch_hold()
  {
    if (!holding_) {if (pos_ && local_valid_) {command_=local_point();}holding_=true;}
    if (command_) {limiter_->reset(*command_);}
  }
  void clear_fault() {fault_since_.reset();holding_=false;}
  void fault(const std::string & reason)
  {
    latch_hold();follower_.reset();goal_since_.reset();
    if (!fault_since_) {fault_since_=monotonic_time();}
    publish_setpoint(hover_height_);publish_status("HOLD",reason);
    if (is_armed() && (monotonic_time()-*fault_since_>=fault_land_time_ || !fatal_.empty())) {
      trigger_landing("3D adapter: "+reason);
    }
  }
  void publish_status(const std::string & reason,const std::string & fault_reason)
  {
    Json j{{"state",reason},{"flight_state",state_name(state_)},{"fault",fault_reason},
      {"allow_unknown",allow_unknown_},{"auto_arm",auto_arm_},{"map_generation",generation_},
      {"map_stamp_ns",map_stamp_},{"pose_map_generation",pose_generation_},{"path_map_generation",path_generation_},
      {"goal_generation",path_goal_},{"required_clearance",radius_},{"artificial_start_clear",artificial_clear_},
      {"goal_exact",path_exact_},{"requested_goal_reached",reason=="GOAL_REACHED"},{"stamp_ns",now().nanoseconds()}};
    if (ground_) {j["ground_ned"]={ground_->x,ground_->y,ground_->z};}
    if (command_) {j["command_ned"]={command_->x,command_->y,command_->z};}
    if (pose_) {j["body_map"]={pose_->position.x,pose_->position.y,pose_->position.z};}
    j["setpoint_timestamp_us"]=last_setpoint_us_;j["command_clearance_checked"]=clearance_checked_;
    if (last_map_command_ && clearance_checked_) {
      j["command_map"]={last_map_command_->x,last_map_command_->y,last_map_command_->z};
      j["previous_command_map"]={previous_map_command_.x,previous_map_command_.y,previous_map_command_.z};
      j["command_origin_ned"]={command_origin_.x,command_origin_.y,command_origin_.z};
      j["map_from_odom"]=pose_json(*correction_);j["vio_yaw_offset_deg"]=yaw_offset_*180/M_PI;
    }
    std_msgs::msg::String m;m.data=j.dump();status_pub_->publish(m);
  }
  bool allow_unknown_{},fresh_goal_{},yaw_track_{},yaw_holding_{},hover_ready_{},holding_{},terminal_hold_{};
  bool clearance_checked_{},local_valid_{},ev_valid_{},info_lost_{true},path_exact_{},path_terminal_{},artificial_clear_{};
  std::string frame_,odom_frame_,path_topic_,fatal_;
  double radius_{},geofence_{},min_height_{},max_height_{},map_timeout_{},pose_timeout_{},path_timeout_{},px4_timeout_{};
  double fault_land_time_{},goal_hold_time_{},yaw_offset_{},yaw_align_{},yaw_resume_{},leash_xy_{},leash_z_{};
  Follower3DConfig config_;
  std::unique_ptr<FlightPositionLimiter3D> limiter_;
  std::unique_ptr<RouteFollower3D> follower_;
  std::optional<Point3> ground_,command_,last_map_command_;
  std::optional<double> yaw_target_,local_received_,mode_received_,map_received_,pose_received_,path_received_,fault_since_,goal_since_;
  std::optional<Pose3> pose_,correction_;Pose3 map_correction_;
  std::optional<std::array<unsigned,3>> resets_;std::optional<unsigned> ev_reset_;
  std::uint64_t local_stamp_{},last_setpoint_us_{};
  Point3 previous_map_command_{},command_origin_{};
  std::int64_t generation_{},map_stamp_{},pending_stamp_{},pose_stamp_{},pose_generation_{},pose_map_stamp_{},info_stamp_{};
  std::int64_t path_stamp_{},path_generation_{},path_map_stamp_{},path_goal_{},latest_goal_{},minimum_goal_{},follower_path_stamp_{},follower_generation_{};
  std::vector<Point3> path_points_,active_points_;
  Json pending_metadata_;
  std::unique_ptr<octomap::AbstractOcTree> tree_,pending_tree_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub_,pose_sub_,path_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::OdomInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,config_pub_;
};
}
int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<px4_vio_bridge::OffboardPlanner3D>();rclcpp::spin(node);node->on_shutdown();
  } catch (const std::exception & e) {std::fprintf(stderr,"3D flight adapter: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
