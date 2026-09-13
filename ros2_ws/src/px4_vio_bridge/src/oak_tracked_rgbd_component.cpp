#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <depthai_ros_msgs_v3/msg/tracked_features.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rtabmap/core/SensorData.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_odom/OdometryROS.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

namespace px4_vio_bridge
{

// ROS RTAB-Map's RGBDImage conversion currently discards the message's
// key_points field. Deriving from OdometryROS lets us preserve all of its
// odometry publication, IMU interpolation, TF and reset behavior while
// constructing SensorData with the OAK's RVC2-tracked points directly.
class OakTrackedOdometry : public rtabmap_odom::OdometryROS
{
public:
  explicit OakTrackedOdometry(const rclcpp::NodeOptions & options)
  : OdometryROS("oak_tracked_odometry", options)
  {
    // Same RTAB-Map F2M visual core as RGBDOdometry. init() also sets up the
    // standard odom, odom_info, TF, IMU buffering and reset services.
    init(false, true, false);

    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/oak/left/image_rect");
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/oak/stereo/image_raw");
    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/oak/left/camera_info");
    const auto features_topic = declare_parameter<std::string>(
      "features_topic", "/oak/left_rect_feature_tracker/tracked_features");
    const auto queue_size = static_cast<std::uint32_t>(
      std::max<std::int64_t>(2, declare_parameter<int>("input_queue_size", 10)));
    // The driver uses independently based clocks for tracks and images, so
    // stamps can differ by sub-microseconds. Allow that offset, but never a
    // neighboring 30 Hz frame: its keypoints describe different image pixels.
    const auto max_interval = declare_parameter<double>("input_max_interval", 0.001);
    const auto input_stride = declare_parameter<int>("input_stride", 1);
    if (input_stride < 1) {
      throw std::invalid_argument("input_stride must be at least 1");
    }
    input_stride_ = static_cast<std::uint32_t>(input_stride);

    feature_count_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/rtabmap/oak_feature_count", rclcpp::SensorDataQoS().keep_last(2));

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = dataCallbackGroup_;
    // Honor OdometryROS's qos setting for these custom inputs too. Reliable
    // replay publishers can then retransmit large depth messages; hard-coded
    // best-effort readers silently lose fragmented samples over DDS.
    auto input_qos = rmw_qos_profile_sensor_data;
    input_qos.reliability = static_cast<rmw_qos_reliability_policy_t>(
      get_parameter("qos").as_int());
    image_sub_.subscribe(this, image_topic, input_qos, subscription_options);
    depth_sub_.subscribe(this, depth_topic, input_qos, subscription_options);
    camera_info_sub_.subscribe(this, camera_info_topic, input_qos, subscription_options);
    features_sub_.subscribe(this, features_topic, input_qos, subscription_options);

    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(queue_size), image_sub_, depth_sub_, camera_info_sub_, features_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_interval));
    sync_->registerCallback(std::bind(
      &OakTrackedOdometry::callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    initDiagnosticMsg(
      "\noak_tracked_odometry subscribed to (approx sync):\n"
      "   " + image_topic + "\n"
      "   " + depth_topic + "\n"
      "   " + camera_info_topic + "\n"
      "   " + features_topic,
      true,
      features_topic);

    RCLCPP_INFO(
      get_logger(),
      "ROS F2M odometry using every %u synchronized OAK RVC2 tuple from %s",
      input_stride_, features_topic.c_str());
  }

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using TrackedFeatures = depthai_ros_msgs_v3::msg::TrackedFeatures;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    Image, Image, CameraInfo, TrackedFeatures>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void callback(
    const Image::ConstSharedPtr & image,
    const Image::ConstSharedPtr & depth,
    const CameraInfo::ConstSharedPtr & camera_info,
    const TrackedFeatures::ConstSharedPtr & features)
  {
    if(isPaused()) {
      return;
    }
    // ApproximateTime can emit the same image twice when a newer feature or
    // depth sample gives it another valid tuple. Odometry requires strictly
    // increasing image stamps, so discard duplicate/out-of-order tuples before
    // copying their images or ticking the input diagnostic.
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    const rclcpp::Time stamp(image->header.stamp);
    if(last_input_stamp_.nanoseconds() != 0 && stamp <= last_input_stamp_) {
      return;
    }
    last_input_stamp_ = stamp;
    // OdometryROS's max_update_rate gate runs after synchronization and its
    // processing mutex can therefore produce a data-dependent, irregular
    // frame sequence under load. A fixed source stride gives F2M predictable
    // visual displacement. Count complete, strictly ordered tuples here,
    // before image copies and before handing work to OdometryROS.
    if (input_tuple_index_++ % input_stride_ != 0) {
      return;
    }
    // OdometryROS transforms IMU measurements into frameId(). The camera
    // model MUST use that same body frame, as standard RGBDOdometry does.
    // The conversion's default identity silently treated optical coordinates
    // as body coordinates and omitting the camera lever arm.
    const auto local_transform = rtabmap_conversions::getTransform(
      frameId(), image->header.frame_id, stamp, tfBuffer(), waitForTransform());
    if(local_transform.isNull()) {
      return;
    }

    // processData() may hand this frame to OdometryROS's worker thread. Use
    // owned CvImage buffers rather than views into ROS message storage.
    const auto image_cv = cv_bridge::toCvCopy(image, image->encoding);
    const auto depth_cv = cv_bridge::toCvCopy(depth, depth->encoding);
    // This tuple is RGB-D in the left optical frame, not a right-camera
    // stereo observation. The driver puts a stereo baseline in left P[3];
    // carrying it into F2M changes its bundle-adjustment camera model. Match
    // the SDK adapter's model.left(), whose projection translation is zero.
    auto rgbd_info = *camera_info;
    rgbd_info.p[3] = 0.0;
    rgbd_info.p[7] = 0.0;
    auto camera_model = rtabmap_conversions::cameraModelFromROS(rgbd_info, local_transform);
    rtabmap::SensorData data(
      image_cv->image,
      depth_cv->image,
      camera_model,
      0,
      rtabmap_conversions::timestampFromROS(image->header.stamp));

    std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(features->features.size());
    for(const auto & feature : features->features) {
      keypoints.emplace_back(
        static_cast<float>(feature.position.x),
        static_cast<float>(feature.position.y),
        3.0F);
    }
    // Match depthai-core's RTABMapVIO adapter exactly: pass tracked locations,
    // not the device's persistent feature IDs. RTAB-Map owns correspondence
    // and local-map bookkeeping from this point onward.
    data.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());

    tick(stamp);
    std_msgs::msg::Int32 count;
    count.data = static_cast<std::int32_t>(keypoints.size());
    feature_count_pub_->publish(count);
    processData(data, image->header);
  }

  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<CameraInfo> camera_info_sub_;
  message_filters::Subscriber<TrackedFeatures> features_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr feature_count_pub_;
  std::mutex callback_mutex_;
  rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t input_tuple_index_{0};
  std::uint32_t input_stride_{1};
};

}  // namespace px4_vio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(px4_vio_bridge::OakTrackedOdometry)
