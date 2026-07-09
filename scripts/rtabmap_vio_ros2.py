import argparse
import io
import time
import threading
import struct
import numpy as np
from PIL import Image as PILImage
import depthai as dai
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CompressedImage


def parse_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


parser = argparse.ArgumentParser(description='Publish OAK-D RTAB-Map VIO poses to ROS 2.')
parser.add_argument('--fps', type=int, default=30, help='Stereo camera FPS.')
parser.add_argument('--width', type=int, default=640, help='Stereo image width.')
parser.add_argument('--height', type=int, default=400, help='Stereo image height.')
parser.add_argument('--publish-image', type=parse_bool, default=True, help='Publish the camera feed.')
parser.add_argument('--image-format', choices=['jpeg', 'raw'], default='jpeg',
                    help='jpeg -> CompressedImage on /rtabmap/image/compressed (low latency); '
                         'raw -> Image on /rtabmap/image (heavy, backs up over Foxglove).')
parser.add_argument('--image-publish-stride', type=int, default=1,
                    help='Publish the camera feed every N frames (throttle bandwidth).')
parser.add_argument('--image-jpeg-quality', type=int, default=60,
                    help='JPEG quality 1-95 for --image-format jpeg.')
parser.add_argument('--num-features', type=int, default=1000,
                    help='FeatureTracker target features. Fewer = lighter/faster VIO (try 500).')
parser.add_argument('--path-publish-stride', type=int, default=1, help='Publish /rtabmap/path every N poses.')
parser.add_argument('--path-size', type=int, default=5000, help='Maximum poses retained in /rtabmap/path.')
parser.add_argument('--trajectory-log', default='vslam_trajectory.txt', help='Trajectory log path, or "none" to disable.')
parser.add_argument('--trajectory-flush-stride', type=int, default=30, help='Flush trajectory log every N writes.')
args = parser.parse_args()

rclpy.init()

class Ros2Node(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputTrans = dai.Node.Input(self)
        self.inputImg = dai.Node.Input(self)
        self.initialized = False  # add this

        self.ros_node = Node('rtabmap_vio_bridge')
        self.path_pub = self.ros_node.create_publisher(Path, '/rtabmap/path', 10)
        self.odom_pub = self.ros_node.create_publisher(Odometry, '/rtabmap/odometry', 10)
        self.pose_pub = self.ros_node.create_publisher(PoseStamped, '/basalt/pose', 10)

        self.publish_image = args.publish_image
        self.image_format = args.image_format
        self.image_publish_stride = max(1, args.image_publish_stride)
        self.jpeg_quality = min(95, max(1, args.image_jpeg_quality))
        self.image_count = 0

        # Best-effort, keep-last-1 so a slow Foxglove/WiFi link drops stale frames
        # instead of queueing them -- queueing is what makes the camera delay grow
        # without bound. The pose/path topics stay on their own reliable publishers.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.img_pub = None
        self.img_compressed_pub = None
        if self.publish_image:
            if self.image_format == 'jpeg':
                self.img_compressed_pub = self.ros_node.create_publisher(
                    CompressedImage, '/rtabmap/image/compressed', sensor_qos)
            else:
                self.img_pub = self.ros_node.create_publisher(
                    Image, '/rtabmap/image', sensor_qos)

        self.path_publish_stride = max(1, args.path_publish_stride)
        self.path_size = max(1, args.path_size)
        self.trajectory_flush_stride = max(1, args.trajectory_flush_stride)
        self.pose_count = 0
        self.log_count = 0

        self.traj_file = None
        if args.trajectory_log.lower() not in ('', 'none', 'false', 'off'):
            self.traj_file = open(args.trajectory_log, 'w')
            self.traj_file.write('# timestamp tx ty tz qx qy qz qw\n')

        self.path = Path()
        self.path.header.frame_id = 'world'

        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

    def run(self):
        while self.mainLoop():
            transData = self.inputTrans.get()
            # tryGet (non-blocking): never let the camera feed stall the pose
            # stream that feeds PX4 if the image queue runs dry.
            imgFrame = self.inputImg.tryGet() if self.publish_image else None

            now = self.ros_node.get_clock().now().to_msg()

            if transData is not None:
                trans = transData.getTranslation()
                quat = transData.getQuaternion()

                # add initialization check
                if not self.initialized:
                    if abs(trans.x) > 0.001 or abs(trans.y) > 0.001 or abs(trans.z) > 0.001:
                        self.initialized = True
                        print("✓ VIO initialized successfully - tracking features")
                    else:
                        print("⚠ VIO not yet initialized - move camera slowly across a textured surface")

                pose = PoseStamped()
                # ... rest of your code unchanged
                pose.header.stamp = now
                pose.header.frame_id = 'world'
                pose.pose.position.x = trans.x
                pose.pose.position.y = trans.y
                pose.pose.position.z = trans.z
                pose.pose.orientation.x = quat.qx
                pose.pose.orientation.y = quat.qy
                pose.pose.orientation.z = quat.qz
                pose.pose.orientation.w = quat.qw
                self.pose_pub.publish(pose)

                odom = Odometry()
                odom.header = pose.header
                odom.child_frame_id = 'rtabmap_body'
                odom.pose.pose = pose.pose
                self.odom_pub.publish(odom)

                self.pose_count += 1

                if self.traj_file is not None:
                    timestamp = now.sec + now.nanosec * 1e-9
                    self.traj_file.write(
                        f"{timestamp:.9f} {trans.x} {trans.y} {trans.z} "
                        f"{quat.qx} {quat.qy} {quat.qz} {quat.qw}\n"
                    )
                    self.log_count += 1
                    if self.log_count % self.trajectory_flush_stride == 0:
                        self.traj_file.flush()

                self.path.header.stamp = now
                self.path.poses.append(pose)
                if len(self.path.poses) > self.path_size:
                    self.path.poses = self.path.poses[-self.path_size:]
                if self.pose_count % self.path_publish_stride == 0:
                    self.path_pub.publish(self.path)

            if self.publish_image and imgFrame is not None:
                self.image_count += 1
                if self.image_count % self.image_publish_stride == 0:
                    self.publish_frame(imgFrame, now)

    def publish_frame(self, imgFrame, now):
        w = imgFrame.getWidth()
        h = imgFrame.getHeight()
        raw = bytes(imgFrame.getData())

        if self.img_compressed_pub is not None:
            # JPEG-encode on the Pi: a 640x400 mono frame drops from ~256 KB
            # raw to ~15-25 KB, ~15x less to push over the Foxglove WebSocket.
            try:
                arr = np.frombuffer(raw, dtype=np.uint8)
                if arr.size == w * h:
                    arr = arr.reshape(h, w)
                    buf = io.BytesIO()
                    PILImage.fromarray(arr, mode='L').save(
                        buf, format='JPEG', quality=self.jpeg_quality)
                    msg = CompressedImage()
                    msg.header.stamp = now
                    msg.header.frame_id = 'camera'
                    msg.format = 'mono8; jpeg compressed'
                    msg.data = buf.getvalue()
                    self.img_compressed_pub.publish(msg)
            except Exception as exc:  # never let a bad frame kill the loop
                print(f"image encode error: {exc}")
        elif self.img_pub is not None:
            img_msg = Image()
            img_msg.header.stamp = now
            img_msg.header.frame_id = 'camera'
            img_msg.height = h
            img_msg.width = w
            img_msg.encoding = 'mono8'
            img_msg.step = w
            img_msg.data = raw
            self.img_pub.publish(img_msg)


# Create pipeline
with dai.Pipeline() as p:
    fps = args.fps
    width = args.width
    height = args.height

    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    featureTracker = p.create(dai.node.FeatureTracker)
    odom = p.create(dai.node.RTABMapVIO)

    ros2Viewer = p.create(Ros2Node)

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    featureTracker.setHardwareResources(1, 2)
    featureTracker.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    featureTracker.initialConfig.setNumTargetFeatures(args.num_features)
    featureTracker.initialConfig.setMotionEstimator(False)
    featureTracker.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49

    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

    # Linking
    left.requestOutput((width, height)).link(stereo.left)
    right.requestOutput((width, height)).link(stereo.right)
    stereo.rectifiedLeft.link(featureTracker.inputImage)
    featureTracker.passthroughInputImage.link(odom.rect)
    stereo.depth.link(odom.depth)
    featureTracker.outputFeatures.link(odom.features)
    imu.out.link(odom.imu)

    if args.publish_image:
        odom.passthroughRect.link(ros2Viewer.inputImg)
    odom.transform.link(ros2Viewer.inputTrans)

    p.start()
    while p.isRunning():
        time.sleep(1)
