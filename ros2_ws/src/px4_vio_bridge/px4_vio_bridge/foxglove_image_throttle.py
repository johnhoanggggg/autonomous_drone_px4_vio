"""Drop excess visualization frames without changing camera/VIO inputs."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


class FrameGate:
    def __init__(self, rate_hz):
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('rate_hz must be finite and positive')
        self.period = 1.0 / rate_hz
        self.last_sent = None

    def accept(self, now):
        if self.last_sent is not None and now - self.last_sent < self.period:
            return False
        self.last_sent = now
        return True


class FoxgloveImageThrottle(Node):
    def __init__(self):
        super().__init__('foxglove_image_throttle')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('input_topic', '/oak/left/image_rect/compressed')
        self.declare_parameter('output_topic', '/foxglove/camera/image/compressed')
        self.gate = FrameGate(float(self.get_parameter('rate_hz').value))
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(
            CompressedImage, self.get_parameter('output_topic').value, qos)
        self.subscription = self.create_subscription(
            CompressedImage, self.get_parameter('input_topic').value, self.receive, qos)

    def receive(self, message):
        # No timer, queue, decoding or re-encoding: never replay an old frame.
        if self.publisher.get_subscription_count() and self.gate.accept(time.monotonic()):
            self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveImageThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
