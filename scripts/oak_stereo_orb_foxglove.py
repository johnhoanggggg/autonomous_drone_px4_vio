#!/usr/bin/env python3

import argparse
import time
from collections import deque

import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def make_image_msg(stamp, frame_id, width, height, encoding, data):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = width if encoding == "mono8" else width * 3
    msg.data = bytes(data)
    return msg


def gray_to_bgr(gray):
    out = bytearray(len(gray) * 3)
    j = 0
    for value in gray:
        out[j] = value
        out[j + 1] = value
        out[j + 2] = value
        j += 3
    return out


def set_pixel_bgr(buf, width, height, x, y, b, g, r):
    if x < 0 or y < 0 or x >= width or y >= height:
        return
    idx = (y * width + x) * 3
    buf[idx] = b
    buf[idx + 1] = g
    buf[idx + 2] = r


def draw_cross(buf, width, height, x, y):
    for delta in range(-3, 4):
        set_pixel_bgr(buf, width, height, x + delta, y, 0, 40, 255)
        set_pixel_bgr(buf, width, height, x, y + delta, 0, 40, 255)


def draw_line(buf, width, height, x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    while True:
        set_pixel_bgr(buf, width, height, x0, y0, 255, 0, 220)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


class OakStereoOrbPublisher(Node):
    def __init__(self):
        super().__init__("oak_stereo_orb_publisher")
        self.left_pub = self.create_publisher(Image, "/oak/left/image", 10)
        self.right_pub = self.create_publisher(Image, "/oak/right/image", 10)
        self.orb_pub = self.create_publisher(Image, "/oak/orb/image", 10)
        self.paths = {}

    def publish_frames(self, left_frame, right_frame, features):
        stamp = self.get_clock().now().to_msg()

        left_width = left_frame.getWidth()
        left_height = left_frame.getHeight()
        right_width = right_frame.getWidth()
        right_height = right_frame.getHeight()

        left_gray = bytes(left_frame.getData())
        right_gray = bytes(right_frame.getData())

        self.left_pub.publish(
            make_image_msg(stamp, "oak_left", left_width, left_height, "mono8", left_gray)
        )
        self.right_pub.publish(
            make_image_msg(stamp, "oak_right", right_width, right_height, "mono8", right_gray)
        )

        overlay_width = left_width + right_width
        overlay_height = max(left_height, right_height)
        left_bgr = gray_to_bgr(left_gray)
        right_bgr = gray_to_bgr(right_gray)
        overlay = bytearray(overlay_width * overlay_height * 3)

        for y in range(left_height):
            src = y * left_width * 3
            dst = y * overlay_width * 3
            overlay[dst : dst + left_width * 3] = left_bgr[src : src + left_width * 3]

        for y in range(right_height):
            src = y * right_width * 3
            dst = (y * overlay_width + left_width) * 3
            overlay[dst : dst + right_width * 3] = right_bgr[src : src + right_width * 3]

        active_ids = set()
        for feature in features:
            feature_id = feature.id
            active_ids.add(feature_id)
            x = int(feature.position.x)
            y = int(feature.position.y)
            path = self.paths.setdefault(feature_id, deque(maxlen=12))
            path.append((x, y))

        for feature_id in list(self.paths):
            if feature_id not in active_ids:
                del self.paths[feature_id]

        for path in self.paths.values():
            points = list(path)
            for index in range(len(points) - 1):
                draw_line(overlay, overlay_width, overlay_height, *points[index], *points[index + 1])
            if points:
                draw_cross(overlay, overlay_width, overlay_height, *points[-1])

        self.orb_pub.publish(
            make_image_msg(stamp, "oak_stereo_orb", overlay_width, overlay_height, "bgr8", overlay)
        )


def build_pipeline(width, height, fps, max_features):
    pipeline = dai.Pipeline()

    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)

    left_out = left.requestOutput((width, height), type=dai.ImgFrame.Type.GRAY8, fps=fps)
    right_out = right.requestOutput((width, height), type=dai.ImgFrame.Type.GRAY8, fps=fps)

    feature_tracker = pipeline.create(dai.node.FeatureTracker)
    feature_tracker.initialConfig.setCornerDetector(
        dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS
    )
    feature_tracker.initialConfig.setNumTargetFeatures(max_features)
    motion_estimator = dai.FeatureTrackerConfig.MotionEstimator()
    motion_estimator.enable = True
    feature_tracker.initialConfig.setMotionEstimator(motion_estimator)
    feature_tracker.setHardwareResources(2, 2)

    left_out.link(feature_tracker.inputImage)

    return (
        pipeline,
        right_out.createOutputQueue(maxSize=1, blocking=False),
        feature_tracker.passthroughInputImage.createOutputQueue(maxSize=1, blocking=False),
        feature_tracker.outputFeatures.createOutputQueue(maxSize=1, blocking=False),
    )


def main():
    parser = argparse.ArgumentParser(description="Publish OAK stereo images and tracked features.")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--max-features", type=int, default=256)
    args = parser.parse_args()

    rclpy.init()
    node = OakStereoOrbPublisher()
    pipeline, right_queue, left_queue, feature_queue = build_pipeline(
        args.width, args.height, args.fps, args.max_features
    )

    try:
        pipeline.start()
        node.get_logger().info(
            "Publishing /oak/left/image, /oak/right/image, and /oak/orb/image"
        )
        while rclpy.ok() and pipeline.isRunning():
            rclpy.spin_once(node, timeout_sec=0.0)
            left_frame = left_queue.get()
            features = feature_queue.get().trackedFeatures
            right_frame = right_queue.get()
            node.publish_frames(left_frame, right_frame, features)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
