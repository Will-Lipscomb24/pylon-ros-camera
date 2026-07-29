#!/usr/bin/env python3
# image_saver/image_saver/stream_compressor_node.py
"""downscale and JPEG-compress raw camera frames for live viewing"""

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    from theo_msgs.msg import TheoCode
    HAS_THEO    = True
except ImportError:
    HAS_THEO    = False
    TheoCode    = None


class StreamCompressorNode(Node):
    """ROS node that republishes a small compressed live-view image stream"""

    def __init__(self):
        super().__init__('stream_compressor_node')

        # configure camera input, compressed output, and optional broker gating
        self.declare_parameter('input_topic', '/basler_cam/pylon_ros2_camera_node/image_raw')
        self.declare_parameter('output_topic', '/basler_cam/pylon_ros2_camera_node/image_compressed_small')
        self.declare_parameter('target_width', 1024)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('stream_rate_hz', 5.0)
        self.declare_parameter('reliability', 'best_effort')
        self.declare_parameter('only_when_broadcasting', False)
        self.declare_parameter('broker_topic', '/eowyn/external/broker_robotic_topic')
        self.declare_parameter('broadcast_code', 4)

        self.input_topic             = self.get_parameter('input_topic').value
        self.output_topic            = self.get_parameter('output_topic').value
        self.target_width            = int(self.get_parameter('target_width').value)
        self.jpeg_quality            = int(self.get_parameter('jpeg_quality').value)
        self.stream_rate_hz          = float(self.get_parameter('stream_rate_hz').value)
        self.reliability             = str(self.get_parameter('reliability').value).lower()
        self.only_when_broadcasting  = self.get_parameter('only_when_broadcasting').value
        self.broker_topic            = self.get_parameter('broker_topic').value
        self.broadcast_code          = int(self.get_parameter('broadcast_code').value)

        # stream by default unless broker gating is explicitly enabled and available
        self.stream_active   = (not self.only_when_broadcasting) or (not HAS_THEO)

        # rate limit using camera timestamps so disk-save rate remains independent
        self.min_period_ns  = (1.0 / self.stream_rate_hz) * 1e9 if self.stream_rate_hz > 0 else 0.0
        self.last_stream_ns = None

        # match the camera SensorDataQoS for the raw image subscription
        image_qos   = QoSProfile(
                        reliability = ReliabilityPolicy.BEST_EFFORT,
                        durability = DurabilityPolicy.VOLATILE,
                        history = HistoryPolicy.KEEP_LAST,
                        depth = 5,
                    )
        broker_qos  = QoSProfile(
                        reliability = ReliabilityPolicy.RELIABLE,
                        durability = DurabilityPolicy.TRANSIENT_LOCAL,
                        history = HistoryPolicy.KEEP_LAST,
                        depth = 1,
                    )

        # allow reliable output when the WiFi link benefits from retransmits
        pub_reliability = (
                            ReliabilityPolicy.RELIABLE
                            if self.reliability == 'reliable'
                            else ReliabilityPolicy.BEST_EFFORT
                        )
        pub_qos         = QoSProfile(
                            reliability = pub_reliability,
                            durability = DurabilityPolicy.VOLATILE,
                            history = HistoryPolicy.KEEP_LAST,
                            depth = 5,
                        )

        self.pub = self.create_publisher(
                    CompressedImage,
                    self.output_topic,
                    pub_qos,
                )
        self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            image_qos,
        )

        if self.only_when_broadcasting:
            if HAS_THEO:
                self.create_subscription(
                    TheoCode,
                    self.broker_topic,
                    self.broker_callback,
                    broker_qos,
                )
            else:
                self.get_logger().warn('theo_msgs not found, streaming unconditionally')

        self.get_logger().info(
            'StreamCompressorNode started\n'
            f'  In topic    : {self.input_topic}\n'
            f'  Out topic   : {self.output_topic}\n'
            f'  Target width: {self.target_width} px (height from source aspect)\n'
            f'  JPEG quality: {self.jpeg_quality}\n'
            f'  Stream rate : {self.stream_rate_hz} Hz\n'
            f'  Reliability : {self.reliability}\n'
            f'  Gated       : {self.only_when_broadcasting} (active={self.stream_active})'
        )

    def broker_callback(self, msg):
        """
        update stream activity from broker code

        Inputs:
        msg (TheoCode): broker status message

        Outputs:
        None
        """
        self.stream_active = (int(msg.code) == self.broadcast_code)

    def image_callback(self, msg: Image):
        """
        resize and JPEG-encode one raw ROS image

        Inputs:
        msg (Image): raw camera image message

        Outputs:
        None
        """
        if not self.stream_active:
            return

        # throttle with the camera timestamp so old or repeated frames do not burst
        stamp_ns    = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.last_stream_ns is not None and self.min_period_ns > 0:
            if (stamp_ns - self.last_stream_ns) < self.min_period_ns:
                return
        self.last_stream_ns = stamp_ns

        # decode bgr8/rgb8 directly to avoid cv_bridge and preserve row stride
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warn(
                f"Unsupported encoding '{msg.encoding}', expected bgr8/rgb8; skipping",
                throttle_duration_sec = 5.0,
            )
            return

        try:
            arr     = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            frame   = arr[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'raw image reshape failed: {exc}', throttle_duration_sec = 5.0)
            return

        # preserve aspect ratio, deriving height from the source image
        if self.target_width > 0 and self.target_width < msg.width:
            target_h    = max(1, round(msg.height * self.target_width / msg.width))
            frame       = cv2.resize(
                            frame,
                            (self.target_width, target_h),
                            interpolation = cv2.INTER_AREA,
                        )

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            self.get_logger().error('cv2.imencode failed', throttle_duration_sec = 5.0)
            return

        out         = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data   = buf.tobytes()

        self.pub.publish(out)


def main(args=None):
    """
    spin the stream compressor node

    Inputs:
    args (list | None): ROS command line arguments

    Outputs:
    None
    """
    rclpy.init(args = args)
    node    = StreamCompressorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
