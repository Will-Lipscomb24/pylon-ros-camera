#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

import cv2
import os
from datetime import datetime, timezone

try:
    from theo_msgs.msg import TheoCode
    HAS_THEO = True
except ImportError:
    HAS_THEO = False
    TheoCode = None


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        self.lock_ =  threading.Lock()

        # --- Parameters ---
        self.declare_parameter('image_topic',           '/camera/image_raw')
        self.declare_parameter('save_directory',        '/tmp/captured_images')
        self.declare_parameter('save_rate_hz',          0.1)
        self.declare_parameter('image_prefix',          'frame')
        self.declare_parameter('timestamp_source',      'header')
        self.declare_parameter('skip_duplicates',       True)
        self.declare_parameter('broker_topic',          '/external/broker_robotic_topic')
        self.declare_parameter('broadcast_code',        4)
        self.declare_parameter('always_capture',        False)
        self.declare_parameter('trigger_topic',         '/image_saver/trigger')
        self.declare_parameter('wait_for_event_saver',  True)   # <-- new flag
        self.declare_parameter('event_saver_timeout_s', 30.0)   # <-- how long to wait

        self.image_topic            = self.get_parameter('image_topic').value
        self.broker_topic           = self.get_parameter('broker_topic').value
        self.broadcast_code         = self.get_parameter('broadcast_code').value
        self.save_dir               = self.get_parameter('save_directory').value
        self.save_rate_hz           = self.get_parameter('save_rate_hz').value
        self.image_prefix           = self.get_parameter('image_prefix').value
        self.timestamp_source       = self.get_parameter('timestamp_source').value
        self.skip_duplicates        = self.get_parameter('skip_duplicates').value
        self.always_capture         = self.get_parameter('always_capture').value
        self.trigger_topic          = self.get_parameter('trigger_topic').value
        self.wait_for_event_saver   = self.get_parameter('wait_for_event_saver').value
        self.event_saver_timeout_s  = self.get_parameter('event_saver_timeout_s').value

        os.makedirs(self.save_dir, exist_ok=True)
        self.bridge          = CvBridge()
        self.latest_msg      = None
        self.last_saved_seq  = None
        self.saved_count     = 0
        self.capture_active  = self.always_capture or not HAS_THEO
        self.event_saver_ready = False  # gates saving until event_saver is up

        # --- QoS ---
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        broker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Subscriptions ---
        self.create_subscription(Image, self.image_topic, self.image_callback, image_qos)

        if HAS_THEO:
            self.create_subscription(TheoCode, self.broker_topic, self.broker_callback, broker_qos)
        else:
            self.get_logger().warn('theo_msgs not found — capture always active')

        # --- Trigger publisher ---
        self.trigger_pub = self.create_publisher(Header, self.trigger_topic, 10)

        # --- Event saver ready client ---
        self.ready_client = self.create_client(Trigger, '/event_saver/ready')

        # --- Save timer ---
        self.timer = self.create_timer(1.0 / self.save_rate_hz, self.save_image)

        # --- Wait for event saver if flag is set ---
        if self.wait_for_event_saver:
            self.get_logger().info(
                f'Waiting for event_saver (timeout={self.event_saver_timeout_s:.0f}s)...'
            )
            self._wait_for_event_saver()
        else:
            self.event_saver_ready = True

        self.get_logger().info(
            f'ImageSaverNode started\n'
            f'  Topic              : {self.image_topic}\n'
            f'  Save dir           : {self.save_dir}\n'
            f'  Rate               : {self.save_rate_hz} Hz\n'
            f'  Trigger topic      : {self.trigger_topic}\n'
            f'  Wait for event saver: {self.wait_for_event_saver}\n'
            f'  Event saver ready  : {self.event_saver_ready}\n'
            f'  Always capture     : {self.capture_active}'
        )

    # ------------------------------------------------------------------

    def _wait_for_event_saver(self):
        """Block until /event_saver/ready service responds or timeout."""
        deadline = self.get_clock().now().nanoseconds + int(self.event_saver_timeout_s * 1e9)

        while self.get_clock().now().nanoseconds < deadline:
            if self.ready_client.wait_for_service(timeout_sec=2.0):
                future = self.ready_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() and future.result().success:
                    self.event_saver_ready = True
                    self.get_logger().info('event_saver is ready — starting image capture')
                    return
            self.get_logger().info(
                'event_saver not ready yet — retrying...',
            )

        self.get_logger().warn(
            f'event_saver did not respond within {self.event_saver_timeout_s:.0f}s '
            f'— starting anyway'
        )
        self.event_saver_ready = True  # don't block forever

    # ------------------------------------------------------------------

    def image_callback(self, msg: Image):
        with self.lock_:
            self.latest_msg = msg

    def broker_callback(self, msg: TheoCode):
        if int(msg.code) == self.broadcast_code:
            self.capture_active = True
        else:
            self.capture_active = False

    def _ros_stamp_to_datetime(self, stamp) -> tuple[datetime, int]:
        total_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        dt = datetime.fromtimestamp(stamp.sec + stamp.nanosec * 1e-9, tz=timezone.utc)
        return dt, total_ns

    # ------------------------------------------------------------------

    def save_image(self):
        if not self.capture_active:
            return

        if not self.event_saver_ready:
            self.get_logger().info('Waiting for event_saver...', throttle_duration_sec=5.0)
            return

        if self.latest_msg is None:
            self.get_logger().warn('No image received yet — skipping.', throttle_duration_sec=5.0)
            return
        
        with self.lock_:
            msg = self.latest_msg

        if self.skip_duplicates:
            frame_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if frame_key == self.last_saved_seq:
                self.get_logger().debug('Duplicate frame — skipping.')
                return
            self.last_saved_seq = frame_key

        header_dt, header_ns = self._ros_stamp_to_datetime(msg.header.stamp)
        now_stamp            = self.get_clock().now().to_msg()
        wall_dt, wall_ns     = self._ros_stamp_to_datetime(now_stamp)

        if self.timestamp_source == 'header':
            file_dt, file_ns = header_dt, header_ns
        elif self.timestamp_source == 'wall':
            file_dt, file_ns = wall_dt, wall_ns
        else:
            file_dt, file_ns = header_dt, header_ns

        timestamp_str = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec // 1_000_000:03d}ms'
        filename      = f'{self.image_prefix}_{timestamp_str}.png'
        filepath      = os.path.join(self.save_dir, filename)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        if not cv2.imwrite(filepath, cv_image):
            self.get_logger().error(f'Failed to write: {filepath}')
            return

        self.saved_count += 1

        # Publish trigger to event_saver with the exact image stamp
        self.trigger_pub.publish(msg.header)

        log = (
            f'[{self.saved_count}] Saved: {filename}\n'
            f'  Camera stamp : {header_dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]} UTC'
        )
        if self.timestamp_source == 'both':
            log += f'\n  Wall clock   : {wall_dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]} UTC'
        self.get_logger().info(log)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Shutting down. Total saved: {node.saved_count}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
