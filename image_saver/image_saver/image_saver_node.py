#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from theo_msgs.msg import TheoCode

import cv2
import os
from datetime import datetime, timezone


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('save_directory', '/tmp/captured_images')
        self.declare_parameter('save_rate_hz', 1.0)
        self.declare_parameter('image_prefix', 'frame')
        # 'header' = camera stamp | 'wall' = system clock | 'both' = log both
        self.declare_parameter('timestamp_source', 'header')
        self.declare_parameter('skip_duplicates', True)
        self.declare_parameter('trigger_topic', '/start_capture')
        self.declare_parameter('broker_topic', '/external/broker_robotic_topic')
        self.declare_parameter('broadcast_code', 4)

        self.image_topic       = self.get_parameter('image_topic').value
        self.broker_topic      = self.get_parameter('broker_topic').value
        self.broadcast_code    = self.get_parameter('broadcast_code').value
        self.save_dir          = self.get_parameter('save_directory').value
        self.save_rate_hz      = self.get_parameter('save_rate_hz').value
        self.image_prefix      = self.get_parameter('image_prefix').value
        self.timestamp_source  = self.get_parameter('timestamp_source').value
        self.skip_duplicates   = self.get_parameter('skip_duplicates').value


        # --- Setup ---
        os.makedirs(self.save_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.latest_msg: Image | None = None
        self.last_saved_seq = None   # used for duplicate detection
        self.saved_count = 0
        self.capture_active = False


        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.subscription_brokerage = self.create_subscription(
            Image,
            self.broker_topic,
            self.broker_callback,
            10 # change
        )
        self.capture_active = False

        # --- Save timer ---
        save_period = 1.0 / self.save_rate_hz
        self.timer = self.create_timer(save_period, self.save_image)

        self.get_logger().info(
            f"ImageSaverNode started\n"
            f"  Topic      : {self.image_topic}\n"
            f"  Save dir   : {self.save_dir}\n"
            f"  Rate       : {self.save_rate_hz} Hz\n"
            f"  Timestamp  : {self.timestamp_source}\n"
            f"  Skip dupes : {self.skip_duplicates}"
        )

    def trigger_callback(self, msg: String):                  
    """Unlock saving when the trigger message arrives."""  
    if not self.capture_active:                          
        self.get_logger().info(                          
            f"Trigger received: '{msg.data}' — image saving ACTIVE"  
        )                                                
        self.capture_active = True                       

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        self.latest_msg = msg
        
    def broker_callback(self, msg: TheoCode):
        if int( msg ) == self.broadcast_code:
       	    self.capture_active = True
        else:
            self.capture_active = False

    # ------------------------------------------------------------------
    def _ros_stamp_to_datetime(self, stamp) -> tuple[datetime, int]:
        """Return (UTC datetime, nanoseconds) from a ROS stamp."""
        total_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        dt = datetime.fromtimestamp(stamp.sec + stamp.nanosec * 1e-9, tz=timezone.utc)
        return dt, total_ns

    # ------------------------------------------------------------------
    def save_image(self):
        if not self.capture_active:
            return 
            
        if self.latest_msg is None:
            self.get_logger().warn(
                'No image received yet — skipping.', throttle_duration_sec=5.0
            )
            return

        msg = self.latest_msg

        # --- Duplicate detection using header stamp as unique key ---
        if self.skip_duplicates:
            frame_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if frame_key == self.last_saved_seq:
                self.get_logger().debug('Duplicate frame — skipping.')
                return
            self.last_saved_seq = frame_key

        # --- Timestamps ---
        header_dt, header_ns = self._ros_stamp_to_datetime(msg.header.stamp)

        now_stamp = self.get_clock().now().to_msg()
        wall_dt, wall_ns = self._ros_stamp_to_datetime(now_stamp)

        if self.timestamp_source == 'header':
            file_dt, file_ns = header_dt, header_ns
        elif self.timestamp_source == 'wall':
            file_dt, file_ns = wall_dt, wall_ns
        else:  # 'both' — use header for filename, log wall too
            file_dt, file_ns = header_dt, header_ns

        # --- Build filename ---
        ms = (file_ns % 1_000_000_000) // 1_000_000
        timestamp_str = file_dt.strftime('%Y%m%d_%H%M%S') + f'_{ms:03d}ms'
        filename = f'{self.image_prefix}_{timestamp_str}.png'
        filepath = os.path.join(self.save_dir, filename)

        # --- Convert and save ---
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        if not cv2.imwrite(filepath, cv_image):
            self.get_logger().error(f'Failed to write: {filepath}')
            return

        self.saved_count += 1

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
