#!/usr/bin/env python3
# image_saver/utils/view_stream.py
"""display a compressed ROS image stream in an OpenCV window"""

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


class Viewer(Node):
    """simple viewer for sensor_msgs/msg/CompressedImage topics"""

    def __init__(self, topic):
        super().__init__('stream_viewer')

        qos = QoSProfile(
                reliability = ReliabilityPolicy.BEST_EFFORT,
                durability = DurabilityPolicy.VOLATILE,
                history = HistoryPolicy.KEEP_LAST,
                depth = 5,
            )

        self.create_subscription(CompressedImage, topic, self.cb, qos)
        self.get_logger().info(f'Viewing {topic}, press q to quit')

    def cb(self, msg: CompressedImage):
        """
        decode and display one compressed frame

        Inputs:
        msg (CompressedImage): compressed image message

        Outputs:
        None
        """
        frame   = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        cv2.imshow('stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main():
    """
    run the stream viewer

    Inputs:
    None

    Outputs:
    None
    """
    parser  = argparse.ArgumentParser()
    parser.add_argument('--topic', default = '/basler_cam/pylon_ros2_camera_node/image_compressed_small')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node    = Viewer(args.topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
