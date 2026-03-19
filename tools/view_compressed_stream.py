#!/usr/bin/env python3
"""Live viewer for the pylon ROS 2 compressed image stream."""

import argparse
import sys
import threading
import time
from collections import deque
from typing import Optional


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('value must be > 0')
    return parsed


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError('value must be >= 0')
    return parsed


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Subscribe to a ROS 2 CompressedImage topic and display a live OpenCV window.'
    )
    parser.add_argument(
        '--topic',
        default='/basler_cam/pylon_ros2_camera_node/image_compressed',
        help='CompressedImage topic to subscribe to.',
    )
    parser.add_argument(
        '--window-name',
        default='Basler Compressed Stream',
        help='Name of the OpenCV display window.',
    )
    parser.add_argument(
        '--display-scale',
        type=positive_float,
        default=1.0,
        help='Scale applied only to the displayed window image.',
    )
    parser.add_argument(
        '--stale-timeout-sec',
        type=positive_float,
        default=2.0,
        help='Mark the stream as stale if no frame arrives within this timeout.',
    )
    parser.add_argument(
        '--spin-timeout-sec',
        type=non_negative_float,
        default=0.01,
        help='ROS spin timeout used by the display loop.',
    )
    parser.add_argument(
        '--no-overlay',
        action='store_true',
        help='Disable the on-image status overlay.',
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage
    except ModuleNotFoundError as exc:
        parser.error(
            'Missing runtime dependency: {}. Install ROS 2 Python deps, NumPy, and OpenCV on the listener machine.'.format(
                exc.name
            )
        )
        return 2

    class CompressedStreamViewer(Node):
        def __init__(self) -> None:
            super().__init__('compressed_stream_viewer')
            self._lock = threading.Lock()
            self._latest_frame = None
            self._latest_stamp = 0.0
            self._latest_shape = None
            self._first_frame_logged = False
            self._last_decode_error_log_time = 0.0
            self._frame_times: deque[float] = deque(maxlen=200)

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self._subscription = self.create_subscription(
                CompressedImage,
                args.topic,
                self._image_callback,
                qos,
            )

        def _image_callback(self, msg: CompressedImage) -> None:
            receipt_time = time.monotonic()
            try:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('OpenCV returned an empty decoded frame')
            except Exception as exc:  # noqa: BLE001
                if receipt_time - self._last_decode_error_log_time >= 5.0:
                    self.get_logger().error(f'Failed to decode compressed frame: {exc}')
                    self._last_decode_error_log_time = receipt_time
                return

            with self._lock:
                self._latest_frame = frame
                self._latest_stamp = receipt_time
                self._latest_shape = frame.shape[:2]
                self._frame_times.append(receipt_time)

            if not self._first_frame_logged:
                self.get_logger().info(
                    f'Receiving frames on {args.topic} at {frame.shape[1]}x{frame.shape[0]}'
                )
                self._first_frame_logged = True

        def _fps(self) -> float:
            with self._lock:
                times = list(self._frame_times)
            if len(times) < 2:
                return 0.0
            duration = times[-1] - times[0]
            if duration <= 0.0:
                return 0.0
            return (len(times) - 1) / duration

        def current_display_frame(self):
            with self._lock:
                latest_frame = None if self._latest_frame is None else self._latest_frame.copy()
                latest_stamp = self._latest_stamp
                latest_shape = self._latest_shape

            now = time.monotonic()
            status = 'WAITING FOR FRAMES'
            color = (0, 200, 255)
            if latest_frame is not None:
                if now - latest_stamp > args.stale_timeout_sec:
                    status = 'STALE'
                    color = (0, 0, 255)
                else:
                    status = 'LIVE'
                    color = (0, 200, 0)

            if latest_frame is None:
                display = np.zeros((480, 854, 3), dtype=np.uint8)
            else:
                display = latest_frame

            if not args.no_overlay:
                info_lines = [status]
                if latest_shape is not None:
                    info_lines.append(f'Size: {latest_shape[1]}x{latest_shape[0]}')
                info_lines.append(f'FPS: {self._fps():.1f}')
                info_lines.append(f'Topic: {args.topic}')
                self._draw_overlay(display, info_lines, color)

            if args.display_scale != 1.0:
                display = cv2.resize(
                    display,
                    None,
                    fx=args.display_scale,
                    fy=args.display_scale,
                    interpolation=cv2.INTER_AREA if args.display_scale < 1.0 else cv2.INTER_LINEAR,
                )

            return display

        @staticmethod
        def _draw_overlay(frame, lines: list[str], color: tuple[int, int, int]) -> None:
            x = 20
            y = 32
            line_gap = 28
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.7
            thickness = 2
            for line in lines:
                cv2.putText(frame, line, (x, y), font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
                cv2.putText(frame, line, (x, y), font, scale, color, thickness, cv2.LINE_AA)
                y += line_gap

    rclpy.init(args=None)
    node = CompressedStreamViewer()

    try:
        cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
        node.get_logger().info(
            f'Viewing {args.topic} in window "{args.window_name}". Press q or Esc to exit.'
        )

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=args.spin_timeout_sec)
            frame = node.current_display_frame()
            cv2.imshow(args.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    except KeyboardInterrupt:
        pass
    except cv2.error as exc:
        node.get_logger().error(
            'OpenCV GUI failed to open or render. This viewer requires a local desktop display session: '
            f'{exc}'
        )
        return 1
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
