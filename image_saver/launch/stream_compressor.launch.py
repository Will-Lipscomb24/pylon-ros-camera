#!/usr/bin/env python3
# image_saver/launch/stream_compressor.launch.py
"""launch the image_saver stream compressor node"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

config_file = os.path.join(
                get_package_share_directory('image_saver'),
                'config',
                'stream_compressor_config.yaml',
            )


def generate_launch_description():
    """
    create the launch description for stream compression

    Inputs:
    None

    Outputs:
    ld (LaunchDescription): launch description containing the compressor node
    """
    stream_compressor_node  = Node(
                                package = 'image_saver',
                                executable = 'stream_compressor_node',
                                name = 'stream_compressor_node',
                                output = 'screen',
                                emulate_tty = True,
                                parameters = [config_file],
                            )

    ld  = LaunchDescription()
    ld.add_action(stream_compressor_node)

    return ld
