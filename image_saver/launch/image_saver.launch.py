#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

config_file = os.path.join(
    get_package_share_directory('image_saver'),
    'config',
    'image_saver_config.yaml'
)


def generate_launch_description():

    # --- Image Saver Arguments ---
    declare_save_directory_cmd = DeclareLaunchArgument(
        'save_directory',
        default_value='/tmp/captured_images',
        description='Directory to save images to.'
    )

    declare_save_rate_cmd = DeclareLaunchArgument(
        'save_rate_hz',
        default_value='1.0',
        description='Rate at which to save images (Hz).'
    )

    declare_image_prefix_cmd = DeclareLaunchArgument(
        'image_prefix',
        default_value='frame',
        description='Prefix for saved image filenames.'
    )

    declare_timestamp_source_cmd = DeclareLaunchArgument(
        'timestamp_source',
        default_value='header',        # 'header', 'wall', or 'both'
        description='Timestamp source: header (camera), wall (system), or both.'
    )

    declare_skip_duplicates_cmd = DeclareLaunchArgument(
        'skip_duplicates',
        default_value='true',
        description='Skip saving if no new frame has arrived since last save.'
    )

    # --- Include the existing pylon camera launch file ---
#    pylon_launch = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource(
#            os.path.join(
#                get_package_share_directory('pylon_ros2_camera_wrapper'),
#                'launch',
#                'pylon_ros2_camera.launch.py'
#            )
#        )
#    )

    # --- Image Saver Node --
    image_saver_node = Node(
    package='image_saver',
    executable='image_saver_node',
    name='image_saver_node',
    output='screen',
    emulate_tty=True,
    parameters=[config_file]   # ← yaml only, no overrides
)

    ld = LaunchDescription()

    ld.add_action(declare_save_directory_cmd)
    ld.add_action(declare_save_rate_cmd)
    ld.add_action(declare_image_prefix_cmd)
    ld.add_action(declare_timestamp_source_cmd)
    ld.add_action(declare_skip_duplicates_cmd)

   # ld.add_action(pylon_launch)
    ld.add_action(image_saver_node)

    return ld
