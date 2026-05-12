"""
Launch `marker_reader` node with package config parameters.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    marker_reader_config = os.path.join(
        get_package_share_directory('ur_sorter_calibration'),
        'config',
        'phase2_marker_reader_config.yaml'
    )

    marker_reader_node = Node(
        package='ur_sorter_calibration',
        executable='marker_reader',
        name='marker_reader',
        output='screen',
        parameters=[marker_reader_config]
    )

    encoder_reader_config = os.path.join(
        get_package_share_directory('ur_sorter_calibration'),
        'config',
        'encoder_reader_config.yaml'
    )

    encoder_reader_node = Node(
        package='ur_sorter_calibration',
        executable='encoder_reader',
        name='encoder_reader',
        output='screen',
        parameters=[encoder_reader_config]
    )


    return LaunchDescription([
        marker_reader_node,
        encoder_reader_node
    ])

