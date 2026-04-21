"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('handeye_realsense'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='handeye_realsense',
            executable='robot',
            name='robot_state_estimation',
            parameters=[config]
        ),
        Node(
            package='handeye_realsense',
            executable='aruco',
            name='aruco_estimation',
            parameters=[config]
        ),
    ])