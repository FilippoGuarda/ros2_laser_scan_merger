#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#

import os
from ament_index_python.packages import get_package_share_directory
from distutils.command.config import config
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'config',
        'laser_params.yaml'
    )

    return LaunchDescription([
        
        launch_ros.actions.Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        
    ])
