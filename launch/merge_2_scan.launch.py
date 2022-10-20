#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    #general parameter for the cloud
    pointCloudTopic = LaunchConfiguration('pointCloudTopic', default="/cloud")
    pointCloutFrameId = LaunchConfiguration('pointCloutFrameId', default="lidar_frame")
    launch_ros.actions.SetParameter(name='use_sim_time', value=True),
    # 'use_sim_time' will be set on all nodes following the line above
    
    #parameter for the first laserscan, feel free to duplicate and rename for other laserscans
    scanTopic1 = LaunchConfiguration('scanTopic1', default="/scan_right")
    laser1XOff = LaunchConfiguration('laser1XOff', default=0.5)
    laser1YOff = LaunchConfiguration('laser1YOff', default=-0.5)
    laser1ZOff = LaunchConfiguration('laser1ZOff', default=0.0)
    laser1Alpha = LaunchConfiguration('laser1Alpha', default=-180.0)
    laser1AngleMin = LaunchConfiguration('laser1AngleMin', default=1.0)
    laser1AngleMax = LaunchConfiguration('laser1AngleMax', default=265.0)
    laser1R = LaunchConfiguration('laser1R', default=255)
    laser1G = LaunchConfiguration('laser1G', default=0)
    laser1B = LaunchConfiguration('laser1B', default=0)
    show1 = LaunchConfiguration('show1', default=True)

    #parameter for the second laserscan, feel free to duplicate and rename for other laserscans
    scanTopic2 = LaunchConfiguration('scanTopic2', default="/scan_left")
    laser2XOff = LaunchConfiguration('laser2XOff', default=0.5)
    laser2YOff = LaunchConfiguration('laser2YOff', default=0.5)
    laser2ZOff = LaunchConfiguration('laser2ZOff', default=0.0)
    laser2Alpha = LaunchConfiguration('laser2Alpha', default=-85.0)
    laser2AngleMin = LaunchConfiguration('laser2AngleMin', default=0.0)
    laser2AngleMax = LaunchConfiguration('laser2AngleMax', default=264.0)
    laser2R = LaunchConfiguration('laser2R', default=0)
    laser2G = LaunchConfiguration('laser2G', default=0)
    laser2B = LaunchConfiguration('laser2B', default=255)
    show2 = LaunchConfiguration('show2', default=True)

    #parameter for the third laserscan, feel free to duplicate and rename for other laserscans
    scanTopic3 = LaunchConfiguration('scanTopic3', default="/scan_center")
    laser3XOff = LaunchConfiguration('laser3XOff', default=-0.5)
    laser3YOff = LaunchConfiguration('laser3YOff', default=0.0)
    laser3ZOff = LaunchConfiguration('laser3ZOff', default=0.0)
    laser3Alpha = LaunchConfiguration('laser3Alpha', default=90.0)
    laser3AngleMin = LaunchConfiguration('laser3AngleMin', default=1.0)
    laser3AngleMax = LaunchConfiguration('laser3AngleMax', default=180.0)
    laser3R = LaunchConfiguration('laser3R', default=0)
    laser3G = LaunchConfiguration('laser3G', default=255)
    laser3B = LaunchConfiguration('laser3B', default=0)
    show3 = LaunchConfiguration('show3', default=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'pointCloudTopic',
            default_value=pointCloudTopic,
            description='desc',
        ),
        DeclareLaunchArgument(
            'pointCloutFrameId',
            default_value=pointCloutFrameId,
            description='desc',
        ),

        DeclareLaunchArgument(
            'scanTopic1',
            default_value=scanTopic1,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1XOff',
            default_value=laser1XOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1YOff',
            default_value=laser1YOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1ZOff',
            default_value=laser1ZOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1Alpha',
            default_value=laser1Alpha,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1AngleMin',
            default_value=laser1AngleMin,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1AngleMax',
            default_value=laser1AngleMax,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1R',
            default_value=laser1R,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1G',
            default_value=laser1G,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1B',
            default_value=laser1B,
            description='desc',
        ),
        DeclareLaunchArgument(
            'show1',
            default_value=show1,
            description='desc',
        ),

        DeclareLaunchArgument(
            'scanTopic2',
            default_value=scanTopic2,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2XOff',
            default_value=laser2XOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2YOff',
            default_value=laser2YOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2ZOff',
            default_value=laser2ZOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2Alpha',
            default_value=laser2Alpha,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2AngleMin',
            default_value=laser2AngleMin,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2AngleMax',
            default_value=laser2AngleMax,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2R',
            default_value=laser2R,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2G',
            default_value=laser2G,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2B',
            default_value=laser2B,
            description='desc',
        ),
        DeclareLaunchArgument(
            'show2',
            default_value=show2,
            description='desc',
        ),

        DeclareLaunchArgument(
            'scanTopic3',
            default_value=scanTopic3,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3XOff',
            default_value=laser3XOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3YOff',
            default_value=laser3YOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3ZOff',
            default_value=laser3ZOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3Alpha',
            default_value=laser3Alpha,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3AngleMin',
            default_value=laser3AngleMin,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3AngleMax',
            default_value=laser3AngleMax,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3R',
            default_value=laser3R,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3G',
            default_value=laser3G,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser3B',
            default_value=laser3B,
            description='desc',
        ),
        DeclareLaunchArgument(
            'show3',
            default_value=show3,
            description='desc',
        ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'base_link', '--child-frame-id', 'map'
        #     ]
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '-0.25',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'base_link', '--child-frame-id', 'cloud'
        #     ]
        # ),
        launch_ros.actions.Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[{
                'pointCloudTopic' : pointCloudTopic,
                'pointCloutFrameId' : pointCloutFrameId,
                'scanTopic1' : scanTopic1,
                'laser1XOff' : laser1XOff,
                'laser1YOff' : laser1YOff,
                'laser1ZOff' : laser1ZOff,
                'laser1Alpha' : laser1Alpha,
                'laser1AngleMin' : laser1AngleMin,
                'laser1AngleMax' : laser1AngleMax,
                'laser1R' : laser1R,
                'laser1G' : laser1G,
                'laser1B' : laser1B,
                'show1' : show1,
                'scanTopic2' : scanTopic2,
                'laser2XOff' : laser2XOff,
                'laser2YOff' : laser2YOff,
                'laser2ZOff' : laser2ZOff,
                'laser2Alpha' : laser2Alpha,
                'laser2AngleMin' : laser2AngleMin,
                'laser2AngleMax' : laser2AngleMax,
                'laser2R' : laser2R,
                'laser2G' : laser2G,
                'laser2B' : laser2B,
                'show2' : show2,
                'scanTopic3' : scanTopic3,
                'laser3XOff' : laser3XOff,
                'laser3YOff' : laser3YOff,
                'laser3ZOff' : laser3ZOff,
                'laser3Alpha' : laser3Alpha,
                'laser3AngleMin' : laser3AngleMin,
                'laser3AngleMax' : laser3AngleMax,
                'laser3R' : laser3R,
                'laser3G' : laser3G,
                'laser3B' : laser3B,
                'show3' : show3,
            }],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        
    ])
