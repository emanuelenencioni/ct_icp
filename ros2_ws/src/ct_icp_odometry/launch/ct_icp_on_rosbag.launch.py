#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch description for CT-ICP SLAM on a ROS2 bag."""
    
    # Declare arguments
    declare_rosbag = DeclareLaunchArgument(
        'rosbag',
        description='Path to the ROS2 bag file'
    )
    
    declare_topic = DeclareLaunchArgument(
        'topic',
        description='Point cloud topic in the rosbag'
    )
    
    declare_rate = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate for the rosbag'
    )
    
    declare_start_sec = DeclareLaunchArgument(
        'start_sec',
        default_value='0.0',
        description='Starting time in seconds'
    )
    
    declare_config = DeclareLaunchArgument(
        'config',
        default_value='',
        description='Path to the CT-ICP config file'
    )
    
    # Rosbag playback process
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             LaunchConfiguration('rosbag'),
             '--rate', LaunchConfiguration('rate'),
             '--start-offset', LaunchConfiguration('start_sec')],
        output='screen'
    )
    
    # Include CT-ICP SLAM launch
    ct_icp_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ct_icp_odometry'),
                'launch',
                'ct_icp_slam.launch.py'
            ])
        ]),
        launch_arguments=[
            ('config', LaunchConfiguration('config')),
            ('pc_topic', LaunchConfiguration('topic')),
            ('rviz', 'true'),
        ]
    )
    
    return LaunchDescription([
        declare_rosbag,
        declare_topic,
        declare_rate,
        declare_start_sec,
        declare_config,
        rosbag_play,
        ct_icp_slam_launch,
    ])
