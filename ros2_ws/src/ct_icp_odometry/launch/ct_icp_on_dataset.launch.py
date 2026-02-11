#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch description for CT-ICP SLAM on a dataset."""
    
    # Declare arguments
    declare_dataset = DeclareLaunchArgument(
        'dataset',
        description='Name of the dataset (e.g., kitti, hilti_2021)'
    )
    
    declare_root_path = DeclareLaunchArgument(
        'root_path',
        description='Root path to the dataset directory'
    )
    
    declare_sequence = DeclareLaunchArgument(
        'sequence',
        description='Sequence name within the dataset'
    )
    
    declare_config = DeclareLaunchArgument(
        'config',
        default_value='',
        description='Path to the CT-ICP config file'
    )
    
    declare_frequency = DeclareLaunchArgument(
        'frequency',
        default_value='10',
        description='Publishing frequency in Hz'
    )
    
    # CT-ICP Dataset Node
    ct_icp_dataset_node = Node(
        package='ct_icp_odometry',
        executable='ct_icp_dataset_node',
        name='ct_icp_dataset_node',
        output='screen',
        parameters=[
            {'dataset': LaunchConfiguration('dataset')},
            {'root_path': LaunchConfiguration('root_path')},
            {'sequence': LaunchConfiguration('sequence')},
            {'frequency_hz': LaunchConfiguration('frequency')},
        ]
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
            ('pc_topic', '/ct_icp/pointcloud'),
            ('rviz', 'true'),
        ]
    )
    
    return LaunchDescription([
        declare_dataset,
        declare_root_path,
        declare_sequence,
        declare_config,
        declare_frequency,
        ct_icp_dataset_node,
        ct_icp_slam_launch,
    ])
