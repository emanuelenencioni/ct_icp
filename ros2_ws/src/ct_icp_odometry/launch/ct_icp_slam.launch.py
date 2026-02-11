#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch description for CT-ICP SLAM with optional RViz visualization."""
    
    # Declare arguments
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    declare_config = DeclareLaunchArgument(
        'config',
        default_value='',
        description='Path to the CT-ICP config file'
    )
    
    declare_pc_topic = DeclareLaunchArgument(
        'pc_topic',
        default_value='/ct_icp/pointcloud',
        description='Point cloud input topic'
    )
    
    declare_debug_print = DeclareLaunchArgument(
        'debug_print',
        default_value='false',
        description='Enable debug printing'
    )
    
    # CT-ICP Odometry Node
    ct_icp_odometry_node = Node(
        package='ct_icp_odometry',
        executable='ct_icp_odometry_node',
        name='ct_icp_odometry_node',
        output='screen',
        remappings=[
            ('/ct_icp/pointcloud', LaunchConfiguration('pc_topic')),
        ],
        parameters=[
            {'config': LaunchConfiguration('config')},
            {'debug_print': LaunchConfiguration('debug_print')},
        ]
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('ct_icp_odometry'),
                'params',
                'ct_icp_odometry.rviz'
            ])
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        declare_rviz,
        declare_config,
        declare_pc_topic,
        declare_debug_print,
        ct_icp_odometry_node,
        rviz_node,
    ])
