#!/usr/bin/env/ python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch_ros.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
        get_package_share_directory('fusion'),
        'param',
        'A_bfs.yaml'))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to param file to load'),

        Node(
            package='fusion',
            executable='compare_box_image_deliver',
            name='compare_box_image_deliver',
            parameters=[param_dir],
            output='screen'),

        Node(
            package='fusion',
            executable='lidar_pre_deli',
            name='lidar_pre_deli',
            output='screen'),
    ])        
        
    
