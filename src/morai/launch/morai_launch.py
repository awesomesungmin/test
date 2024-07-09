from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='morai',
            executable='gps_to_utm',
            name='gps_to_utm_node',
            namespace='gps_to_utm'
        ),
        Node(
            package='morai',
            executable='morai_camera',
            name='morai_camera_node',
            namespace='camera'
        ),
        Node(
            package='morai',
            executable='morai_gps',
            name='morai_gps_node',
            namespace='gps'
        ),
        Node(
            package='morai',
            executable='morai_imu',
            name='morai_imu_node',
            namespace='imu'
        ),
    ])
