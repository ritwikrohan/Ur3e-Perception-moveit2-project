from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
            #  arguments = ['--x', '0.48', '--y', '0.68', '--z', '0.1', '--yaw', '-1.57', '--pitch', '0.174533', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
             arguments = ['--x', '0.48', '--y', '0.60', '--z', '0.1', '--yaw', '-1.57', '--pitch', '0.174533', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
        ),
    ])