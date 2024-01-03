import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
import launch

# this is the function launch  system will look for


def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory(
        'get_cube_pose'), 'rviz', 'perception_real.rviz')

    perception_action_server_node = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        parameters=[{'debug_topics': True}]
    )

    get_pose_client_node = Node(
        package="get_cube_pose",
        executable="get_pose_client",
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_dir])

    

    # create and return launch description object
    return LaunchDescription(
        [
            perception_action_server_node,
            get_pose_client_node,
            rviz_node
        ]
    )
