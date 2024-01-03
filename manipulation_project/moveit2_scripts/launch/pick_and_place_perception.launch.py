import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("my").to_moveit_configs()
    moveit_config = MoveItConfigsBuilder("name",package_name="my_moveit_config").to_moveit_configs()

    rviz_config_dir = os.path.join(get_package_share_directory(
        'get_cube_pose'), 'rviz', 'perception.rviz')

    perception_action_server_node = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        parameters=[{'debug_topics': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_and_place_perception",
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node,
        perception_action_server_node,
        rviz_node]
    )