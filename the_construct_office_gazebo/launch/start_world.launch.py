#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('the_construct_office_gazebo')
    

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "barista_description"
    install_dir = get_package_prefix(description_package_name)

    # Add to path the models for Ingestors and Digestors
    extra_models_package = "barista_extra_models"
    extra_models_package_path = get_package_share_directory(extra_models_package)
    extra_models_path = os.path.join(extra_models_package_path, 'models')

    autogen_models_package = "the_construct_office_gazebo"
    autogen_models_package_path = get_package_share_directory(
        autogen_models_package)
    autogen_models_path = os.path.join(
        autogen_models_package_path, 'maps', 'starbots_complete', 'models')

    # Plugins
    gazebo_plugins_name = "gazebo_plugins"
    gazebo_plugins_name_path_install_dir = get_package_prefix(gazebo_plugins_name)

    # The slot car plugin is inside here
    plugin_pkg = "rmf_robot_sim_gz_classic_plugins"
    plugin_dir = get_package_prefix(plugin_pkg)

    plugin_building_pkg = "rmf_building_sim_gz_classic_plugins"
    plugin_building_dir = get_package_prefix(plugin_building_pkg)


    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + extra_models_path + ':' + autogen_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + extra_models_path+ ':' + autogen_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib' + ':' + gazebo_plugins_name_path_install_dir + '/lib' + ':' + \
            plugin_dir + '/lib' + '/rmf_robot_sim_gz_classic_plugins' + ':' + \
            plugin_building_dir + '/lib' + '/rmf_building_sim_gz_classic_plugins'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib' + ':' + gazebo_plugins_name_path_install_dir + '/lib' + ':' + plugin_dir + '/lib' + '/rmf_robot_sim_gz_classic_plugins' + \
            ':' + plugin_building_dir + '/lib' + '/rmf_building_sim_gz_classic_plugins'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_box_bot_gazebo, 'worlds', 'starbots_complete.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),
        gazebo
    ])