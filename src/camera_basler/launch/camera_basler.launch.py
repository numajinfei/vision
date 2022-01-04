'''
Descripttion: 
version: 
Author: hw
Date: 2021-10-18 20:35:48
LastEditors: hw
LastEditTime: 2021-10-18 20:36:58
'''
import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    """Generate launch description with a component."""

    configFile = os.path.join(
        get_package_share_directory('camera_basler'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['camera_basler_node']['ros__parameters']

    node = ComposableNode(
        package = 'camera_basler',
        plugin = 'camera_basler::CameraBasler',
        parameters = [configParams])

    container = ComposableNodeContainer(
        name = 'camera_basler_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [node],
        output = 'screen')

    return launch.LaunchDescription([container])
