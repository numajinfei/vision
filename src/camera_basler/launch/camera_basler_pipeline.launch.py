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
        handle = yaml.safe_load(file)
        configParams1 = handle['camera_basler_node_l']['ros__parameters']
        configParams2 = handle['camera_basler_node_r']['ros__parameters']

    node1 = ComposableNode(
        package = 'camera_basler',
        plugin = 'camera_basler::CameraBasler',
        name = 'camera_basler_node_l',
        parameters = [configParams1])

    node2 = ComposableNode(
        package = 'camera_basler',
        plugin = 'camera_basler::CameraBasler',
        name = 'camera_basler_node_r',
        parameters = [configParams2])

    container = ComposableNodeContainer(
        name = 'camera_basler_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        # composable_node_descriptions = [node1,node2],
        composable_node_descriptions = [node1, node2],
        output = 'screen')

    return launch.LaunchDescription([container])
