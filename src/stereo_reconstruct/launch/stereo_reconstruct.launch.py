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
        get_package_share_directory('stereo_reconstruct'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['stereo_reconstruct_node']['ros__parameters']

    node = ComposableNode(
        package = 'stereo_reconstruct',
        plugin = 'stereo_reconstruct::StereoReconstruct',
        parameters = [configParams])

    container = ComposableNodeContainer(
        name = 'stereo_reconstruct_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [node],
        output = 'screen')

    return launch.LaunchDescription([container])

