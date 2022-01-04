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
        get_package_share_directory('camera_galaxy'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['camera_galaxy_node']['ros__parameters']

    node = ComposableNode(
        package = 'camera_galaxy',
        plugin = 'camera_galaxy::CameraGalaxy',
        parameters = [configParams])

    container = ComposableNodeContainer(
        name = 'camera_galaxy_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [node],
        output = 'screen')

    return launch.LaunchDescription([container])
