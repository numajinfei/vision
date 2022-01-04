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
        get_package_share_directory('point_cloud_collect'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['point_cloud_collect_node']['ros__parameters']

    node = ComposableNode(
        package = 'point_cloud_collect',
        plugin = 'point_cloud_collect::PointCloudCollect',
        parameters = [configParams])

    container = ComposableNodeContainer(
        name = 'point_cloud_collect_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [node],
        output = 'screen')

    return launch.LaunchDescription([container])

