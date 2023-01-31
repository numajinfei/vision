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
        get_package_share_directory('condition_monitoring'),
        'config',
        'condition_monitoring_params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['condition_monitoring_node']['ros__params']

    node = ComposableNode(
        package = 'condition_monitoring',
        plugin = 'condition_monitoring::ConditionMonitoring',
        name = 'condition_monitoring_node',
        remappings = [('~/status_publisher', '/mqtt_ros_node/status')],
        parameters = [configParams],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
        name = 'condition_monitoring_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [node],
        output = 'screen')

    return launch.LaunchDescription([container])

