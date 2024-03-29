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
        get_package_share_directory('mqtt_ros'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['mqtt_ros_node']['ros__parameters']

    node = ComposableNode(
        package = 'mqtt_ros',
        plugin = 'mqtt_ros::MqttRos',
        parameters = [configParams])

    container = ComposableNodeContainer(
        name = 'mqtt_ros_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [node],
        output = 'screen')

    return launch.LaunchDescription([container])

