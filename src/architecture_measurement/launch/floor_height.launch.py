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
        get_package_share_directory('architecture_measurement'),
        'config',
        'floor_height_params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['floor_height_node']['ros__parameters']

    node = ComposableNode(
        package = 'architecture_measurement',
        plugin='am::FloorHeight',
        name = 'floor_height_node',
        remappings = [
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            # ('~/pointcloud', '/phoxi_control_node/pointcloud'),
            ('~/laserRanger', '/aux_box_client_node/laserRanger'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
        name = 'floor_height_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [node],
        output = 'screen')

    return launch.LaunchDescription([container])
