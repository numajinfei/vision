import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    """Generate launch description with a component."""

    configFile1 = os.path.join(
        get_package_share_directory('camera_spinnaker'),
        'config',
        'params.yaml')

    with open(configFile1, 'r') as file:
        configParams1 = yaml.safe_load(file)['camera_spinnaker_node']['ros__parameters']

    node1 = ComposableNode(
        package = 'camera_spinnaker',
        plugin = 'camera_spinnaker::CameraSpinnaker',
        parameters = [configParams1],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile2 = os.path.join(
        get_package_share_directory('seam_tracking'),
        'config',
        'params.yaml')

    with open(configFile2, 'r') as file:
        handle = yaml.safe_load(file)
        configParams2 = handle['seam_tracking_node']['ros__parameters']

    node2 = ComposableNode(
        package = 'seam_tracking',
        plugin='seam_tracking::SeamTracking',
        remappings = [('~/image', '/camera_spinnaker_node/image')],
        parameters = [configParams2],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile3 = os.path.join(
        get_package_share_directory('modbus_ros'),
        'config',
        'params.yaml')

    with open(configFile3, 'r') as file:
        configParams3 = yaml.safe_load(file)['modbus_ros_node']['ros__parameters']

    node3 = ComposableNode(
        package = 'modbus_ros',
        plugin = 'modbus_ros::ModbusRos',
        remappings = [('~/coord', '/seam_tracking_node/coord')],
        parameters = [configParams3],
        extra_arguments=[{'use_intra_process_comms': True}])

    container1 = ComposableNodeContainer(
        name = 'pipeline_hc_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container_mt',
        composable_node_descriptions = [node1, node2, node3],
        output = 'screen')

    return launch.LaunchDescription([container1])
