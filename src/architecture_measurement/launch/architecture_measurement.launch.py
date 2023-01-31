import os
from readline import append_history_file
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    """Generate launch description with a component."""

#floor_height
    configFile1 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'floor_height_params.yaml')

    with open(configFile1, 'r') as file:
        configParams1 = yaml.safe_load(file)['floor_height_node']['ros__parameters']

    node1 = ComposableNode(
        package = 'architecture_measurement',
        plugin='am::FloorHeight',
        name = 'floor_height_node',
        remappings = [
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            # ('~/pointcloud', '/phoxi_control_node/pointcloud'),
            ('~/laserRanger', '/aux_box_client_node/laserRanger'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams1],
        extra_arguments=[{'use_intra_process_comms': True}])

#internal_and_external_angle
    configFile2 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'internal_and_external_angle_params.yaml')

    with open(configFile2, 'r') as file:
        configParams2 = yaml.safe_load(file)['internal_and_external_angle_node']['ros__parameters']

    node2 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::InternalAndExternalAngle',
        name = 'internal_and_external_angle_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams2],
        extra_arguments=[{'use_intra_process_comms': True}])

#levelness
    configFile3 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'levelness_params.yaml')

    with open(configFile3, 'r') as file:
        configParams3 = yaml.safe_load(file)['levelness_node']['ros__parameters']

    node3 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::Levelness',
        name = 'levelness_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams3],
        extra_arguments=[{'use_intra_process_comms': True}])

#perpendicularity
    configFile4 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'perpendicularity_params.yaml')

    with open(configFile4, 'r') as file:
        configParams4 = yaml.safe_load(file)['perpendicularity_node']['ros__parameters']

    node4 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::Perpendicularity',
        name = 'perpendicularity_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams4],
        extra_arguments=[{'use_intra_process_comms': True}])

#pillar_section_size
    configFile5 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'pillar_section_size_params.yaml')

    with open(configFile5, 'r') as file:
        configParams5 = yaml.safe_load(file)['pillar_section_size_node']['ros__parameters']

    node5 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::PillarSectionSize',
        name = 'pillar_section_size_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams5],
        extra_arguments=[{'use_intra_process_comms': True}])

#surface_flatness
    configFile6 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'surface_flatness_params.yaml')

    with open(configFile6, 'r') as file:
        configParams6 = yaml.safe_load(file)['surface_flatness_node']['ros__parameters']

    node6 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::SurfaceFlatness',
        name = 'surface_flatness_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams6],
        extra_arguments=[{'use_intra_process_comms': True}])


#all
    container1 = ComposableNodeContainer(
        name = 'pipeline_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container_mt',
        composable_node_descriptions = [node1, node2, node3, node4, node5, node6],
        output = 'screen')

    return launch.LaunchDescription([container1])
