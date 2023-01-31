import os
from readline import append_history_file
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description with a component."""

#phoxi_control
    configFile1 = os.path.join(
        get_package_share_directory('phoxi_control'),
        'config',
        'phoxi_control_params.yaml')

    with open(configFile1, 'r') as file:
        configParams1 = yaml.safe_load(file)['phoxi_control_node']['ros__parameters']

    node1 = ComposableNode(
        package = 'phoxi_control',
        plugin = 'phoxi_control::PhoXiControl',
        name = 'phoxi_control_node',
        remappings = [('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams1],
        extra_arguments=[{'use_intra_process_comms': True}])

#floor_height
    configFile2 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'floor_height_params.yaml')

    with open(configFile2, 'r') as file:
        configParams2 = yaml.safe_load(file)['floor_height_node']['ros__parameters']

    node2 = ComposableNode(
        package = 'architecture_measurement',
        plugin='am::FloorHeight',
        name = 'floor_height_node',
        remappings = [
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/laserRanger', '/aux_box_client_node/laserRanger'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams2],
        extra_arguments=[{'use_intra_process_comms': True}])

#internal_and_external_angle
    configFile3 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'internal_and_external_angle_params.yaml')

    with open(configFile3, 'r') as file:
        configParams3 = yaml.safe_load(file)['internal_and_external_angle_node']['ros__parameters']

    node3 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::InternalAndExternalAngle',
        name = 'internal_and_external_angle_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams3],
        extra_arguments=[{'use_intra_process_comms': True}])

#levelness
    configFile4 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'levelness_params.yaml')

    with open(configFile4, 'r') as file:
        configParams4 = yaml.safe_load(file)['levelness_node']['ros__parameters']

    node4 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::Levelness',
        name = 'levelness_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams4],
        extra_arguments=[{'use_intra_process_comms': True}])

#perpendicularity
    configFile5 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'perpendicularity_params.yaml')

    with open(configFile5, 'r') as file:
        configParams5 = yaml.safe_load(file)['perpendicularity_node']['ros__parameters']

    node5 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::Perpendicularity',
        name = 'perpendicularity_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams5],
        extra_arguments=[{'use_intra_process_comms': True}])

#pillar_section_size
    configFile6 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'pillar_section_size_params.yaml')

    with open(configFile6, 'r') as file:
        configParams6 = yaml.safe_load(file)['pillar_section_size_node']['ros__parameters']

    node6 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::PillarSectionSize',
        name = 'pillar_section_size_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams6],
        extra_arguments=[{'use_intra_process_comms': True}])

#surface_flatness
    configFile7 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'surface_flatness_params.yaml')

    with open(configFile7, 'r') as file:
        configParams7 = yaml.safe_load(file)['surface_flatness_node']['ros__parameters']

    node7 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::SurfaceFlatness',
        name = 'surface_flatness_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams7],
        extra_arguments=[{'use_intra_process_comms': True}])

#opening_size
    configFile8 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'opening_size_params.yaml')

    with open(configFile8, 'r') as file:
        configParams8 = yaml.safe_load(file)['opening_size_node']['ros__parameters']

    node8 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::OpeningSize',
        name = 'opening_size_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/rollPitchYaw', '/aux_box_client_node/rollPitchYaw'),
            ('~/result','/function_node/result_subscription_ex'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams8],
        extra_arguments=[{'use_intra_process_comms': True}])

#masonry_surface_flatness
    configFile9 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'masonry_surface_flatness_params.yaml')

    with open(configFile9, 'r') as file:
        configParams9 = yaml.safe_load(file)['masonry_surface_flatness_node']['ros__parameters']

    node9 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::MasonrySurfaceFlatnessNode',
        name = 'masonry_surface_flatness_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/image', '/mortar_joint_node/image_publish'),
            ('~/inclinometer', '/aux_box_client_node/inclinometer'),
            ('~/result','/function_node/result_subscription_ex'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams9],
        extra_arguments=[{'use_intra_process_comms': True}])

#masonry_perpendicularity
    configFile10 = os.path.join(
        get_package_share_directory('architecture_measurement'),
        'config',
        'masonry_perpendicularity_params.yaml')

    with open(configFile10, 'r') as file:
        configParams10 = yaml.safe_load(file)['masonry_perpendicularity_node']['ros__parameters']

    node10 = ComposableNode(
        package = 'architecture_measurement',
        plugin = 'am::MasonryPerpendicularityNode',
        name = 'masonry_perpendicularity_node',
        remappings = [
            ('~/pointcloud', '/phoxi_control_node/pointcloud'), 
            ('~/image', '/mortar_joint_node/image_publish'),
            ('~/inclinometer', '/aux_box_client_node/inclinometer'),
            ('~/result','/function_node/result_subscription_ex'),
            ('~/status','/condition_monitoring_node/status_subscription')],
        parameters = [configParams10],
        extra_arguments=[{'use_intra_process_comms': True}])


    container1 = ComposableNodeContainer(
        name = 'pipeline_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container_mt',
        composable_node_descriptions = [node1, node2, node3, node4, node5, node6, node7, node8, node9],
        output = 'screen')

    # return launch.LaunchDescription([container1])

    embedded_parts_identification_node = Node(
        package = 'embedded_parts_identification',
        executable = 'embedded_parts_identification_node',        
        remappings = [
            ('~/image', '/phoxi_control_node/image_embedded_parts'),
            ('~/result','/function_node/result_mebedded_parts_subscription')
        ]
    )

    defect_detection_node = Node(
        package = 'defect_detection',
        executable = 'defect_detection_node',
        remappings = [
            ('~/image', '/phoxi_control_node/image_defect_detection'),
            ('~/result','/function_node/result_defect_subscription')
        ]
    )

    mortar_joint_node = Node(
        package = 'mortar_joint',
        executable = 'mortar_joint_node',
        remappings = [
            ('~/image_subscribe', '/phoxi_control_node/image_mortar_joint')            
        ]
    )


    # container1 = ComposableNodeContainer(
    #     name = 'pipeline_container',
    #     namespace = '',
    #     package = 'rclcpp_components',
    #     executable = 'component_container_mt',
    #     composable_node_descriptions = [node1, node2, node3, node4, node5, node6],
    #     output = 'screen')

    return launch.LaunchDescription([container1, embedded_parts_identification_node, defect_detection_node,mortar_joint_node])
