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
        get_package_share_directory('camera_basler'),
        'config',
        'params.yaml')

    with open(configFile1, 'r') as file:
        handle = yaml.safe_load(file)
        configParams1_1 = handle['camera_basler_node_l']['ros__parameters']
        configParams1_2 = handle['camera_basler_node_r']['ros__parameters']
        # configParams1 = yaml.safe_load(file)['camera_basler_node']['ros__parameters']

    node1_1 = ComposableNode(
        package = 'camera_basler',
        plugin = 'camera_basler::CameraBasler',
        name = 'camera_node',
        parameters = [configParams1_1],
        #remappings = [('~/image', '/camera_basler_node_l/image_r')],
        extra_arguments=[{'use_intra_process_comms': True}])

    node1_2 = ComposableNode(
        package = 'camera_basler',
        plugin = 'camera_basler::CameraBasler',
        name = 'camera_node_r',
        parameters = [configParams1_2],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile2 = os.path.join(
        get_package_share_directory('laser_line_center'),
        'config',
        'params.yaml')

    with open(configFile2, 'r') as file:
        handle = yaml.safe_load(file)
        configParams2_1 = handle['laser_line_center_node_l']['ros__parameters']
        configParams2_2 = handle['laser_line_center_node_r']['ros__parameters']

    node2_1 = ComposableNode(
        package = 'laser_line_center',
        plugin='laser_line_center::LaserLineCenter',
        name = 'laser_line_center_node_l',
        remappings = [('~/image', '/camera_node/image')],
        parameters = [configParams2_1],
        extra_arguments=[{'use_intra_process_comms': True}])

    node2_2 = ComposableNode(
        package = 'laser_line_center',
        plugin='laser_line_center::LaserLineCenter',
        name = 'laser_line_center_node_r',
        remappings = [('~/image', '/camera_node_r/image')],
        parameters = [configParams2_2],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile3 = os.path.join(
        get_package_share_directory('laser_line_reconstruct'),
        'config',
        'params.yaml')

    with open(configFile3, 'r') as file:
        configParams3 = yaml.safe_load(file)['laser_line_reconstruct_node']['ros__parameters']

    node3 = ComposableNode(
        package = 'laser_line_reconstruct',
        plugin = 'laser_line_reconstruct::LaserLineReconstruct',
        remappings = [('~/line_l', '/laser_line_center_node_l/line'), ('~/line_r', '/laser_line_center_node_r/line')],
        parameters = [configParams3],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile4 = os.path.join(
        get_package_share_directory('point_cloud_collect'),
        'config',
        'params.yaml')

    with open(configFile4, 'r') as file:
        configParams4 = yaml.safe_load(file)['point_cloud_collect_node']['ros__parameters']

    node4 = ComposableNode(
        package = 'point_cloud_collect',
        plugin = 'point_cloud_collect::PointCloudCollect',
        remappings = [('~/line', '/laser_line_reconstruct_node/line')],
        parameters = [configParams4],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile5 = os.path.join(
        get_package_share_directory('point_cloud_analyse'),
        'config',
        'params.yaml')

    with open(configFile5, 'r') as file:
        configParams5 = yaml.safe_load(file)['point_cloud_analyse_node']['ros__parameters']

    node5 = ComposableNode(
        package = 'point_cloud_analyse',
        plugin = 'point_cloud_analyse::PointCloudAnalyse',
        remappings = [('~/points', '/point_cloud_collect_node/points'), ('~/result', '/common/result'), ('~/rpy', '/inclinometer_node/rpy')],
        parameters = [configParams5],
        extra_arguments=[{'use_intra_process_comms': True}])

    container1 = ComposableNodeContainer(
        name = 'pipeline_container',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container_mt',
        composable_node_descriptions = [node1_1, node1_2, node2_1, node2_2, node3, node4, node5],
        output = 'screen')

    return launch.LaunchDescription([container1])

