# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    """Generate launch description with a component."""
    configFile = os.path.join(
        get_package_share_directory('laser_line_reconstruct'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['laser_line_reconstruct_node']['ros__parameters']

    node = ComposableNode(
        package='laser_line_reconstruct',
        plugin='laser_line_reconstruct::LaserLineReconstruct',
        remappings = [('~/line_l', '/laser_line_center_node_l/line'), ('~/line_r', '/laser_line_center_node_r/line')],
        parameters=[configParams])

    container = ComposableNodeContainer(
        name='laser_line_reconstruct_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[node],
        output='screen')

    return launch.LaunchDescription([container])