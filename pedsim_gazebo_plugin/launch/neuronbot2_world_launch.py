# Copyright (c) 2018 Intel Corporation
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
#!/usr/bin/env python3

"""Test gazebo_plugins for pedsim."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    # pedsim_dir = get_package_share_directory('pedsim_gazebo_plugin')
    nb2_gazebo_dir = get_package_share_directory('neuronbot2_gazebo')
    launch_dir = os.path.join(nb2_gazebo_dir, 'launch')
    nb2_view_dir = get_package_share_directory('neuronbot2_description')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    pedsim_gazebo_path = os.path.join(get_package_share_directory('pedsim_gazebo_plugin'), 'models')
    nb2_gazebo_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models')
    gazebo_model_path = pedsim_gazebo_path + ':' + nb2_gazebo_path;

    print(gazebo_model_path)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(os.environ['GAZEBO_MODEL_PATH'])

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(nb2_gazebo_dir, 'worlds', 'mememan_world.model'),
        description='Full path to world model file to load')

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', 
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        world],
        cwd=[launch_dir], output='screen')


    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')
    
    nb2_spawn_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file' , nb2_gazebo_path + '/neuronbot2_w_top_camera/' + 'model.sdf', '-entity', 'nb2'],
        cwd=[launch_dir], output='screen')

    agent_spawner_cmd = Node(
        package='pedsim_gazebo_plugin',
        node_executable='spawn_pedsim_agents.py',
        node_name='spawn_pedsim_agents',
        output='screen')

    nb2_urdf = os.path.join(nb2_view_dir, 'urdf', 'neuronbot2_w_top_camera.urdf')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        node_namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[nb2_urdf])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(nb2_spawn_cmd)
    ld.add_action(agent_spawner_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
