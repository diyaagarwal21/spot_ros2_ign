# Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# This will only take in effect if you are running THorvald in Simulation
os.environ['GAZEBO_MODEL_PATH'] = os.path.join(get_package_share_directory('champ_gazebo'),
                                               'models')


def generate_launch_description():

    # Get hare directories of thorvald packages
    champ_bringup_share_dir = get_package_share_directory(
        'champ_bringup')
    champ_description_share_dir = get_package_share_directory(
        'champ_description')

    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration("use_rviz")
    tf_prefix = LaunchConfiguration('tf_prefix')
    rviz_config = LaunchConfiguration("rviz_config")
    joy_config_filepath = LaunchConfiguration('config_filepath')

    declare_use_simulator = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='whether to use Gazebo Simulation.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use or not sim time.')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='...')
    declare_tf_prefix = DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='...')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            champ_bringup_share_dir, 'rviz', 'thorvald_default_view.rviz'),
        description='...')
    declare_joy_config_filepath = DeclareLaunchArgument(
        'config_filepath',
        default_value=os.path.join(
            champ_bringup_share_dir, 'config', 'joystick_xbox.yaml'),
        description='path to locks params.')

    # DECLARE THE msg relay ROS2 NODE
    declare_message_relay_node = Node(
        package='champ_base',
        executable='message_relay',
        name='message_relay',
        output='screen',
        namespace='',
        # prefix=['xterm -e gdb -ex run --args'],
    )

    # DECLARE THE msg relay ROS2 NODE
    declare_quadruped_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller',
        name='quadruped_controller',
        output='screen',
        namespace='',
        # prefix=['xterm -e gdb -ex run --args'],
    )

    # DECLARE THE msg relay ROS2 NODE
    declare_state_estimation_node = Node(
        package='champ_base',
        executable='state_estimation',
        name='state_estimation',
        output='screen',
        namespace='',
        # prefix=['xterm -e gdb -ex run --args'],
    )

    # DECLARE THE ROBOT STATE PUBLISHER NODE
    xacro_file_name = 'champ.urdf.xacro'
    xacro_full_dir = os.path.join(
        champ_description_share_dir, 'urdf', xacro_file_name)
    declare_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command(['xacro ', xacro_full_dir])}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')])

    declare_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        remappings=[('joint_states', 'joint_states')])

    # CALL JOYSTICK TELEOP
    joy_config_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')]),
        launch_arguments={
            'config_filepath': joy_config_filepath}.items()
    )
    # TWIST MUX FOR MIXING MULTIPLE CMD VEL COMMANDS
    twist_mux_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('twist_mux'), 'launch', 'twist_mux_launch.py')),
    )

    # SPAWN THE ROBOT TO GAZEBO IF use_simulator, FROM THE TOPIC "robot_description"
    declare_spawn_entity_to_gazebo_node = Node(package='gazebo_ros',
                                               condition=IfCondition(
                                                   use_simulator),
                                               executable='spawn_entity.py',
                                               arguments=[
                                                   '-entity', '',
                                                   '-topic', '/robot_description'],
                                               output='screen')

    # START GAZEBO ONLY IF use_simulator IS SET TO TRUE
    gazebo_world = os.path.join(
        get_package_share_directory('champ_gazebo'), 'worlds/', 'void.world'),
    declare_start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', gazebo_world,
            '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'
        ],
        condition=IfCondition(PythonExpression(
            [use_simulator])),
        output='screen')

    #  INCLUDE RVIZ LAUNCH FILE IF use_rviz IS SET TO TRUE
    declare_rviz_launch_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(champ_bringup_share_dir,
                     'launch',
                     'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
        'rviz_config': rviz_config
    }.items())

    localization_params = LaunchConfiguration('localization_params')
    decleare_localization_params = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(champ_bringup_share_dir, 'config', 'robot_localization_params.yaml'),
        description='Path to the vox_nav parameters file.')

    base_to_footprint_ekf = Node(package='robot_localization',
                                 executable='ekf_node',
                                 name='base_to_footprint_ekf',
                                 output='screen',
                                 parameters=[localization_params],
                                 remappings=[('odometry/filtered', 'odometry/local')])

    footprint_to_odom_ekf = Node(package='robot_localization',
                                 executable='ekf_node',
                                 name='footprint_to_odom_ekf',
                                 output='screen',
                                 parameters=[localization_params],
                                 remappings=[('odometry/filtered', 'odom')])

    return LaunchDescription([
        declare_use_simulator,
        declare_use_sim_time,
        declare_use_rviz,
        declare_tf_prefix,
        declare_rviz_config,
        declare_quadruped_controller_node,
        declare_state_estimation_node,
        declare_message_relay_node,
        declare_robot_state_publisher_node,
        declare_joint_state_publisher_node,
        declare_rviz_launch_include,
        declare_spawn_entity_to_gazebo_node,
        declare_start_gazebo_cmd,
        declare_joy_config_filepath,
        twist_mux_cmd,
        joy_config_cmd,
        decleare_localization_params,
        base_to_footprint_ekf,
        footprint_to_odom_ekf
    ])
