#!/usr/bin/env python3
#
# dsr_bringup2_spawn_on_gazebo.launch.py
# Author: Doosan Robotics (cleaned & explained)
#

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
    PythonExpression
)

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


# =====================================================
# MAIN LAUNCH
# =====================================================
def generate_launch_description():

    # -------------------------------------------------
    # 1. LAUNCH ARGUMENTS (USER INPUTS)
    # -------------------------------------------------
    arguments = [

        DeclareLaunchArgument('name',  default_value='dsr01'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1'),
        DeclareLaunchArgument('port',  default_value='12345'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50'),

        DeclareLaunchArgument('mode',  default_value='virtual'),
        DeclareLaunchArgument('model', default_value='a0509'),
        DeclareLaunchArgument('color', default_value='white'),

        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('gz', default_value='true'),

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0'),
        DeclareLaunchArgument('R', default_value='0'),
        DeclareLaunchArgument('P', default_value='0'),
        DeclareLaunchArgument('Y', default_value='0'),

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('remap_tf', default_value='false'),
    ]

    set_sim_time = SetLaunchConfiguration(
        name='use_sim_time',
        value='true'
    )

    # -------------------------------------------------
    # 2. ROBOT DESCRIPTION (URDF from XACRO)
    # -------------------------------------------------
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('dsr_description2'),
            'xacro',
            LaunchConfiguration('model')
        ]),
        '.urdf.xacro ',
        'name:=', LaunchConfiguration('name'), ' ',
        'host:=', LaunchConfiguration('host'), ' ',
        'rt_host:=', LaunchConfiguration('rt_host'), ' ',
        'port:=', LaunchConfiguration('port'), ' ',
        'mode:=', LaunchConfiguration('mode'), ' ',
        'model:=', LaunchConfiguration('model'), ' ',
        'color:=', LaunchConfiguration('color')
    ])

    robot_description_param = {
        'robot_description': robot_description
    }

    controller_yaml = PathJoinSubstitution([
        FindPackageShare('dsr_controller2'),
        'config',
        'dsr_controller2.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('dsr_description2'),
        'rviz',
        'default.rviz'
    ])

    # -------------------------------------------------
    # 3. RUN DOOSAN EMULATOR (ONLY IN VIRTUAL MODE)
    # -------------------------------------------------
    run_emulator = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace=LaunchConfiguration('name'),
        parameters=[
            {'name': LaunchConfiguration('name')},
            {'host': LaunchConfiguration('host')},
            {'rt_host': LaunchConfiguration('rt_host')},
            {'port': LaunchConfiguration('port')},
            {'mode': LaunchConfiguration('mode')},
            {'model': LaunchConfiguration('model')},
            {'rate': 100},
            {'command': True},
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])
        ),
        output='screen'
    )

    # -------------------------------------------------
    # 4. ROS2 CONTROL NODE
    # -------------------------------------------------
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description_param, controller_yaml],
        output='screen'
    )

    # -------------------------------------------------
    # 5. ROBOT STATE PUBLISHER
    # -------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description_param],
        output='both'
    )

    # -------------------------------------------------
    # 6. CONTROLLERS SPAWN
    # -------------------------------------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('name'),
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
    )

    dsr_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('name'),
        arguments=['dsr_controller2', '-c', 'controller_manager'],
    )

    # Spawn dsr controller AFTER joint_state_broadcaster
    controller_delay = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[dsr_controller],
        )
    )

    # -------------------------------------------------
    # 7. RVIZ (OPTIONAL)
    # -------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('name'),
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='log'
    )

    # -------------------------------------------------
    # 8. GAZEBO SPAWN
    # -------------------------------------------------
    gazebo_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dsr_gazebo2'),
                'launch',
                'dsr_spawn_on_gazebo.launch.py'
            )
        ),
        launch_arguments={
            'use_gazebo': LaunchConfiguration('gz'),
            'name': LaunchConfiguration('name'),
            'color': LaunchConfiguration('color'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'R': LaunchConfiguration('R'),
            'P': LaunchConfiguration('P'),
            'Y': LaunchConfiguration('Y'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'remap_tf': LaunchConfiguration('remap_tf'),
        }.items()
    )

    gazebo_delay = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_controller,
            on_exit=[gazebo_spawn_launch],
        )
    )

    # -------------------------------------------------
    # 9. TF REMAPPING OPTION
    # -------------------------------------------------
    tf_group_normal = GroupAction(
        actions=[robot_state_publisher],
        condition=UnlessCondition(LaunchConfiguration('remap_tf'))
    )

    tf_group_remap = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            robot_state_publisher
        ],
        condition=IfCondition(LaunchConfiguration('remap_tf'))
    )

    # -------------------------------------------------
    # 10. FINAL LAUNCH DESCRIPTION
    # -------------------------------------------------
    return LaunchDescription(
        arguments + [
            set_sim_time,
            run_emulator,
            ros2_control_node,
            joint_state_broadcaster,
            controller_delay,
            gazebo_delay,
            tf_group_normal,
            tf_group_remap,
            rviz,
        ]
    )
