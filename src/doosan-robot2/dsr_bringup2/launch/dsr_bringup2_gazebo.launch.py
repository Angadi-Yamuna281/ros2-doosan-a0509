#
#  dsr_bringup2
#  Author: Minsoo Song (Doosan Robotics)
#  Updated: 2025
#  Description: Launch file for A0509 with Tic Tac Toe board in Gazebo
#

import os

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ------------------------
    # Launch Arguments
    # ------------------------
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='dsr01', description='Namespace'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='Robot IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='Robot Port'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='Operation Mode'),
        DeclareLaunchArgument('model', default_value='a0509', description='Robot Model'),
        DeclareLaunchArgument('color', default_value='white', description='Robot Color'),
        DeclareLaunchArgument('gui',   default_value='false', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='true', description='Use Gazebo'),
        DeclareLaunchArgument('x',     default_value='0', description='Gazebo X'),
        DeclareLaunchArgument('y',     default_value='0', description='Gazebo Y'),
        DeclareLaunchArgument('z',     default_value='0', description='Gazebo Z'),
        DeclareLaunchArgument('R',     default_value='0', description='Gazebo Roll'),
        DeclareLaunchArgument('P',     default_value='0', description='Gazebo Pitch'),
        DeclareLaunchArgument('Y',     default_value='0', description='Gazebo Yaw'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='Robot RT IP'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Simulation Time'),
        DeclareLaunchArgument('remap_tf', default_value='false', description='Remap TF'),
    ]

    set_use_sim_time = SetLaunchConfiguration(name='use_sim_time', value='true')
    gui = LaunchConfiguration('gui')
    mode = LaunchConfiguration('mode')

    xacro_path = os.path.join(get_package_share_directory('dsr_description2'), 'xacro')

    # ------------------------
    # Robot Description
    # ------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution([FindPackageShare('dsr_description2'), 'xacro', LaunchConfiguration('model')]),
            ".urdf.xacro",
            " name:=", LaunchConfiguration('name'),
            " host:=", LaunchConfiguration('host'),
            " rt_host:=", LaunchConfiguration('rt_host'),
            " port:=", LaunchConfiguration('port'),
            " mode:=", LaunchConfiguration('mode'),
            " model:=", LaunchConfiguration('model'),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare('dsr_description2'), 'rviz', 'default.rviz'])

    # ------------------------
    # Emulator Node (Virtual Mode)
    # ------------------------
    run_emulator_node = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace=LaunchConfiguration('name'),
        parameters=[
            {'name': LaunchConfiguration('name')},
            {'rate': 100},
            {'standby': 5000},
            {'command': True},
            {'host': LaunchConfiguration('host')},
            {'port': LaunchConfiguration('port')},
            {'mode': LaunchConfiguration('mode')},
            {'model': LaunchConfiguration('model')},
            {'gripper': 'none'},
            {'mobile': 'none'},
            {'rt_host': LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output='screen',
    )

    # ------------------------
    # Gazebo Connection Node
    # ------------------------
    gazebo_connection_node = Node(
        package='dsr_bringup2',
        executable='gazebo_connection',
        namespace=LaunchConfiguration('name'),
        parameters=[{'model': LaunchConfiguration('model')}],
        output='log',
    )

    # ------------------------
    # Controller Node
    # ------------------------
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description, robot_controllers],
    )

    # ------------------------
    # Robot State Publisher
    # ------------------------
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])
        }],
    )

    # ------------------------
    # RViz Node
    # ------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('name'),
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui),
    )

    # ------------------------
    # Robot Controller Spawner
    # ------------------------
    robot_controller_spawner = Node(
        package='controller_manager',
        namespace=LaunchConfiguration('name'),
        executable='spawner',
        arguments=['dsr_controller2', '-c', 'controller_manager'],
    )

    delay_rviz_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=robot_controller_spawner, on_exit=[rviz_node])
    )

    # ------------------------
    # Joint State Broadcaster
    # ------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        namespace=LaunchConfiguration('name'),
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        parameters=[PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'joint_state_broadcaster.yaml'])]
    )

    # ------------------------
    # Include Gazebo Launch
    # ------------------------
    included_launch_file_path = os.path.join(get_package_share_directory('dsr_gazebo2'), 'launch', 'dsr_gazebo.launch.py')
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file_path),
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
        }.items(),
    )

    included_launch_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=robot_controller_spawner, on_exit=[included_launch])
    )

    # ------------------------
    # Spawn Tic Tac Toe Board (Adjusted Position)
    # ------------------------
    tictactoe_model_path = os.path.expanduser('~/ros3_ws/src/doosan-robot2/dsr_gazebo2/model/tictactoe_board/model.sdf')
    spawn_tictactoe = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', tictactoe_model_path,
            '-name', 'tictactoe_board',
            '-x', '0.45',   # distance in front of A0509
            '-y', '0.0',    # centered
            '-z', '0.02',   # slightly above ground
        ],
        output='screen'
    )

    # ------------------------
    # Launch Order
    # ------------------------
    nodes = [
        set_use_sim_time,
        run_emulator_node,
        gazebo_connection_node,
        robot_state_pub_node,
        rviz_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        included_launch_after_controller,
        control_node,
        spawn_tictactoe,   # ✅ board spawn
    ]

    return LaunchDescription(ARGUMENTS + nodes)
