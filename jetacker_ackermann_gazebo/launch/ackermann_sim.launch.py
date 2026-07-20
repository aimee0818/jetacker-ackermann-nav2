#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------
    ackermann_gazebo_dir = get_package_share_directory(
        'jetacker_ackermann_gazebo'
    )

    robot_gazebo_dir = get_package_share_directory(
        'robot_gazebo'
    )

    ros_gz_sim_dir = get_package_share_directory(
        'ros_gz_sim'
    )

    # ---------------------------------------------------------
    # File paths
    # ---------------------------------------------------------
    robot_xacro_file = os.path.join(
        ackermann_gazebo_dir,
        'urdf',
        'jetacker_ackermann.gazebo.xacro'
    )

    world_file = os.path.join(
        robot_gazebo_dir,
        'worlds',
        'robocup_home.sdf'
    )

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro_file
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # ---------------------------------------------------------
    # Gazebo Sim
    # ---------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
    )
    
    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_gazebo_dir,
                'launch',
                'spawn_objects.launch.py'
            )
        )
    )

    # ---------------------------------------------------------
    # Spawn robot
    # ---------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_jetacker_ackermann',
        output='screen',
        arguments=[
            '-name', 'jetacker_ackermann',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
            '-Y', '0.0',
        ]
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/odom/tf', '/tf'),
        ],
        output='screen'
    )
    
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map',
            'odom',
        ],
        output='screen'
    )
    
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Controller spawners
    # ---------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'steering_position_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    rear_wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rear_wheel_velocity_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    # 로봇 생성이 끝난 뒤 controller들을 실행
    controller_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                steering_controller_spawner,
                rear_wheel_controller_spawner,
            ]
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        odom_bridge,
        scan_bridge,
        map_to_odom_tf,
        spawn_objects,
        controller_start_handler,
    ])
