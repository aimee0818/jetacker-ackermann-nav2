#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Default files
    default_map_file = os.path.join(
        robot_gazebo_dir,
        'maps',
        'map_01.yaml'
    )

    default_params_file = os.path.join(
        robot_gazebo_dir,
        'config',
        'nav2_params.yaml'
    )

    default_rviz_file = os.path.join(
        robot_gazebo_dir,
        'rviz',
        'nav_nav2.rviz'
    )

    # Launch configurations
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    # Launch arguments
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Absolute path to the occupancy grid map YAML file'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Absolute path to the Nav2 parameter file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )

    # Standard Nav2 bringup:
    # map_server + AMCL + planner + controller + BT navigator, etc.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        declare_use_rviz,

        nav2_bringup,
        rviz_node,
    ])
