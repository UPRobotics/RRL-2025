#!/usr/bin/env python3
"""
Launch file for LSC LiDAR with SLAM Toolbox mapping.
Includes robot description, LiDAR driver, SLAM Toolbox, and RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package and config paths
    testing_pkg = get_package_share_directory('testing')
    slam_config_file = os.path.join(testing_pkg, 'config', 'slam_toolbox.yaml')
    rf2o_config_file = os.path.join(testing_pkg, 'config', 'rf2o_config.yaml')
    urdf_file = os.path.join(testing_pkg, 'urdf', 'robot_chassis.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    lidar_ip_arg = DeclareLaunchArgument(
        'lidar_ip',
        default_value='192.168.0.235',
        description='IP address of the LSC LiDAR sensor'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # LSC LiDAR driver node
    lsc_lidar_node = Node(
        package='lsc_ros2_driver',
        executable='autonics_lsc_lidar',
        name='autonics_lsc_lidar',
        output='screen',
        parameters=[{
            'addr': LaunchConfiguration('lidar_ip'),
            'port': 8000,
            'frame_id': 'laser',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RF2O Laser Odometry node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        output='screen',
        parameters=[
            rf2o_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # SLAM Toolbox node (async mode - should auto-configure)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True
            }
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(testing_pkg, 'rviz', 'slam_config.rviz')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Note: SLAM Toolbox auto-configuration script removed since autostart: true is set
    # The node will automatically configure and activate on startup

    return LaunchDescription([
        use_sim_time_arg,
        lidar_ip_arg,
        robot_state_publisher_node,
        lsc_lidar_node,
        rf2o_node,
        slam_toolbox_node,
        rviz_node
    ])
