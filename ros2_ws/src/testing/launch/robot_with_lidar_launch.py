# File: robot_with_lidar_launch.py
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    testing_package_dir = get_package_share_directory('testing')
    
    # Read robot URDF file
    robot_urdf_file = os.path.join(testing_package_dir, 'urdf', 'robot_chassis.urdf')
    with open(robot_urdf_file, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

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

    return LaunchDescription([
        use_sim_time_arg,
        lidar_ip_arg,
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # LSC LiDAR driver node
        Node(
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
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])