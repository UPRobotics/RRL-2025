# File: robot_chassis_tf_launch.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('your_package'),  # Replace with your package name
        'urdf', 'robot_chassis.urdf'
    )

    return LaunchDescription([
        # Static transform publisher: base_frame -> lsc_mount
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lsc_mount',
            arguments=[
                '0', '0', '0.1',
                '0', '0', '0', '1',
                'base_frame', 'lsc_mount'
            ]
        ),
        # Robot state publisher for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
    ])