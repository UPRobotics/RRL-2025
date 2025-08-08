import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
  xacro_file = os.path.join(
    get_package_share_directory('lsc_ros2_driver'), 
    "urdf", 
    "lsc_ros2_driver.urdf.xacro"
  )
  robot_description = xacro.process_file(xacro_file)
  params = {"robot_description": robot_description.toxml(), "use_sim_time":
            False}
  
  rviz_config_dir = os.path.join(
    get_package_share_directory('lsc_ros2_driver'),
    'rviz',
    'lsc_ros2_driver.rviz'
  )
  
  return LaunchDescription([
    Node(
      package='lsc_ros2_driver',
      executable='autonics_lsc_lidar',
      name='autonics_lsc_lidar',
      parameters=[{
        'addr': "192.168.0.235",
        'port': 8000,
        'frame_id' : "laser",
        'range_min': 0.0,
        'range_max': 25.0,
        'intensities': True,
        'pub_topic' : "scan",
        'diagnostics_tolerance' : 0.1,
        'diagnostics_windows_time' : 1,
        'angle_min' : -45.0,
        'angle_max' : 225.0,
        'angle_offset' : 0.0,
        'password': "0000",
        'ip_change' : False,
        'prev_addr' : "",
        'new_addr' : ""
      }],
      output='screen'),

    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[params],
    ),

    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d' + rviz_config_dir],
      output='screen')
  ])
