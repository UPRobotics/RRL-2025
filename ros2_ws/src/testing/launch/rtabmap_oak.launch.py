from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                {'frame_id': 'oak'},
                {'subscribe_depth': True},
                {'subscribe_rgb': True},
                {'map_frame_id': 'map'},
                {'Grid/FromDepth': True},
                {'Grid/RangeMax': 10.0},
                {'RGBD/OptimizeMaxError': 3.0},
                {'use_sim_time': False}
            ],
            remappings=[
                ('rgb/image', '/oak/rgb/image_rect'),
                ('rgb/camera_info', '/oak/rgb/camera_info'),
                ('depth/image', '/oak/stereo/image_raw'),  # or /image_rect/zstd if depth
                ('depth/camera_info', '/oak/stereo/camera_info')
            ],
            output='screen'
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[
                {'frame_id': 'oak'},
                {'subscribe_depth': True},
                {'subscribe_rgb': True}
            ],
            remappings=[
                ('rgb/image', '/oak/rgb/image_rect'),
                ('rgb/camera_info', '/oak/rgb/camera_info'),
                ('depth/image', '/oak/stereo/image_raw'),  # or /image_rect/zstd
                ('depth/camera_info', '/oak/stereo/camera_info')
            ]
        )
    ])