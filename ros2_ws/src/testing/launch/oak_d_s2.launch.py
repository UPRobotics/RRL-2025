from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    parameters = [
        {
            'camera': {
                'i_enable_sync': True,
                'i_pipeline_type': 'RGBD',  # RGB and depth pipeline
                'i_tf_camera_model': 'OAK-D-S2',  # Camera model
                'i_enable_nn': False,  # Disable neural network
                'i_nn_type': 'none',  # Explicitly set no NN
                'i_log_level': 'debug',  # Enable debug logging
                'i_tf_parent_frame': 'oak-d-base-frame',
            },
            'rgb': {  # Central RGB camera for color
                'i_publish_topic': True,
                'i_output_format': 'bgr',  # Color output
                'i_fps': 15.0,  # Conservative frame rate
                'i_resolution': '1080P',  # Resolution for IMX378
            },
            'stereo': {  # Stereo for depth
                'i_publish_topic': True,
                'i_align_depth': True,  # Align depth with RGB
                'i_fps': 15.0,
                'i_resolution': '800P',  # Resolution for OV9282
                'i_publish_depth': True,  # Explicitly enable depth publishing
                'i_depth_output_format': 'raw16',  # Ensure depth format
                'i_align_Depth': True,  # Align depth to RGB camera
                'i_publish_right_rect': True,  # Publish right rectified image
                'i_publish_synced_rect_pair': True,  # Publish synced rectified pair
            },
            'left': {  # Left stereo camera (grayscale)
            
                'i_publish_topic': True,
                'i_output_format': 'mono',  # Grayscale
                'i_fps': 15.0,
                'i_resolution': '800P',  # Resolution for OV9282
            },
            'right': {  # Right stereo camera (grayscale)
                'i_publish_topic': True,
                'i_output_format': 'mono',
                'i_fps': 15.0,
                'i_resolution': '800P',
            },
            'pipeline_gen': {
                'i_enable_sync': False,  # Disable sync to prevent crash
                'i_enable_imu': True,  # Keep IMU disabled
            },
            'imu': {
                'i_acc_frequency': 400.0,  # IMU frequency
                'i_gyro_frequency': 400.0,  # IMU frequency
                'i_mag_frequency': 100.0,  # Magnetometer frequency
                'i_publish_imu': True,  # Enable IMU publishing
                'i_imu_mode': 'ACC_GYRO_MAG',  # 9-axis IMU mode
                'i_message_type': 'IMU',  # Message type
            }

        }
    ]

    container = ComposableNodeContainer(
        name='depthai_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='camera',
                parameters=parameters,
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])