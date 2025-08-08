from setuptools import find_packages, setup

package_name = 'testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        ('share/' + package_name + '/launch', [
            'launch/oak_d_s2.launch.py', 
            'launch/rtabmap_oak.launch.py', 
            'launch/lsc_slam_mapping.launch.py',
            'launch/robot_chassis_tf_launch.py',
            'launch/robot_with_lidar_launch.py'
        ]),
        # Install all config files
        ('share/' + package_name + '/config', [
            'config/lidar_2d.lua', 
            'config/map_builder.lua', 
            'config/slam_toolbox.yaml',
            'config/rf2o_config.yaml',
            'config/tf_launch.xml'
        ]),
        # Install URDF files
        ('share/' + package_name + '/urdf', [
            'urdf/robot_chassis.urdf',
            'urdf/robot_with_lsc_lidar.urdf.xacro'
        ]),
        # Install RViz config files
        ('share/' + package_name + '/rviz', [
            'rviz/slam_config.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chumbi',
    maintainer_email='chumbi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camara = testing.camara:main',
            'oak_d_s2_depth_node = testing.oak_d_s2_depth_node:main',
            'test = testing.test:main',
            'modelo = testing.modelo:main',
        ],
    },
)
