# Robot with LiDAR Launch File

The `robot_with_lidar_launch.py` launch file provides a complete robot setup with:
- LSC ROS2 LiDAR driver
- Robot state publisher with chassis URDF
- RViz visualization
- Proper transform tree

## Usage

```bash
source /home/chumbi/ros2_ws/install/setup.bash
ros2 launch testing robot_with_lidar_launch.py
```

To specify a different LiDAR IP address:
```bash
ros2 launch testing robot_with_lidar_launch.py lidar_ip:=192.168.1.100
```

To use simulation time:
```bash
ros2 launch testing robot_with_lidar_launch.py use_sim_time:=true
```

## What it does

1. **Starts LSC LiDAR driver** with the same configuration as the SLAM launch file
2. **Publishes robot description** from `urdf/robot_chassis.urdf`
3. **Opens RViz** for robot and sensor visualization
4. **Establishes complete transform tree**: base_frame → lsc_mount → laser

## Transform Tree

- `base_frame`: Robot chassis base
- `lsc_mount`: LiDAR mounting point (10 cm above base)
- `laser`: LiDAR sensor frame (5.46 cm above mount, matching LSC driver)

## URDF Description

The robot chassis URDF defines:
- A simple rectangular robot base (10cm x 20cm x 10cm) in blue
- LiDAR mount 10cm above the base (small black cylinder)
- Complete transform chain matching the LSC driver specifications
- LiDAR sensor frame 5.46cm above the mount (matches LSC driver URDF)

## Use Cases

- Robot visualization in RViz
- Testing LiDAR integration with robot model
- Development and debugging
- Base for more complex robot descriptions

## Combine with SLAM

To use this robot model with SLAM mapping:

```bash
# Terminal 1: Start robot with LiDAR
ros2 launch testing robot_with_lidar_launch.py

# Terminal 2: Start SLAM (works with base_frame)
ros2 launch testing lsc_slam_mapping.launch.py
```

**Note**: You may need to adjust the SLAM configuration to use `base_frame` instead of `base_link` for complete integration, or modify the URDF to use `base_link` as the root frame.
