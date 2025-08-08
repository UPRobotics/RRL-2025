# LSC SLAM Mapping Launch File

This launch file (`lsc_slam_mapping.launch.py`) provides a complete solution for 2D SLAM mapping using:
- LSC ROS2 LiDAR driver
- SLAM Toolbox (without odometry)
- Static transforms
- RViz visualization

## Prerequisites

Make sure you have the following packages installed:
```bash
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-tf2-ros
```

## Usage

1. Source your workspace:
```bash
source /home/chumbi/ros2_ws/install/setup.bash
```

2. Launch the SLAM mapping:
```bash
ros2 launch testing lsc_slam_mapping.launch.py
```

3. To specify a different LiDAR IP address:
```bash
ros2 launch testing lsc_slam_mapping.launch.py lidar_ip:=192.168.1.100
```

4. To use simulation time:
```bash
ros2 launch testing lsc_slam_mapping.launch.py use_sim_time:=true
```

**Note**: The launch file includes timed sequences to properly initialize SLAM Toolbox. The sequence is:
- **0s**: LiDAR driver and static base_link→laser transform start
- **2s**: SLAM Toolbox is configured (lifecycle transition)
- **4s**: SLAM Toolbox is activated and starts mapping
- **6s**: RViz opens with proper map display

## Configuration

### LiDAR Settings
The launch file is configured for the LSC-C25CT3-ET with:
- IP: 192.168.0.235 (configurable via `lidar_ip` argument)
- Port: 8000
- Range: 0.05 - 25.0 meters
- Angle: -45° to +225° (270° field of view)
- Frame ID: laser

### SLAM Parameters
- Map resolution: 5cm
- No odometry required (scan-matching only)
- **Base frame: base_link** (standard robot frame)
- **Transform chain**: map → odom → base_link → laser
- Loop closure enabled
- Interactive mode for real-time adjustments

## Manual Map Saving

To save your map during or after mapping:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: data: '/home/chumbi/ros2_ws/my_map'"
```

## Troubleshooting

1. **LiDAR not connecting**: Check IP address and network connectivity
2. **No laser data**: Verify the `/scan` topic is publishing: `ros2 topic echo /scan`
3. **Transform errors**: Ensure static transforms are correct for your robot setup
4. **Poor mapping quality**: Adjust `minimum_travel_distance` and `minimum_travel_heading` in the SLAM config

### Common Issues and Solutions

**No map being built / SLAM Toolbox not active:**
The launch file now automatically configures and activates SLAM Toolbox. If you still have issues:
```bash
# Check SLAM Toolbox state
ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState

# Manually configure if needed
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "transition: {id: 1}"

# Manually activate if needed  
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "transition: {id: 3}"
```

**RViz message filter dropping messages:**
This is normal and can happen for several reasons:
- When switching between different fixed frames
- During SLAM initialization before map frame is established
- High laser scan frequency vs. slower transform updates
- **Solution**: Use `map` as fixed frame and wait for SLAM to initialize (move the robot)

**Transform errors in RViz:**
- Make sure all nodes are running: `ros2 node list`
- Check available topics: `ros2 topic list | grep -E "(map|scan|tf)"`
- Transform chain: map → odom → base_link → laser
- SLAM Toolbox publishes map→odom, static transform publishes base_link→laser

**RViz configuration tips:**
- Set Fixed Frame to `map` for best results
- If you see "No transform" errors, wait for SLAM to initialize and start moving the robot
- The laser scan should appear immediately, the map will build as you move

**Quick verification that SLAM is working:**
```bash
# Check if map topic is publishing
ros2 topic hz /map

# Check SLAM Toolbox status
ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState

# Check laser scan data
ros2 topic hz /scan

# Check transform chain
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link laser

# List all available frames
ros2 topic echo /tf_static --once
```

## Adjustments

- **Transform setup**: Uses standard ROS transform chain (map→odom→base_link→laser)
- **Static transform**: base_link to laser is currently set to identity (0,0,0) - adjust if needed
- Edit `/config/slam_toolbox.yaml` to fine-tune SLAM parameters
- Adjust LiDAR parameters in the launch file as needed
- **For better performance**: Reduce laser scan frequency or increase SLAM processing thresholds
