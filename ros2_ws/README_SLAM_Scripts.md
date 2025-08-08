# SLAM Toolbox Manual Configuration Scripts

This directory contains scripts to manually configure and activate the SLAM Toolbox mapping node when the auto-start functionality is not working properly.

## Available Scripts

### 1. `configure_slam_simple.sh` (Recommended)
**Simple and reliable bash script for quick SLAM activation**

```bash
./configure_slam_simple.sh
```

**Features:**
- Quick SLAM Toolbox activation
- Basic verification of mapping topics
- Fallback to Python script if needed
- Easy to use and understand

### 2. `configure_slam_mapping.py` (Advanced)
**Python script with detailed lifecycle management**

```bash
python3 ./configure_slam_mapping.py
```

**Features:**
- Full lifecycle state management
- Detailed error reporting and diagnostics
- Topic verification and monitoring
- Robust error handling
- Real-time status updates

### 3. `configure_slam_mapping.sh` (Detailed)
**Comprehensive bash script with full state management**

```bash
./configure_slam_mapping.sh
```

**Features:**
- Complete lifecycle state transitions
- Colored output for better readability
- Detailed error messages
- State transition verification
- Timeout handling

### 4. `setup_slam_complete.sh` (All-in-One)
**Complete setup script that starts SLAM and configures it**

```bash
./setup_slam_complete.sh
```

**Features:**
- Automatically starts SLAM Toolbox if not running
- Configures the mapping node
- Opens new terminal windows for SLAM processes
- Complete end-to-end setup

## Quick Start Guide

### Prerequisites
1. Ensure ROS2 is installed and sourced
2. Build your workspace: `colcon build`
3. Source the workspace: `source install/setup.bash`
4. Ensure your robot/laser scanner is running

### Basic Usage

#### **Complete Launch Sequence**

**Terminal 1: Start SLAM Toolbox**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```
*Wait for SLAM Toolbox to initialize (5-10 seconds)*

**Terminal 2: Configure SLAM for Mapping**
```bash
cd ~/ros2_ws
./configure_slam_simple.sh
```

**Terminal 3: Start Your Robot/Laser Scanner**
```bash
cd ~/ros2_ws
source install/setup.bash
# Replace with your actual robot launch command
ros2 launch your_robot_package robot_launch.py
```

**Terminal 4: Launch RViz with Robot Model**
```bash
cd ~/ros2_ws
source install/setup.bash
rviz2 -d src/testing/rviz/slam_config.rviz
```

**Optional: Optimize Mapping Speed**
```bash
cd ~/ros2_ws
./optimize_slam_speed.sh
```

#### **Alternative: One-Command Setup**
```bash
# Complete automated setup
./setup_slam_complete.sh
```

#### **Verification Commands**
```bash
# Check map topic
ros2 topic echo /map --once

# Check robot pose
ros2 topic echo /pose --once

# Monitor mapping frequency
ros2 topic hz /map

# List all SLAM topics
ros2 topic list | grep -E "(map|slam|pose)"
```

## Troubleshooting

### Common Issues

1. **"SLAM Toolbox not running" error**
   - Start SLAM Toolbox: `ros2 launch slam_toolbox online_async_launch.py`
   - Wait for initialization (5-10 seconds)
   - Try the configuration script again

2. **Configuration fails**
   - Check if laser data is available: `ros2 topic echo /scan`
   - Verify transform tree: `ros2 run tf2_tools view_frames`
   - Try the Python script for better error diagnostics

3. **No map being generated**
   - Ensure robot is moving and laser is scanning
   - Check laser topic: `ros2 topic hz /scan`
   - Verify base_link to laser transform exists

4. **RViz not showing map**
   - Check if /map topic exists: `ros2 topic list | grep map`
   - Reload RViz configuration: `rviz2 -d src/testing/rviz/slam_config.rviz`
   - Set Fixed Frame to "map" in RViz

### Manual SLAM Lifecycle States

The SLAM Toolbox uses ROS2 lifecycle management with these states:

1. **unconfigured** → **configure** → **inactive**
2. **inactive** → **activate** → **active** (mapping)
3. **active** → **deactivate** → **inactive**
4. **inactive** → **cleanup** → **unconfigured**

### Manual Service Calls

If scripts fail, you can manually control SLAM Toolbox:

```bash
# Check current state
ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState

# Configure (unconfigured → inactive)
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"

# Activate (inactive → active)
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

# Deactivate (active → inactive)
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 4}}"
```

## Monitoring Commands

### Real-time Monitoring
```bash
# Monitor map updates
ros2 topic hz /map

# Monitor robot pose
ros2 topic hz /pose

# Monitor laser scans
ros2 topic hz /scan

# View transform tree
ros2 run tf2_tools view_frames
```

### Diagnostic Commands
```bash
# List all SLAM topics
ros2 topic list | grep -E "(map|slam|pose)"

# Check SLAM Toolbox parameters
ros2 param list /slam_toolbox

# Monitor SLAM feedback
ros2 topic echo /slam_toolbox/feedback
```

## Files Created

- `configure_slam_simple.sh` - Simple activation script
- `configure_slam_mapping.py` - Advanced Python configurator
- `configure_slam_mapping.sh` - Detailed bash configurator
- `setup_slam_complete.sh` - Complete setup automation
- `README_SLAM_Scripts.md` - This documentation

## Success Indicators

When SLAM is properly configured and mapping, you should see:

✅ SLAM Toolbox state: "active"  
✅ `/map` topic publishing OccupancyGrid messages  
✅ `/pose` topic publishing robot position  
✅ `/map_metadata` topic with map parameters  
✅ Map visualization in RViz  
✅ Robot pose updates as it moves  

The robot will now build a map in real-time as it navigates through the environment!
