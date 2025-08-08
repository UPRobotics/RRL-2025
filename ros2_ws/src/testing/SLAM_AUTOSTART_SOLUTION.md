# SLAM Toolbox Autostart Configuration Solutions

## Problem
SLAM Toolbox starts in "unconfigured" state and doesn't automatically begin mapping.

## Root Cause
SLAM Toolbox is a **ROS2 lifecycle node** that requires explicit transitions through states:
- `unconfigured` → `inactive` → `active`

## Solutions (in order of preference)

### Solution 1: Set autostart parameter (RECOMMENDED)
Add this to your SLAM Toolbox configuration file:

```yaml
slam_toolbox:
  ros__parameters:
    # ... other parameters ...
    autostart: true  # This makes the node automatically configure and activate
```

### Solution 2: Use Nav2 lifecycle manager
If using Nav2, configure the lifecycle manager to manage SLAM Toolbox:

```yaml
lifecycle_manager:
  ros__parameters:
    node_names: ['slam_toolbox', 'map_server', 'amcl']  # Add slam_toolbox here
    autostart: true
```

### Solution 3: Manual lifecycle management (WORKAROUND)
Use the provided script `configure_slam.sh` to manually transition the lifecycle states.

### Solution 4: Launch file with lifecycle management
Create a launch file that handles the lifecycle transitions:

```python
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{'autostart': True}],  # Key parameter
        # ... other configuration
    )
    
    return LaunchDescription([
        slam_toolbox_node,
        # Optional: manually emit lifecycle events
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        ),
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            )
        ),
    ])
```

## Key Parameters for SLAM Toolbox

```yaml
slam_toolbox:
  ros__parameters:
    # Essential for automatic startup
    autostart: true
    
    # Mode configuration
    mode: "mapping"  # or "localization"
    
    # Basic parameters
    odom_frame: "odom"
    map_frame: "map"
    base_frame: "base_link"
    scan_topic: "/scan"
    
    # Performance
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
```

## Verification Commands

```bash
# Check node state
ros2 lifecycle get /slam_toolbox

# Check if mapping is working
ros2 topic echo /map --once

# List all lifecycle nodes
ros2 lifecycle list

# Manual lifecycle control (if needed)
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

## References
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)
- [Nav2 with SLAM Tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)
