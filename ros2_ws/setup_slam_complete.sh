#!/bin/bash

# SLAM Toolbox Complete Setup Script
# This script starts SLAM Toolbox and configures it for mapping

echo "================================================"
echo "SLAM Toolbox Complete Setup and Configuration"
echo "================================================"
echo

# Source ROS2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[INFO] Sourced ROS2 Jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[INFO] Sourced ROS2 Humble"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    echo "[INFO] Sourced ROS2 Galactic"
else
    echo "[ERROR] ROS2 installation not found"
    exit 1
fi

# Source workspace
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
    echo "[INFO] Sourced workspace overlay"
fi

echo

# Function to check if SLAM Toolbox is running
check_slam_running() {
    if ros2 service list | grep -q "/slam_toolbox/get_state"; then
        return 0
    else
        return 1
    fi
}

# Function to check if robot chassis TF is running
check_robot_tf_running() {
    if ros2 node list | grep -q "robot_chassis_tf"; then
        return 0
    else
        return 1
    fi
}

# Start LSC SLAM mapping launch file first
if ! check_robot_tf_running; then
    echo "[INFO] Starting LSC SLAM mapping launch file..."
    echo "[CMD] ros2 launch testing lsc_slam_mapping.launch.py"
    echo
    
    # Start LSC SLAM mapping in background
    gnome-terminal -- bash -c "
        source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash;
        source ./install/setup.bash 2>/dev/null || true;
        echo 'Starting LSC SLAM mapping launch file...';
        ros2 launch testing lsc_slam_mapping.launch.py;
        echo 'LSC SLAM mapping stopped. Press Enter to close...';
        read
    " &
    
    echo "[INFO] LSC SLAM mapping launched in separate terminal"
    echo "[INFO] Waiting for robot chassis TF to initialize..."
    
    # Wait for robot chassis TF to be ready
    timeout=15
    count=0
    while [ $count -lt $timeout ]; do
        if check_robot_tf_running; then
            echo "[INFO] Robot chassis TF is ready!"
            break
        fi
        sleep 1
        ((count++))
        if [ $((count % 3)) -eq 0 ]; then
            echo "[INFO] Still waiting for robot TF... ($count/$timeout seconds)"
        fi
    done
    
    if [ $count -eq $timeout ]; then
        echo "[ERROR] Timeout waiting for robot chassis TF to start"
        exit 1
    fi
    
    sleep 2  # Additional wait for TF to stabilize
else
    echo "[INFO] Robot chassis TF is already running"
fi

echo

# Start SLAM Toolbox if not running
if ! check_slam_running; then
    echo "[INFO] Starting SLAM Toolbox..."
    echo "[CMD] ros2 launch slam_toolbox online_async_launch.py"
    echo
    
    # Start SLAM Toolbox in background
    gnome-terminal -- bash -c "
        source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash;
        source ./install/setup.bash 2>/dev/null || true;
        echo 'Starting SLAM Toolbox...';
        ros2 launch slam_toolbox online_async_launch.py;
        echo 'SLAM Toolbox stopped. Press Enter to close...';
        read
    " &
    
    echo "[INFO] SLAM Toolbox launched in separate terminal"
    echo "[INFO] Waiting for SLAM Toolbox to initialize..."
    
    # Wait for SLAM Toolbox to be ready
    timeout=30
    count=0
    while [ $count -lt $timeout ]; do
        if check_slam_running; then
            echo "[INFO] SLAM Toolbox is ready!"
            break
        fi
        sleep 1
        ((count++))
        if [ $((count % 5)) -eq 0 ]; then
            echo "[INFO] Still waiting... ($count/$timeout seconds)"
        fi
    done
    
    if [ $count -eq $timeout ]; then
        echo "[ERROR] Timeout waiting for SLAM Toolbox to start"
        exit 1
    fi
    
    sleep 2  # Additional wait for full initialization
else
    echo "[INFO] SLAM Toolbox is already running"
fi

echo
echo "[INFO] Configuring SLAM Toolbox for mapping..."

# Run configuration script
if [ -f "./configure_slam_simple.sh" ]; then
    ./configure_slam_simple.sh
    config_result=$?
else
    echo "[ERROR] Configuration script not found"
    exit 1
fi

if [ $config_result -eq 0 ]; then
    echo
    echo "================================================"
    echo "✅ SLAM SETUP COMPLETED SUCCESSFULLY!"
    echo "================================================"
    echo
    echo "Your SLAM system is now ready for mapping!"
    echo
    echo "Services launched:"
    echo "  ✓ LSC SLAM mapping launch file (includes robot chassis TF)"
    echo "  ✓ SLAM Toolbox (configured and active)"
    echo
    echo "Next steps:"
    echo "1. Open RViz for visualization:"
    echo "   rviz2 -d src/testing/rviz/slam_config.rviz"
    echo "2. Monitor mapping progress:"
    echo "   ros2 topic echo /map --once"
    echo "   ros2 topic echo /pose --once"
    echo "3. Optional - Optimize mapping speed:"
    echo "   ./optimize_slam_speed.sh"
    echo
    echo "The robot will now build a map as it moves around!"
    echo "================================================"
else
    echo
    echo "[ERROR] SLAM configuration failed"
    exit 1
fi
