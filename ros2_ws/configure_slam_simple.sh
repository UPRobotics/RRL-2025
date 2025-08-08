#!/bin/bash

# Simple SLAM Toolbox Configuration Script
# This script manually configures the SLAM Toolbox mapping node

echo "=========================================="
echo "Simple SLAM Toolbox Configuration Script"
echo "=========================================="
echo

# Check if SLAM Toolbox is running
if ! ros2 service list | grep -q "/slam_toolbox/get_state"; then
    echo "[ERROR] SLAM Toolbox not running. Please start it first:"
    echo "  ros2 launch slam_toolbox online_async_launch.py"
    exit 1
fi

echo "[INFO] SLAM Toolbox node detected"

# Try to activate SLAM Toolbox directly
echo "[INFO] Attempting to activate SLAM Toolbox..."

# Get current state
CURRENT_STATE=$(ros2 lifecycle get /slam_toolbox 2>/dev/null | awk '{print $1}')
echo "[INFO] Current state: $CURRENT_STATE"

# Configure first if unconfigured
if [ "$CURRENT_STATE" = "unconfigured" ]; then
    echo "[INFO] Configuring SLAM Toolbox..."
    CONFIGURE_RESULT=$(ros2 lifecycle set /slam_toolbox configure 2>&1)
    
    if echo "$CONFIGURE_RESULT" | grep -q "Transitioning successful"; then
        echo "[SUCCESS] ✓ Configuration successful"
        sleep 3  # Wait for configuration to complete
    else
        echo "[ERROR] Configuration failed: $CONFIGURE_RESULT"
        echo "[INFO] Trying Python configurator as fallback..."
        python3 ./configure_slam_mapping.py
        exit $?
    fi
fi

# Activate (transition ID 3)
echo "[INFO] Activating SLAM Toolbox..."
ACTIVATE_RESULT=$(ros2 lifecycle set /slam_toolbox activate 2>&1)

if echo "$ACTIVATE_RESULT" | grep -q "Transitioning successful"; then
    echo "[SUCCESS] ✅ SLAM Toolbox activated successfully!"
    
    # Wait a moment for activation to complete
    sleep 2
    
    # Verify final state
    FINAL_STATE=$(ros2 lifecycle get /slam_toolbox 2>/dev/null | awk '{print $1}')
    echo "[INFO] Final state: $FINAL_STATE"
    
    if [ "$FINAL_STATE" = "active" ]; then
        # Verify mapping topics
        echo
        echo "[INFO] Verifying mapping functionality..."
        
        if ros2 topic list | grep -q "/map"; then
            echo "[SUCCESS] ✓ Map topic is available"
        fi
        
        if ros2 topic list | grep -q "/pose"; then
            echo "[SUCCESS] ✓ Pose topic is available"
        fi
        
        if ros2 topic list | grep -q "/map_metadata"; then
            echo "[SUCCESS] ✓ Map metadata topic is available"
        fi
        
        echo
        echo "🎉 SLAM Toolbox is now configured and actively mapping!"
        echo
        echo "You can now:"
        echo "  • View the map: ros2 topic echo /map --once"
        echo "  • Check robot pose: ros2 topic echo /pose --once"
        echo "  • Open RViz: rviz2 -d src/testing/rviz/slam_config.rviz"
        echo
    else
        echo "[WARNING] SLAM may still be transitioning. Current state: $FINAL_STATE"
    fi
    
else
    echo "[ERROR] Activation failed: $ACTIVATE_RESULT"
    echo "[INFO] Trying Python configurator as fallback..."
    python3 ./configure_slam_mapping.py
fi
