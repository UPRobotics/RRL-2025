#!/bin/bash

echo "Starting SLAM Toolbox auto-configuration..."

# The recommended solution is to set autostart: true in your SLAM Toolbox configuration
# This way the node will automatically configure and activate on startup
echo "NOTE: The best solution is to set 'autostart: true' in your SLAM Toolbox config file"
echo "This script provides manual lifecycle management as a workaround"

# Wait for SLAM Toolbox node to be available
echo "Waiting for SLAM Toolbox node..."
timeout=30
counter=0
while ! ros2 node list | grep -q "/slam_toolbox" && [ $counter -lt $timeout ]; do
    sleep 1
    counter=$((counter + 1))
    echo "  Waiting... ($counter/$timeout)"
done

if [ $counter -ge $timeout ]; then
    echo "ERROR: SLAM Toolbox node not found after $timeout seconds!"
    exit 1
fi

echo "SLAM Toolbox node found! Current state:"
current_state=$(ros2 lifecycle get /slam_toolbox)
echo "$current_state"

# Check if already active
if echo "$current_state" | grep -q "active"; then
    echo "✓ SLAM Toolbox is already active!"
    exit 0
fi

# Configure SLAM Toolbox if needed
if echo "$current_state" | grep -q "unconfigured"; then
    echo "Configuring SLAM Toolbox..."
    if ros2 lifecycle set /slam_toolbox configure; then
        echo "✓ Configuration successful"
    else
        echo "✗ Configuration failed"
        exit 1
    fi
    sleep 2
fi

# Activate SLAM Toolbox
echo "Activating SLAM Toolbox..."
if ros2 lifecycle set /slam_toolbox activate; then
    echo "✓ Activation successful"
    echo "✓ SLAM Toolbox should now be active and mapping!"
    
    # Verify final state
    echo "Final state:"
    ros2 lifecycle get /slam_toolbox
else
    echo "✗ Activation failed"
    exit 1
fi
