#!/bin/bash

# SLAM Toolbox Manual Configuration Script
# This script manually configures and activates the SLAM Toolbox mapping node
# Author: Auto-generated for ROS2 SLAM Toolbox

set -e  # Exit on any error

echo "=========================================="
echo "SLAM Toolbox Manual Configuration Script"
echo "=========================================="
echo

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Function to check if a service exists
check_service() {
    if ros2 service list | grep -q "$1"; then
        return 0
    else
        return 1
    fi
}

# Function to get current SLAM Toolbox state
get_slam_state() {
    local output=$(ros2 lifecycle get /slam_toolbox 2>/dev/null)
    local state=$(echo "$output" | awk '{print $1}')
    echo "$state"
}

# Function to change SLAM Toolbox state
change_slam_state() {
    local transition_name=$1
    
    print_step "Transitioning SLAM Toolbox to: $transition_name"
    
    local result=$(ros2 lifecycle set /slam_toolbox $transition_name 2>&1)
    
    if echo "$result" | grep -q "Transitioning successful"; then
        print_status "Successfully transitioned to: $transition_name"
        return 0
    else
        print_error "Failed to transition to: $transition_name"
        print_error "Error: $result"
        return 1
    fi
}

# Function to wait for state change
wait_for_state() {
    local target_state=$1
    local timeout=15
    local count=0
    
    print_step "Waiting for state to become: $target_state"
    
    while [ $count -lt $timeout ]; do
        local current_state=$(get_slam_state)
        if [ "$current_state" = "$target_state" ]; then
            print_status "State successfully changed to: $target_state"
            return 0
        fi
        sleep 1
        ((count++))
        if [ $((count % 3)) -eq 0 ]; then
            print_step "Still waiting for $target_state... ($count/$timeout seconds)"
        fi
    done
    print_warning "Timeout waiting for state: $target_state"
    return 1
}

# Main configuration process
main() {
    print_step "Checking ROS2 environment..."
    
    # Check if ROS2 is sourced
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please source your ROS2 installation."
        exit 1
    fi
    
    print_status "ROS2 environment detected"
    
    # Check if SLAM Toolbox node is running
    print_step "Checking SLAM Toolbox node..."
    
    if ! ros2 node list 2>/dev/null | grep -q "/slam_toolbox"; then
        print_error "SLAM Toolbox node not found. Please start the SLAM Toolbox node first:"
        echo "  ros2 launch slam_toolbox online_async_launch.py"
        exit 1
    fi
    
    print_status "SLAM Toolbox node detected"
    
    # Get current state
    current_state=$(get_slam_state)
    print_status "Current SLAM Toolbox state: $current_state"
    
    # Configure based on current state
    case "$current_state" in
        "unconfigured")
            print_step "Configuring SLAM Toolbox..."
            
            # Transition to configured state
            if change_slam_state "configure"; then
                if wait_for_state "inactive"; then
                    print_status "SLAM Toolbox successfully configured"
                else
                    print_warning "Configuration may still be in progress"
                fi
            else
                print_error "Failed to configure SLAM Toolbox"
                exit 1
            fi
            
            # Add delay before activation
            sleep 3
            
            # Transition to active state
            print_step "Activating SLAM Toolbox..."
            if change_slam_state "activate"; then
                if wait_for_state "active"; then
                    print_status "SLAM Toolbox successfully activated"
                else
                    print_warning "Activation may still be in progress"
                fi
            else
                print_error "Failed to activate SLAM Toolbox"
                exit 1
            fi
            ;;
            
        "inactive")
            print_step "SLAM Toolbox already configured, activating..."
            
            # Transition to active state
            if change_slam_state "activate"; then
                if wait_for_state "active"; then
                    print_status "SLAM Toolbox successfully activated"
                else
                    print_warning "Activation may still be in progress"
                fi
            else
                print_error "Failed to activate SLAM Toolbox"
                exit 1
            fi
            ;;
            
        "active")
            print_status "SLAM Toolbox is already active and mapping!"
            ;;
            
        *)
            print_warning "SLAM Toolbox in unexpected state: $current_state"
            print_step "Attempting to reset and reconfigure..."
            
            # Try to cleanup and reconfigure
            if change_slam_state "cleanup"; then
                sleep 3
                if change_slam_state "configure"; then
                    sleep 3
                    change_slam_state "activate"
                fi
            fi
            ;;
    esac
    
    # Final state check
    final_state=$(get_slam_state)
    print_status "Final SLAM Toolbox state: $final_state"
    
    if [ "$final_state" = "active" ]; then
        echo
        print_status "✅ SLAM Toolbox is now active and mapping!"
        echo
        print_step "Verifying mapping functionality..."
        
        # Check if map topic is being published
        if ros2 topic list | grep -q "/map"; then
            print_status "✓ Map topic is available"
        else
            print_warning "✗ Map topic not found"
        fi
        
        # Check if pose topic is being published
        if ros2 topic list | grep -q "/pose"; then
            print_status "✓ Pose topic is available"
        else
            print_warning "✗ Pose topic not found"
        fi
        
        # Check map updates
        if ros2 topic list | grep -q "/map_updates"; then
            print_status "✓ Map updates topic is available"
        else
            print_warning "✗ Map updates topic not found"
        fi
        
        echo
        print_status "SLAM mapping configuration completed successfully!"
        echo
        echo "Available visualization tools:"
        echo "  - rviz2 -d $(pwd)/src/testing/rviz/slam_config.rviz"
        echo "  - ros2 topic echo /map --once"
        echo "  - ros2 topic echo /pose --once"
        echo
        
    else
        print_error "Failed to activate SLAM Toolbox. Current state: $final_state"
        exit 1
    fi
}

# Handle script interruption
trap 'echo; print_warning "Script interrupted by user"; exit 130' INT

# Run main function
main "$@"
