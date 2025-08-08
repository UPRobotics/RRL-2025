#!/bin/bash

# LSC SLAM Complete Setup Script
# This script launches the complete LSC SLAM mapping setup and configures SLAM Toolbox

echo "=================================================="
echo "LSC SLAM Complete Mapping Setup"
echo "=================================================="
echo

# Color codes for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Source ROS2 environment
print_step "Setting up ROS2 environment..."

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    print_success "Sourced ROS2 Jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    print_success "Sourced ROS2 Humble"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    print_success "Sourced ROS2 Galactic"
else
    print_error "ROS2 installation not found!"
    exit 1
fi

# Source workspace
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
    print_success "Sourced workspace overlay"
else
    print_warning "Workspace overlay not found, continuing without it"
fi

echo

# Function to check if LSC nodes are running
check_lsc_nodes() {
    if ros2 node list 2>/dev/null | grep -q "autonics_lsc_lidar"; then
        return 0
    else
        return 1
    fi
}

# Function to check if SLAM Toolbox is running
check_slam_running() {
    if ros2 service list 2>/dev/null | grep -q "/slam_toolbox/get_state"; then
        return 0
    else
        return 1
    fi
}

# Function to check if lidar is publishing
check_lidar_running() {
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        return 0
    else
        return 1
    fi
}

# Step 1: Launch LSC SLAM mapping (includes lidar driver, transforms, robot model)
print_step "Step 1: Launching LSC SLAM mapping setup..."

if ! check_lsc_nodes; then
    print_info "Starting LSC SLAM mapping launch file..."
    print_info "This includes: Lidar driver, robot transforms, and 3D robot model"
    echo
    
    # Start LSC SLAM mapping in background terminal
    gnome-terminal --title="LSC SLAM Mapping" -- bash -c "
        echo '=== LSC SLAM Mapping Terminal ===';
        echo 'Loading ROS2 environment...';
        source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash;
        source ./install/setup.bash 2>/dev/null || true;
        echo 'Starting LSC SLAM mapping launch file...';
        echo 'This will start: Lidar driver + Robot transforms + 3D model';
        echo;
        ros2 launch testing lsc_slam_mapping.launch.py;
        echo;
        echo 'LSC SLAM mapping stopped. Press Enter to close this terminal...';
        read
    " &
    
    print_success "LSC SLAM mapping launched in separate terminal"
    print_info "Waiting for LSC components to initialize..."
    
    # Wait for LSC components to be ready
    timeout=30
    count=0
    while [ $count -lt $timeout ]; do
        if check_lsc_nodes; then
            print_success "Robot chassis TF is ready!"
            break
        fi
        sleep 1
        ((count++))
        if [ $((count % 5)) -eq 0 ]; then
            print_info "Still waiting for LSC components... ($count/$timeout seconds)"
        fi
    done
    
    if [ $count -eq $timeout ]; then
        print_error "Timeout waiting for LSC components to start"
        print_error "Please check the LSC SLAM mapping terminal for errors"
        exit 1
    fi
    
    # Wait for lidar to be ready
    print_info "Waiting for lidar to start publishing..."
    timeout=20
    count=0
    while [ $count -lt $timeout ]; do
        if check_lidar_running; then
            print_success "Lidar is publishing scan data!"
            break
        fi
        sleep 1
        ((count++))
        if [ $((count % 3)) -eq 0 ]; then
            print_info "Still waiting for lidar... ($count/$timeout seconds)"
        fi
    done
    
    if [ $count -eq $timeout ]; then
        print_warning "Lidar may not be ready yet, continuing anyway..."
    fi
    
    sleep 3  # Additional wait for everything to stabilize
else
    print_success "LSC components are already running"
fi

echo

# Step 2: Launch SLAM Toolbox
print_step "Step 2: Launching SLAM Toolbox..."

if ! check_slam_running; then
    print_info "Starting SLAM Toolbox..."
    echo
    
    # Start SLAM Toolbox in background terminal
    gnome-terminal --title="SLAM Toolbox" -- bash -c "
        echo '=== SLAM Toolbox Terminal ===';
        echo 'Loading ROS2 environment...';
        source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash;
        source ./install/setup.bash 2>/dev/null || true;
        echo 'Starting SLAM Toolbox...';
        echo;
        ros2 launch slam_toolbox online_async_launch.py;
        echo;
        echo 'SLAM Toolbox stopped. Press Enter to close this terminal...';
        read
    " &
    
    print_success "SLAM Toolbox launched in separate terminal"
    print_info "Waiting for SLAM Toolbox to initialize..."
    
    # Wait for SLAM Toolbox to be ready
    timeout=30
    count=0
    while [ $count -lt $timeout ]; do
        if check_slam_running; then
            print_success "SLAM Toolbox is ready!"
            break
        fi
        sleep 1
        ((count++))
        if [ $((count % 5)) -eq 0 ]; then
            print_info "Still waiting for SLAM Toolbox... ($count/$timeout seconds)"
        fi
    done
    
    if [ $count -eq $timeout ]; then
        print_error "Timeout waiting for SLAM Toolbox to start"
        print_error "Please check the SLAM Toolbox terminal for errors"
        exit 1
    fi
    
    sleep 3  # Additional wait for full initialization
else
    print_success "SLAM Toolbox is already running"
fi

echo

# Step 3: Configure SLAM Toolbox for mapping
print_step "Step 3: Configuring SLAM Toolbox for mapping..."

# Check if configuration script exists
if [ -f "./configure_slam_simple.sh" ]; then
    print_info "Running SLAM configuration script..."
    ./configure_slam_simple.sh
    config_result=$?
else
    print_warning "Configuration script not found, attempting manual configuration..."
    
    # Manual configuration as fallback
    print_info "Manually configuring SLAM Toolbox..."
    
    # Get current state
    current_state=$(ros2 lifecycle get /slam_toolbox 2>/dev/null | awk '{print $1}')
    print_info "Current SLAM state: $current_state"
    
    # Configure if needed
    if [ "$current_state" = "unconfigured" ]; then
        print_info "Configuring SLAM Toolbox..."
        if ros2 lifecycle set /slam_toolbox configure 2>/dev/null; then
            print_success "✓ Configuration successful"
            sleep 2
        else
            print_error "✗ Configuration failed"
            exit 1
        fi
    fi
    
    # Activate
    print_info "Activating SLAM Toolbox..."
    if ros2 lifecycle set /slam_toolbox activate 2>/dev/null; then
        print_success "✓ Activation successful"
        config_result=0
    else
        print_error "✗ Activation failed"
        config_result=1
    fi
fi

echo

# Step 4: Verify setup and show results
if [ $config_result -eq 0 ]; then
    print_step "Step 4: Verifying complete setup..."
    
    # Verify components are running
    echo
    print_info "System Status Check:"
    
    if check_lsc_nodes; then
        print_success "✓ LSC robot components (TF, model, lidar driver)"
    else
        print_warning "✗ LSC robot components may not be fully ready"
    fi
    
    if check_lidar_running; then
        print_success "✓ Lidar publishing scan data"
    else
        print_warning "✗ Lidar scan data not detected"
    fi
    
    if check_slam_running; then
        print_success "✓ SLAM Toolbox active and ready"
    else
        print_error "✗ SLAM Toolbox not ready"
    fi
    
    # Check if mapping topics are available
    if ros2 topic list 2>/dev/null | grep -q "/map"; then
        print_success "✓ Map topic available"
    else
        print_warning "✗ Map topic not found"
    fi
    
    if ros2 topic list 2>/dev/null | grep -q "/pose"; then
        print_success "✓ Pose topic available"
    else
        print_warning "✗ Pose topic not found"
    fi
    
    echo
    echo "=================================================="
    echo "✅ LSC SLAM MAPPING SETUP COMPLETED!"
    echo "=================================================="
    echo
    print_success "Your complete SLAM mapping system is now ready!"
    echo
    print_info "Components launched:"
    echo "  🚀 LSC SLAM mapping (lidar driver + robot transforms + 3D model)"
    echo "  🗺️  SLAM Toolbox (configured and actively mapping)"
    echo
    print_info "Next steps:"
    echo "  1. Open RViz for visualization:"
    echo "     rviz2 -d src/testing/rviz/slam_config.rviz"
    echo
    echo "  2. Monitor mapping in real-time:"
    echo "     ros2 topic echo /map --once"
    echo "     ros2 topic echo /pose --once"
    echo "     ros2 topic hz /scan"
    echo
    echo "  3. Optional - Optimize mapping speed:"
    echo "     ./optimize_slam_speed.sh"
    echo
    print_success "🎉 Start moving your robot to begin mapping!"
    echo "=================================================="
    
else
    print_error "SLAM configuration failed"
    print_info "Please check the terminal outputs for error messages"
    exit 1
fi
