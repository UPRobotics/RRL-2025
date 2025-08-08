#!/bin/bash

# SLAM Toolbox Speed Optimization Script
# This script adjusts parameters to make mapping faster and more responsive

echo "=============================================="
echo "SLAM Toolbox Speed Optimization Script"
echo "=============================================="
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

# Check if SLAM Toolbox is running and active
print_info "Checking SLAM Toolbox status..."

if ! ros2 lifecycle get /slam_toolbox &>/dev/null; then
    print_error "SLAM Toolbox not found! Please start it first."
    exit 1
fi

STATE=$(ros2 lifecycle get /slam_toolbox | awk '{print $1}')
if [ "$STATE" != "active" ]; then
    print_error "SLAM Toolbox is not active (current state: $STATE)"
    print_info "Please activate it first using: ./configure_slam_simple.sh"
    exit 1
fi

print_success "SLAM Toolbox is active and ready for optimization"
echo

# Show current parameters
print_info "Current SLAM parameters:"
echo "  minimum_time_interval: $(ros2 param get /slam_toolbox minimum_time_interval 2>/dev/null | grep -o '[0-9]*\.[0-9]*')s"
echo "  minimum_travel_distance: $(ros2 param get /slam_toolbox minimum_travel_distance 2>/dev/null | grep -o '[0-9]*\.[0-9]*')m"
echo "  minimum_travel_heading: $(ros2 param get /slam_toolbox minimum_travel_heading 2>/dev/null | grep -o '[0-9]*\.[0-9]*')rad"
echo "  scan_queue_size: $(ros2 param get /slam_toolbox scan_queue_size 2>/dev/null | grep -o '[0-9]*')"
echo

# Ask user for optimization level
echo "Choose optimization level:"
echo "  1) Conservative (2x faster) - Good balance of speed and stability"
echo "  2) Aggressive (4x faster) - Maximum speed, may be less stable"
echo "  3) Custom - Set your own values"
echo "  4) Reset to defaults"
echo

read -p "Enter your choice (1-4): " choice

case $choice in
    1)
        print_info "Applying conservative speed optimization..."
        NEW_TIME_INTERVAL=0.25
        NEW_TRAVEL_DISTANCE=0.05
        NEW_TRAVEL_HEADING=0.05
        NEW_QUEUE_SIZE=2
        ;;
    2)
        print_info "Applying aggressive speed optimization..."
        NEW_TIME_INTERVAL=0.1
        NEW_TRAVEL_DISTANCE=0.02
        NEW_TRAVEL_HEADING=0.02
        NEW_QUEUE_SIZE=3
        ;;
    3)
        print_info "Custom configuration:"
        read -p "Minimum time interval (default 0.5s, faster: 0.1s): " NEW_TIME_INTERVAL
        read -p "Minimum travel distance (default 0.1m, faster: 0.02m): " NEW_TRAVEL_DISTANCE
        read -p "Minimum travel heading (default 0.1rad, faster: 0.02rad): " NEW_TRAVEL_HEADING
        read -p "Scan queue size (default 1, faster: 2-3): " NEW_QUEUE_SIZE
        
        # Set defaults if empty
        NEW_TIME_INTERVAL=${NEW_TIME_INTERVAL:-0.25}
        NEW_TRAVEL_DISTANCE=${NEW_TRAVEL_DISTANCE:-0.05}
        NEW_TRAVEL_HEADING=${NEW_TRAVEL_HEADING:-0.05}
        NEW_QUEUE_SIZE=${NEW_QUEUE_SIZE:-2}
        ;;
    4)
        print_info "Resetting to default values..."
        NEW_TIME_INTERVAL=0.5
        NEW_TRAVEL_DISTANCE=0.1
        NEW_TRAVEL_HEADING=0.1
        NEW_QUEUE_SIZE=1
        ;;
    *)
        print_error "Invalid choice. Exiting."
        exit 1
        ;;
esac

echo
print_info "Applying new parameters..."

# Apply the parameters
print_info "Setting minimum_time_interval to ${NEW_TIME_INTERVAL}s..."
if ros2 param set /slam_toolbox minimum_time_interval $NEW_TIME_INTERVAL &>/dev/null; then
    print_success "✓ minimum_time_interval updated"
else
    print_error "✗ Failed to update minimum_time_interval"
fi

print_info "Setting minimum_travel_distance to ${NEW_TRAVEL_DISTANCE}m..."
if ros2 param set /slam_toolbox minimum_travel_distance $NEW_TRAVEL_DISTANCE &>/dev/null; then
    print_success "✓ minimum_travel_distance updated"
else
    print_error "✗ Failed to update minimum_travel_distance"
fi

print_info "Setting minimum_travel_heading to ${NEW_TRAVEL_HEADING}rad..."
if ros2 param set /slam_toolbox minimum_travel_heading $NEW_TRAVEL_HEADING &>/dev/null; then
    print_success "✓ minimum_travel_heading updated"
else
    print_error "✗ Failed to update minimum_travel_heading"
fi

print_info "Setting scan_queue_size to ${NEW_QUEUE_SIZE}..."
if ros2 param set /slam_toolbox scan_queue_size $NEW_QUEUE_SIZE &>/dev/null; then
    print_success "✓ scan_queue_size updated"
else
    print_error "✗ Failed to update scan_queue_size"
fi

echo
print_success "SLAM Toolbox optimization completed!"
echo

# Show final parameters
print_info "Updated SLAM parameters:"
echo "  minimum_time_interval: $(ros2 param get /slam_toolbox minimum_time_interval 2>/dev/null | grep -o '[0-9]*\.[0-9]*')s"
echo "  minimum_travel_distance: $(ros2 param get /slam_toolbox minimum_travel_distance 2>/dev/null | grep -o '[0-9]*\.[0-9]*')m"
echo "  minimum_travel_heading: $(ros2 param get /slam_toolbox minimum_travel_heading 2>/dev/null | grep -o '[0-9]*\.[0-9]*')rad"
echo "  scan_queue_size: $(ros2 param get /slam_toolbox scan_queue_size 2>/dev/null | grep -o '[0-9]*')"
echo

# Test the mapping frequency
print_info "Testing map update frequency..."
print_info "Monitoring /map topic for 10 seconds..."

# Monitor map topic frequency
timeout 10s ros2 topic hz /map &
MONITOR_PID=$!

sleep 11

# Check if we can get better frequency info
print_info "You can monitor the new mapping speed with:"
echo "  ros2 topic hz /map"
echo "  ros2 topic hz /pose"
echo

print_success "🚀 SLAM mapping should now be faster and more responsive!"
echo
print_info "Tips for optimal performance:"
echo "  • Move the robot smoothly for best results"
echo "  • Monitor CPU usage - aggressive settings may increase load"
echo "  • Adjust settings if you notice map quality issues"
echo "  • Use 'rviz2 -d src/testing/rviz/slam_config.rviz' to visualize"
echo
print_warning "Note: If you experience instability, run this script again and choose option 4 (reset to defaults)"
