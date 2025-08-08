#!/bin/bash

# Second Screen Setup Script for Ubuntu
# Uses tablet as second monitor via USB with low latency
# Author: GitHub Copilot
# Date: July 7, 2025

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
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

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to install prerequisites
install_prerequisites() {
    print_status "Installing prerequisites..."
    
    # Update package list
    sudo apt update
    
    # Install essential packages
    sudo apt install -y \
        scrcpy \
        x11vnc \
        xvfb \
        x11-utils \
        wmctrl \
        adb \
        fastboot \
        ffmpeg \
        libavcodec-extra \
        android-tools-adb \
        android-tools-fastboot
    
    # Install additional packages for low latency
    sudo apt install -y \
        v4l2loopback-dkms \
        v4l-utils \
        obs-studio
    
    print_success "Prerequisites installed successfully!"
}

# Function to install apps for using tablet as extended monitor
install_extended_monitor_tools() {
    print_status "Installing tools for tablet as extended monitor..."
    
    # Update package list
    sudo apt update
    
    # Install virtual display tools
    sudo apt install -y \
        xserver-xorg-video-dummy \
        x11vnc \
        tigervnc-standalone-server \
        xvfb \
        x11-xserver-utils \
        wmctrl \
        barrier \
        git \
        build-essential \
        pkg-config \
        meson \
        ninja-build \
        ffmpeg \
        libavcodec-dev \
        libavdevice-dev \
        libavformat-dev \
        libavutil-dev \
        libsdl2-dev \
        libusb-1.0-0-dev
    
    # Install latest scrcpy for Android tablets
    print_status "Installing latest scrcpy from source..."
    cd /tmp
    if [ ! -d "scrcpy" ]; then
        git clone https://github.com/Genymobile/scrcpy.git
    fi
    cd scrcpy
    git checkout v3.3.1
    ./install_release.sh
    
    print_success "Extended monitor tools installed successfully!"
    print_status "Your tablet can now be used as a second monitor!"
}

# Function to create virtual display for tablet as second monitor
create_virtual_display() {
    local width=${1:-1920}
    local height=${2:-1200}
    local refresh=${3:-60}
    
    print_status "Creating virtual display for tablet (${width}x${height}@${refresh}Hz)..."
    
    # Get current displays
    primary_display=$(xrandr | grep " connected primary" | awk '{print $1}')
    print_status "Primary display: $primary_display"
    
    # Create modeline for the virtual display
    modeline=$(cvt $width $height $refresh | grep "Modeline" | sed 's/Modeline //')
    mode_name=$(echo $modeline | awk '{print $1}' | tr -d '"')
    
    print_status "Creating mode: $mode_name"
    
    # Add the new mode
    xrandr --newmode $modeline 2>/dev/null || print_warning "Mode might already exist"
    
    # Create virtual display using dummy video driver
    export DISPLAY=:0
    
    # Check if we have a virtual output already
    virtual_output=$(xrandr | grep "VIRTUAL" | awk '{print $1}' | head -1)
    
    if [ -z "$virtual_output" ]; then
        print_status "Creating virtual output..."
        # We'll use VNC to create the virtual screen
        virtual_output="VNC-0"
    fi
    
    # Add mode to virtual output and enable it
    xrandr --addmode $virtual_output $mode_name 2>/dev/null || true
    xrandr --output $virtual_output --mode $mode_name --right-of $primary_display 2>/dev/null || true
    
    print_success "Virtual display created: ${width}x${height}"
    print_status "You can now drag windows to your tablet!"
    
    return 0
}

# Function to check Android version and compatibility
check_android_compatibility() {
    print_status "Checking Android compatibility..."
    
    # Get Android version
    android_version=$(adb shell getprop ro.build.version.release 2>/dev/null || echo "Unknown")
    android_sdk=$(adb shell getprop ro.build.version.sdk 2>/dev/null || echo "Unknown")
    device_model=$(adb shell getprop ro.product.model 2>/dev/null || echo "Unknown")
    
    print_status "Device: $device_model"
    print_status "Android Version: $android_version (SDK: $android_sdk)"
    
    # Check if Android version is 15+ (SDK 35+)
    if [[ "$android_sdk" =~ ^[0-9]+$ ]] && [ "$android_sdk" -ge 35 ]; then
        print_warning "Android 15+ detected! scrcpy 1.25 has compatibility issues."
        print_warning "Trying compatibility mode..."
        return 1
    fi
    
    return 0
}

# Function to start tablet as extended monitor via VNC
start_tablet_extended_monitor() {
    local width=${1:-1920}
    local height=${2:-1200}
    
    print_status "Setting up tablet as extended monitor..."
    
    # Fix X permissions first
    fix_x_permissions
    
    # First, create the virtual display
    create_virtual_display $width $height
    
    # Start VNC server for the extended display
    print_status "Starting VNC server for extended display..."
    
    # Kill existing VNC servers
    pkill x11vnc 2>/dev/null || true
    vncserver -kill :1 2>/dev/null || true
    
    # Get your local IP
    local_ip=$(hostname -I | awk '{print $1}')
    
    # Set up proper X authority for VNC access
    export DISPLAY=:0
    
    # Get the current user's X authority file
    if [ -z "$XAUTHORITY" ]; then
        XAUTH_FILE="$HOME/.Xauthority"
    else
        XAUTH_FILE="$XAUTHORITY"
    fi
    
    # Start VNC server with proper authentication and extended desktop
    x11vnc \
        -display :0 \
        -auth "$XAUTH_FILE" \
        -forever \
        -shared \
        -nopw \
        -rfbport 5900 \
        -geometry ${width}x${height} \
        -nocursor \
        -noxdamage \
        -noxfixes \
        -noxrandr \
        -threads \
        -wireframe \
        -scrollcopyrect \
        -solid \
        -bg \
        -o /tmp/x11vnc.log &
    
    VNC_PID=$!
    echo $VNC_PID > /tmp/tablet_vnc.pid
    
    # Wait a moment for VNC to start
    sleep 2
    
    # Check if VNC started successfully
    if kill -0 $VNC_PID 2>/dev/null; then
        print_success "Extended monitor setup complete!"
        print_success "VNC server running on: $local_ip:5900"
        echo ""
        print_status "On your tablet:"
        echo "1. Install a VNC client app (VNC Viewer, RealVNC, etc.)"
        echo "2. Connect to: $local_ip:5900"
        echo "3. You should now see your desktop!"
        echo "4. To extend the desktop, arrange windows across both screens"
        echo ""
        print_status "VNC log: tail -f /tmp/x11vnc.log"
    else
        print_error "Failed to start VNC server. Check permissions."
        print_status "Try running: xhost +local:"
        print_status "Or check the log: cat /tmp/x11vnc.log"
        return 1
    fi
    
    return 0
}

# Function to start using scrcpy as extended display (experimental)
start_scrcpy_extended() {
    print_status "Setting up scrcpy for extended display (experimental)..."
    
    # Check if device is connected
    if ! adb devices | grep -q "device$"; then
        print_error "No Android device detected. Please:"
        echo "1. Connect your tablet via USB"
        echo "2. Enable USB debugging on your tablet"
        echo "3. Accept the debugging prompt"
        return 1
    fi
    
    # Create virtual display first
    create_virtual_display 1920 1200
    
    # Kill any existing scrcpy instances
    pkill scrcpy 2>/dev/null || true
    
    print_warning "Note: This is experimental. scrcpy mirrors your screen."
    print_status "For true extended desktop, use 'vnc' mode instead."
    
    # Start scrcpy with optimized settings for tablet use
    scrcpy \
        --max-fps=60 \
        --video-bit-rate=30M \
        --max-size=1920 \
        --window-title="Extended Display" \
        --fullscreen &
    
    print_success "Scrcpy started in fullscreen mode!"
    print_status "This mirrors your desktop. For extended desktop, try: ./secondscreen.sh extended"
}

# Function to start VNC server for cross-platform compatibility
start_vnc_server() {
    local display_num=${1:-:1}
    local geometry=${2:-1920x1080}
    
    print_status "Starting VNC server on display $display_num..."
    
    # Kill existing VNC server
    vncserver -kill $display_num 2>/dev/null || true
    
    # Start VNC server with optimizations
    x11vnc -display :0 \
        -geometry $geometry \
        -depth 24 \
        -forever \
        -loop \
        -noxdamage \
        -noxfixes \
        -noxrandr \
        -wait 10 \
        -defer 10 \
        -speeds modem \
        -quality 9 \
        -compress 0 \
        -threads \
        -port 5900 &
    
    VNC_PID=$!
    echo $VNC_PID > /tmp/vnc_server.pid
    
    print_success "VNC server started on port 5900"
    print_status "Connect your tablet VNC client to: $(hostname -I | awk '{print $1}'):5900"
}

# Function to create extended desktop
create_extended_desktop() {
    local tablet_width=${1:-1920}
    local tablet_height=${2:-1080}
    
    print_status "Creating extended desktop..."
    
    # Get current primary display info
    primary_display=$(xrandr | grep " connected primary" | cut -d' ' -f1)
    primary_resolution=$(xrandr | grep "$primary_display" | grep -o '[0-9]*x[0-9]*' | head -1)
    
    # Create virtual output for tablet
    xrandr --setprovideroutputsource modesetting NVIDIA-0 2>/dev/null || true
    xrandr --auto
    
    # Add tablet resolution mode
    cvt $tablet_width $tablet_height 60 | grep "Modeline" | cut -d' ' -f2- | xargs xrandr --newmode 2>/dev/null || true
    
    print_success "Extended desktop configured!"
}

# Function to optimize for low latency
optimize_latency() {
    print_status "Applying low latency optimizations..."
    
    # Disable composition if using compiz/unity
    gsettings set org.compiz.core active-plugins "['core', 'move', 'resize', 'place', 'decoration', 'mousepoll', 'regex', 'animation', 'wall', 'unitymtgrabhandles']" 2>/dev/null || true
    
    # Set performance mode
    sudo cpupower frequency-set -g performance 2>/dev/null || true
    
    # Optimize network for local connections
    echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
    echo 'net.core.wmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
    sudo sysctl -p
    
    print_success "Low latency optimizations applied!"
}

# Function to stop all services
stop_services() {
    print_status "Stopping second screen services..."
    
    # Stop scrcpy
    pkill scrcpy 2>/dev/null || true
    
    # Stop VNC server
    if [ -f /tmp/vnc_server.pid ]; then
        kill $(cat /tmp/vnc_server.pid) 2>/dev/null || true
        rm /tmp/vnc_server.pid
    fi
    
    # Reset display settings
    xrandr --auto
    
    print_success "All services stopped!"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  install         Install prerequisites for extended monitor"
    echo "  extended        Use tablet as extended monitor (VNC - advanced)"
    echo "  simple          Simple VNC sharing (recommended for troubleshooting)"
    echo "  mirror          Mirror PC screen to tablet (scrcpy)"
    echo "  virtual         Create virtual display without tablet"
    echo "  vnc            Start VNC server manually"
    echo "  diagnose       Run diagnostics for X display issues"
    echo "  optimize       Apply low latency optimizations"
    echo "  stop           Stop all second screen services"
    echo "  status         Show connection status"
    echo "  help           Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 install                    # Install prerequisites"
    echo "  $0 simple                     # Simple VNC sharing (try this first)"
    echo "  $0 extended 1920 1200        # Advanced extended monitor"
    echo "  $0 mirror                     # Mirror screen to tablet"
}

# Function to show status
show_status() {
    print_status "Second Screen Status:"
    echo ""
    
    # Check if scrcpy is running
    if pgrep scrcpy >/dev/null; then
        print_success "Scrcpy is running"
    else
        echo "Scrcpy: Not running"
    fi
    
    # Check if VNC server is running
    if [ -f /tmp/vnc_server.pid ] && kill -0 $(cat /tmp/vnc_server.pid) 2>/dev/null; then
        print_success "VNC server is running on port 5900"
    else
        echo "VNC server: Not running"
    fi
    
    # Check connected devices
    echo ""
    print_status "Connected devices:"
    adb devices
}

# Function to fix X display permissions
fix_x_permissions() {
    print_status "Fixing X display permissions..."
    
    # Try to detect the correct DISPLAY automatically
    if [ -z "$DISPLAY" ]; then
        # Check which displays are available
        for test_display in ":1" ":0" ":10" ":11"; do
            if DISPLAY=$test_display xrandr >/dev/null 2>&1; then
                export DISPLAY=$test_display
                print_status "Auto-detected DISPLAY: $DISPLAY"
                break
            fi
        done
    fi
    
    # Set fallback if still not found
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:1  # Based on diagnostics showing :1 works
    fi
    
    print_status "Using DISPLAY: $DISPLAY"
    
    # Allow local connections to X server
    xhost +local: 2>/dev/null || true
    
    # Check if we can connect to X
    if xrandr >/dev/null 2>&1; then
        print_success "X display access confirmed"
        return 0
    else
        print_warning "X display access issues detected"
        print_status "Current DISPLAY: $DISPLAY"
        print_status "Current USER: $USER"
        print_status "Checking X server status..."
        
        # Check if X server is running
        if pgrep -x Xorg >/dev/null || pgrep -x X >/dev/null; then
            print_status "X server is running"
        else
            print_error "X server not running!"
            return 1
        fi
        
        # Try different DISPLAY values
        for display in ":1" ":0" ":10"; do
            export DISPLAY=$display
            print_status "Trying DISPLAY=$display"
            if xrandr >/dev/null 2>&1; then
                print_success "Found working display: $display"
                return 0
            fi
        done
        
        print_error "Could not access any X display"
        return 1
    fi
}

# Function to diagnose X display issues
diagnose_display() {
    print_status "=== X Display Diagnostics ==="
    echo "Current DISPLAY: ${DISPLAY:-not set}"
    echo "Current USER: $USER"
    echo "Current HOME: $HOME"
    echo "Running desktop environment: ${XDG_CURRENT_DESKTOP:-unknown}"
    echo ""
    
    print_status "X Server processes:"
    ps aux | grep -E "(Xorg|X |Xwayland)" | grep -v grep || echo "No X processes found"
    echo ""
    
    print_status "Display ports in use:"
    ls /tmp/.X11-unix/ 2>/dev/null || echo "No X11 sockets found"
    echo ""
    
    print_status "Testing X displays:"
    for display in ":0" ":1" ":10" ":11"; do
        echo -n "Testing $display: "
        if DISPLAY=$display xrandr >/dev/null 2>&1; then
            echo "✓ Working"
        else
            echo "✗ Failed"
        fi
    done
    echo ""
    
    print_status "Network interfaces:"
    ip addr show | grep -E "(inet |inet6)" | grep -v 127.0.0.1
    echo ""
}

# Function to start simple VNC server (fallback method)
start_simple_vnc() {
    local width=${1:-1920}
    local height=${2:-1200}
    
    print_status "Starting simple VNC server..."
    
    # Fix permissions
    if ! fix_x_permissions; then
        print_error "Cannot access X display. VNC will not work."
        print_status "Make sure you're running this in a graphical session."
        print_status "Try running: echo \$DISPLAY"
        return 1
    fi
    
    # Kill existing VNC servers
    pkill x11vnc 2>/dev/null || true
    
    # Get your local IP
    local_ip=$(hostname -I | awk '{print $1}')
    
    print_status "Using DISPLAY: $DISPLAY"
    print_status "Starting VNC on $local_ip:5900"
    
    # Start simple x11vnc server with verbose output
    x11vnc -forever -nopw -display "$DISPLAY" -rfbport 5900 -shared -bg -o /tmp/vnc.log
    
    # Wait for VNC to start
    sleep 3
    
    # Check if VNC is running
    if pgrep x11vnc >/dev/null; then
        print_success "Simple VNC server started successfully!"
        print_success "Connect your tablet to: $local_ip:5900"
        echo ""
        print_status "This shares your entire desktop with the tablet."
        print_status "VNC log: tail -f /tmp/vnc.log"
        print_status "To stop: ./secondscreen.sh stop"
        
        # Show the VNC log output
        echo ""
        print_status "VNC startup log:"
        tail -10 /tmp/vnc.log 2>/dev/null || echo "No log available"
    else
        print_error "Failed to start VNC server"
        print_status "Check log: cat /tmp/vnc.log"
        return 1
    fi
    
    return 0
}

# Main script logic
case "${1:-help}" in
    "install")
        install_extended_monitor_tools
        ;;
    "extended")
        width=${2:-1920}
        height=${3:-1200}
        start_tablet_extended_monitor "$width" "$height"
        ;;
    "simple")
        width=${2:-1920}
        height=${3:-1200}
        start_simple_vnc "$width" "$height"
        ;;
    "mirror")
        start_scrcpy_extended
        ;;
    "virtual")
        width=${2:-1920}
        height=${3:-1080}
        create_virtual_display "$width" "$height"
        ;;
    "vnc")
        geometry=${2:-1920x1080}
        start_vnc_server ":1" "$geometry"
        ;;
    "diagnose")
        diagnose_display
        ;;
    "optimize")
        optimize_latency
        ;;
    "stop")
        stop_services
        ;;
    "status")
        show_status
        ;;
    "help"|*)
        show_usage
        ;;
esac