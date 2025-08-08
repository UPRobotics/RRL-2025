#!/bin/bash

# Ultra-Low Latency Camera Viewer using FFplay
# Direct implementation of the optimized FFplay command

echo "Starting Ultra-Low Latency FFplay Camera Viewer..."
echo "Using direct FFplay processes for minimal latency"
echo ""

# Configuration - edit these URLs for your cameras
CAMERA_URLS=(
    "rtsp://admin:admin@192.168.0.4:8554/profile0"
    "rtsp://admin:admin@192.168.0.5:8554/profile0" 
    "rtsp://admin:admin@192.168.0.6:8554/profile0"
    "rtsp://admin:admin@192.168.0.7:8554/profile0"
)

# Low-res URLs (optional)
CAMERA_LOWRES_URLS=(
    "rtsp://admin:admin@192.168.0.4:8554/profile1"
    "rtsp://admin:admin@192.168.0.5:8554/profile1"
    "rtsp://admin:admin@192.168.0.6:8554/profile1"
    "rtsp://admin:admin@192.168.0.7:8554/profile1"
)

# Window positions for 2x2 grid
POSITIONS_X=(100 1060 100 1060)    # x coordinates
POSITIONS_Y=(100 100 640 640)      # y coordinates

# Window size
WINDOW_WIDTH=960
WINDOW_HEIGHT=540

# Check if ffplay is available
if ! command -v ffplay &> /dev/null; then
    echo "Error: ffplay not found!"
    echo "Please install FFmpeg: sudo apt install ffmpeg"
    exit 1
fi

echo "Controls:"
echo "  Close any window to stop all cameras"
echo "  Each window can be moved and resized"
echo ""

# Array to store process IDs
declare -a PIDS=()

# Function to cleanup processes
cleanup() {
    echo "Stopping all camera processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
        fi
    done
    wait
    echo "All cameras stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start cameras
for i in "${!CAMERA_URLS[@]}"; do
    camera_id=$((i + 1))
    url="${CAMERA_URLS[$i]}"
    pos_x="${POSITIONS_X[$i]}"
    pos_y="${POSITIONS_Y[$i]}"
    
    echo "Starting Camera $camera_id at position ($pos_x, $pos_y)..."
    
    # Your optimized FFplay command
    ffplay \
        -rtsp_transport tcp \
        -i "$url" \
        -an \
        -probesize 1024 \
        -analyzeduration 1000000 \
        -fflags nobuffer+fastseek \
        -flags low_delay \
        -err_detect ignore_err \
        -framedrop \
        -max_delay 0 \
        -reorder_queue_size 0 \
        -vf "setpts=0.5*PTS,scale=$WINDOW_WIDTH:$WINDOW_HEIGHT" \
        -window_title "Camera $camera_id - Ultra Low Latency" \
        -x "$pos_x" \
        -y "$pos_y" \
        > /dev/null 2>&1 &
    
    # Store process ID
    PIDS+=($!)
    
    # Brief delay between camera starts
    sleep 0.5
done

echo "All cameras started successfully!"
echo "Process IDs: ${PIDS[*]}"
echo "Press Ctrl+C to stop all cameras"

# Wait for any process to exit
while true; do
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "Camera process $pid exited. Stopping all cameras..."
            cleanup
        fi
    done
    sleep 1
done
