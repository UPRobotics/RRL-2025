#!/bin/bash

# Multi-Camera RTSP Viewer Launcher
echo "Starting Ultra-Low Latency Multi-Camera RTSP Viewer..."
echo "Advanced Features: Zero-copy processing, hardware decode, aggressive frame dropping"
echo "Optimized for real-time monitoring with sub-100ms latency"
echo ""

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed"
    exit 1
fi

# Check if OpenCV is installed
python3 -c "import cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing required packages..."
    pip3 install opencv-python numpy
fi

# Run the application
echo "Starting ultra-low latency camera viewer..."
echo "Controls:"
echo "  'q' or ESC - Quit"
echo "  'f' - Toggle fullscreen mode"
echo "  ← → Arrow keys - Navigate cameras"
echo "  'l' - Toggle stream quality (main/low-res)"
echo "  'p' - Toggle performance mode (no FPS limit)"
echo "  'z' - Toggle ultra-fast mode (skip display sync)"
echo "  'r' - Restart failed cameras"  
echo "  's' - Show statistics"
echo "  'c' - Reload configuration"
echo ""
echo "Ultra-low latency features enabled in config.ini:"
echo "  • Zero-copy frame processing"
echo "  • Hardware decode acceleration"
echo "  • Aggressive frame dropping"
echo "  • Skip display sync for max FPS"
echo "  • Minimal processing mode"
echo ""

python3 ../main.py

echo "Camera viewer stopped."
