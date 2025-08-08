#!/bin/bash

# Ultra-Low Latency FFmpeg Camera Viewer Launcher
echo "Starting Ultra-Low Latency FFmpeg Camera Viewer..."
echo "This version uses ffmpeg-python for minimal latency"
echo ""

cd "$(dirname "$0")"

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed"
    exit 1
fi

# Check if FFmpeg is available
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: FFmpeg is not installed"
    echo "Please install FFmpeg:"
    echo "  Ubuntu/Debian: sudo apt install ffmpeg"
    echo "  CentOS/RHEL: sudo yum install ffmpeg"
    echo "  macOS: brew install ffmpeg"
    exit 1
fi

# Check and install Python dependencies
echo "Checking Python dependencies..."
python3 -c "import ffmpeg" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing ffmpeg-python..."
    pip3 install ffmpeg-python
fi

python3 -c "import cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing OpenCV..."
    pip3 install opencv-python
fi

python3 -c "import numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing NumPy..."
    pip3 install numpy
fi

echo ""
echo "Starting FFmpeg camera viewer..."
echo "Controls:"
echo "  'q' or ESC - Quit"
echo "  'f' - Toggle fullscreen/grid mode"
echo "  ← → Arrow keys - Navigate cameras (fullscreen mode)"
echo "  'l' - Toggle stream quality (main/low-res)"
echo "  'r' - Restart failed cameras"
echo "  's' - Show statistics"
echo ""

python3 main_ffmpeg.py

echo "FFmpeg camera viewer stopped."
