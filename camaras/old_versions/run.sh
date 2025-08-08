#!/bin/bash

# Simple launcher for the Multi-Camera RTSP Viewer
# This script runs the main.py entry point

cd "$(dirname "$0")"

echo "Starting Multi-Camera RTSP Viewer..."
echo "Using organized project structure:"
echo "  • Entry point: main.py"
echo "  • Source code: src/"
echo "  • Documentation: docs/"
echo "  • Tests: tests/"
echo "  • Scripts: scripts/"
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
    pip3 install -r requirements.txt
fi

echo "Starting camera viewer..."
python3 main.py

echo "Camera viewer stopped."
