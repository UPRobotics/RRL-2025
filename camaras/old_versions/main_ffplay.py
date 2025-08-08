#!/usr/bin/env python3
"""
Ultra-Low Latency Camera Viewer using FFplay
Alternative main entry point using FFplay instead of OpenCV for minimal latency
"""

import sys
import os

# Add src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.ffplay_camera import main as run_ffplay_cameras

def main():
    """Main entry point for the FFplay camera viewer application."""
    print("Starting Ultra-Low Latency FFplay Camera Viewer...")
    print("This uses FFplay directly for minimal latency instead of OpenCV")
    print()
    
    return run_ffplay_cameras()

if __name__ == "__main__":
    sys.exit(main())
