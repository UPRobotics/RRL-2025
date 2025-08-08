#!/usr/bin/env python3
"""
Ultra-Low Latency Camera Viewer using FFmpeg-Python
Main entry point using ffmpeg-python for minimal latency with grid and fullscreen support
"""

import sys
import os

# Add src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def main():
    """Main entry point for the FFmpeg camera viewer application."""
    print("Starting Ultra-Low Latency FFmpeg Camera Viewer...")
    print("Features:")
    print("  • Ultra-low latency using FFmpeg-Python")
    print("  • Grid view (2x2) and fullscreen modes")
    print("  • Stream quality switching")
    print("  • Real-time frame dropping for minimal delay")
    print()
    
    try:
        from src.ffmpeg_camera import main as run_ffmpeg_cameras
        return run_ffmpeg_cameras()
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure to install dependencies:")
        print("  pip install ffmpeg-python")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
