#!/usr/bin/env python3
"""
Simple Ultra-Low Latency Camera Viewer Main
Entry point for the simplified FFmpeg viewer
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def main():
    """Main entry point"""
    print("=== Ultra-Low Latency Camera Viewer ===")
    print("Starting simplified FFmpeg viewer...")
    print("Features:")
    print("  • Ultra-low latency streaming")
    print("  • Scalable 1-8 camera support")
    print("  • Automatic grid layout")
    print("  • Real-time frame dropping")
    print("========================================")
    
    try:
        from src.simple_ffmpeg_viewer import main as run_viewer
        return run_viewer()
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure ffmpeg-python is installed:")
        print("  pip install ffmpeg-python")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
