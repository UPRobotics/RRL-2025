#!/usr/bin/env python3
"""
Shared Stream Multi-Camera Viewer Main
Entry point for the shared stream approach (single RTSP connection, multiple virtual cameras)
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def main():
    """Main entry point"""
    print("=== Shared Stream Multi-Camera Viewer ===")
    print("Using single RTSP connection for multiple virtual cameras")
    print("Features:")
    print("  • Single RTSP connection (no connection conflicts)")
    print("  • Multiple virtual cameras showing same stream")
    print("  • Ultra-low latency frame distribution")
    print("  • Scalable 1-8 camera simulation")
    print("  • Automatic grid layout")
    print("=========================================")
    
    try:
        from src.shared_stream_viewer import main as run_viewer
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
