#!/usr/bin/env python3
"""
Robust 4-Camera FFmpeg Viewer Main
Entry point for the robust FFmpeg viewer with improved error handling
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def main():
    """Main entry point"""
    print("=== Robust Multi-Camera RTSP Viewer ===")
    print("Ultra-low latency viewer with advanced features")
    print("Features:")
    print("  • Dual-stream quality switching (L key)")
    print("  • Persistent camera rotation (R key)")
    print("  • Ultra-low latency FFmpeg optimization")
    print("  • Automatic scalability (any number of cameras)")
    print("  • Grid and fullscreen view modes")
    print("  • Robust error handling and recovery")
    print("  • Health monitoring and statistics")
    print("=========================================")
    
    try:
        from robust_ffmpeg_viewer import main as run_viewer
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
