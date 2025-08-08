#!/usr/bin/env python3
"""
Multi-Camera RTSP Viewer - Main Entry Point
A high-performance application for displaying multiple RTSP camera streams in real-time.
"""


from src.camaras_advanced import main as run_cameras

def main():
    """Main entry point for the camera viewer application."""
    run_cameras()

if __name__ == "__main__":
    main()
