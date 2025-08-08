#!/usr/bin/env python3
"""
Setup script for Multi-Camera RTSP Viewer
"""

import os
import sys
import subprocess

def install_requirements():
    """Install required packages"""
    print("Installing required packages...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
        print("✓ Requirements installed successfully")
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to install requirements: {e}")
        return False
    return True

def check_config():
    """Check if config.ini exists"""
    if os.path.exists("config.ini"):
        print("✓ config.ini found")
        return True
    else:
        print("✗ config.ini not found")
        print("Creating example config.ini...")
        
        example_config = """[cameras]
camera1_url = rtsp://admin:admin@192.168.0.4:8554/profile0
camera2_url = rtsp://admin:admin@192.168.0.5:8554/profile0
camera3_url = rtsp://admin:admin@192.168.0.6:8554/profile0
camera4_url = rtsp://admin:admin@192.168.0.7:8554/profile0

# Optional low-resolution streams for quality switching
camera1_lowres_url = rtsp://admin:admin@192.168.0.4:8554/profile1
camera2_lowres_url = rtsp://admin:admin@192.168.0.5:8554/profile1
camera3_lowres_url = rtsp://admin:admin@192.168.0.6:8554/profile1
camera4_lowres_url = rtsp://admin:admin@192.168.0.7:8554/profile1

[display]
single_camera_width = 640
single_camera_height = 360
max_display_fps = 30
frame_buffer_size = 2

[advanced]
max_retries = 5
restart_delay = 2.0
health_check_timeout = 5.0
connection_timeout_ms = 5000
read_timeout_ms = 100
frame_read_timeout = 1.0
"""
        
        with open("config.ini", "w") as f:
            f.write(example_config)
        
        print("✓ Example config.ini created")
        print("Please edit config.ini with your camera URLs")
        return True

def main():
    """Main setup function"""
    print("Multi-Camera RTSP Viewer Setup")
    print("=" * 40)
    
    # Check Python version
    if sys.version_info < (3, 7):
        print("✗ Python 3.7+ required")
        sys.exit(1)
    else:
        print(f"✓ Python {sys.version_info.major}.{sys.version_info.minor} detected")
    
    # Install requirements
    if not install_requirements():
        sys.exit(1)
    
    # Check config
    check_config()
    
    print("\n" + "=" * 40)
    print("Setup complete!")
    print("\nTo run the camera viewer:")
    print("  python3 main.py")
    print("  or")
    print("  ./run.sh")
    print("\nFor advanced options:")
    print("  ./scripts/run_cameras.sh")

if __name__ == "__main__":
    main()
