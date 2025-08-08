#!/usr/bin/env python3
"""
Test script to demonstrate camera rotation persistence
"""

import json
import os

def test_rotation_persistence():
    """Test the rotation persistence functionality"""
    
    rotation_file = "camera_rotations.json"
    
    print("=== Camera Rotation Persistence Test ===")
    
    # Check if rotation file exists
    if os.path.exists(rotation_file):
        print(f"✓ Rotation file exists: {rotation_file}")
        
        try:
            with open(rotation_file, 'r') as f:
                rotations = json.load(f)
            
            print("Current saved rotations:")
            for camera_id, rotation in rotations.items():
                degrees = rotation * 90
                print(f"  Camera {camera_id}: {degrees}°")
                
            if not rotations:
                print("  No rotations saved yet")
                
        except Exception as e:
            print(f"✗ Error reading rotation file: {e}")
    else:
        print(f"✗ Rotation file not found: {rotation_file}")
        print("This is normal if you haven't rotated any cameras yet.")
    
    print("\n=== How to use rotation persistence ===")
    print("1. Run the camera viewer:")
    print("   python3 src/robust_ffmpeg_viewer.py")
    print("2. Press SPACE to switch to fullscreen mode")
    print("3. Press R to rotate the current camera 90° clockwise")
    print("4. Press LEFT/RIGHT arrows to switch between cameras")
    print("5. Press T to reset all camera rotations to 0°")
    print("6. Exit the app - rotations are saved automatically")
    print("7. Restart the app - rotations are restored automatically")
    
    print("\n=== Features ===")
    print("• Rotations are saved to:", rotation_file)
    print("• Rotations are saved immediately when changed")
    print("• Rotations are restored when app starts")
    print("• Only cameras that exist in config are loaded")
    print("• Invalid rotation values are ignored")
    print("• Reset all rotations with T key")
    
    print("\n=== Rotation Values ===")
    print("0 = 0°   (normal)")
    print("1 = 90°  (clockwise)")
    print("2 = 180° (upside down)")
    print("3 = 270° (counter-clockwise)")

if __name__ == "__main__":
    test_rotation_persistence()
