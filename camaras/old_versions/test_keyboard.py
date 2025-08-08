#!/usr/bin/env python3
"""
Test script to verify keyboard input codes
"""

import cv2
import numpy as np

def test_keyboard():
    """Test keyboard input and display key codes"""
    
    # Create a simple test window
    cv2.namedWindow("Keyboard Test", cv2.WINDOW_NORMAL)
    
    # Create a black image for display
    img = np.zeros((300, 600, 3), dtype=np.uint8)
    
    print("=== Keyboard Test ===")
    print("Press keys to see their codes")
    print("Expected codes:")
    print("  R: 82, r: 114")
    print("  P: 80, p: 112")
    print("  SPACE: 32")
    print("  ESC: 27")
    print("  q: 113")
    print("Press 'q' or ESC to quit")
    print("====================")
    
    while True:
        # Clear the image
        img.fill(0)
        
        # Add instructions
        cv2.putText(img, "Press keys to test", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, "R/r - Rotation", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(img, "P/p - Restart", (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(img, "SPACE - Toggle", (50, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(img, "q/ESC - Quit", (50, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        cv2.imshow("Keyboard Test", img)
        
        # Wait for key press
        key = cv2.waitKey(1) & 0xFF
        
        if key != 255:  # A key was pressed
            char = chr(key) if 32 <= key <= 126 else 'N/A'
            print(f"Key pressed: {key} (char: '{char}')")
            
            if key == ord('q') or key == 27:  # ESC
                print("Quitting...")
                break
            elif key == ord('R') or key == ord('r'):
                print(">>> ROTATION key detected!")
            elif key == ord('P') or key == ord('p'):
                print(">>> RESTART key detected!")
            elif key == ord(' '):
                print(">>> SPACE key detected!")
            elif key in [81, 2, 63234]:
                print(">>> LEFT arrow detected!")
            elif key in [83, 3, 63235]:
                print(">>> RIGHT arrow detected!")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_keyboard()
