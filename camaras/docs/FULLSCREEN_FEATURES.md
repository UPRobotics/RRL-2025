# Fullscreen Mode and Window Features

## New Features Added

### 1. Resizable and Maximizable Window
- The OpenCV window is now created with `cv2.WINDOW_NORMAL` flag
- You can resize the window by dragging the corners
- You can maximize the window using your window manager
- The display automatically scales to fit the window size

### 2. Fullscreen Single Camera Mode
- Press **'F'** to toggle between grid view and fullscreen mode
- In fullscreen mode, you see one camera at a time in large size
- Optimal for detailed monitoring of specific cameras

### 3. Camera Navigation
- Use **← (Left Arrow)** and **→ (Right Arrow)** keys to cycle through cameras
- Works in fullscreen mode to switch between cameras
- In grid mode, arrow keys will switch to fullscreen mode with the selected camera
- Camera selection cycles through all available cameras (1 → 2 → 3 → 4 → 1...)

### 4. Enhanced Controls
- **'F'** - Toggle fullscreen mode (single camera view)
- **← →** - Navigate through cameras in fullscreen mode
- **'Q' or ESC** - Quit application
- **'R'** - Restart failed cameras
- **'S'** - Show statistics
- **'C'** - Reload configuration

### 5. Smart Overlays
- Grid mode: Shows overall system status and controls
- Fullscreen mode: Shows camera-specific information and navigation controls
- Real-time FPS, health status, and frame information

### 6. Improved User Experience
- Window starts at optimal size but can be resized
- Fullscreen mode uses appropriate scaling for the camera feed
- Clear visual indicators for current mode and available controls
- Status overlays adapt to the current view mode

## Usage Tips

1. **Start in Grid Mode**: See all 4 cameras at once
2. **Switch to Fullscreen**: Press 'F' to focus on one camera
3. **Navigate Cameras**: Use arrow keys to cycle through cameras in fullscreen
4. **Resize Window**: Drag window corners to resize as needed
5. **Maximize**: Use your OS window controls to maximize
6. **Return to Grid**: Press 'F' again to see all cameras

The application now provides a much more flexible viewing experience suitable for both overview monitoring (grid mode) and detailed inspection (fullscreen mode).
