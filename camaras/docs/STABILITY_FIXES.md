# Dual Stream Stability Fixes

## Issues Resolved

### Segmentation Fault Prevention
The previous segmentation fault when switching streams has been addressed with several improvements:

1. **Thread-Safe Stream Switching**
   - Added proper synchronization during stream switches
   - Temporary pause of capture thread during URL changes
   - Improved cleanup of OpenCV VideoCapture objects

2. **Staggered Camera Switching**
   - In grid mode, cameras now switch with 200ms delays between each
   - Prevents simultaneous resource allocation conflicts
   - Reduces system load during bulk stream changes

3. **Enhanced Error Handling**
   - Better capture cleanup with proper error handling
   - Safety checks before accessing VideoCapture objects
   - Improved memory management for frame buffers

4. **Robust Capture Setup**
   - More thorough cleanup of failed capture attempts
   - Multiple fallback strategies for backend selection
   - Better detection of invalid capture states

## Key Improvements

### Stream Switching Process
```python
def switch_stream_quality(self) -> bool:
    # 1. Pause capture thread safely
    old_running = self.running
    self.running = False
    time.sleep(0.1)  # Allow current operations to complete
    
    # 2. Clear frame queue to prevent stale frames
    while not self.frame_queue.empty():
        self.frame_queue.get_nowait()
    
    # 3. Switch URL and mode
    self.current_url = new_url
    self.is_lowres_mode = not self.is_lowres_mode
    
    # 4. Safely release capture
    if self.cap and self.cap.isOpened():
        self.cap.release()
        time.sleep(0.05)  # Brief cleanup pause
    
    # 5. Resume operations
    self.running = old_running
```

### Grid Mode Switching
```python
def _toggle_stream_quality(self):
    for i, (camera_id, camera) in enumerate(self.cameras.items()):
        if i > 0:
            time.sleep(0.2)  # Stagger switches
        camera.switch_stream_quality()
```

## Testing

Use the included test script to verify stability:

```bash
python3 test_stream_switching.py
```

This will test stream switching with a single camera to ensure no crashes occur.

## Usage Notes

- **Grid Mode**: Press 'L' to switch all cameras (with staggered timing)
- **Fullscreen Mode**: Press 'L' to switch only the current camera
- **Recovery**: If switching fails, cameras will automatically retry connection
- **Stability**: Stream switching is now much more stable and crash-resistant

## Visual Indicators

- **Main Stream**: Green camera ID text: `Cam 1 (MAIN)`
- **Low-Res Stream**: Yellow camera ID text: `Cam 1 (LOW-RES)`
- **Switching Status**: Console logs show switching progress
- **Health Status**: Overlays show connection health during switches

The system now handles stream switching robustly without segmentation faults or crashes.
