# Dual Stream Support

## Overview
The RTSP camera viewer now supports dual stream quality switching for each camera. Each camera can have both a main (high-resolution) stream and a low-resolution stream that you can switch between using the 'L' key.

## Configuration

### Config File Setup
In your `config.ini` file, you need to define both streams for each camera:

```ini
[cameras]
# Main streams (high resolution)
camera1_url = rtsp://192.168.0.200:554/stream1
camera2_url = rtsp://192.168.0.201:554/stream1  
camera3_url = rtsp://192.168.0.202:554/stream1
camera4_url = rtsp://192.168.0.203:554/stream1

# Low resolution streams (for 'L' key switching)
camera1_url_lowres = rtsp://192.168.0.200:554/stream2
camera2_url_lowres = rtsp://192.168.0.201:554/stream2
camera3_url_lowres = rtsp://192.168.0.202:554/stream2
camera4_url_lowres = rtsp://192.168.0.203:554/stream2
```

### Stream URL Format
- **Main stream**: `camera{N}_url` - Your primary high-resolution stream (e.g., 1080p)
- **Low-res stream**: `camera{N}_url_lowres` - Your secondary low-resolution stream (e.g., 640x480)

The low-res stream is optional. If not configured for a camera, that camera will only use its main stream.

## Controls

### Stream Quality Toggle
- **'L' key**: Toggle between main and low-resolution streams
  - In **Grid Mode**: Toggles all cameras simultaneously
  - In **Fullscreen Mode**: Toggles only the currently displayed camera

### Visual Indicators
- **Camera Overlays**: Show current stream quality (MAIN/LOW-RES)
  - Green text = Main stream
  - Yellow text = Low-res stream
- **Fullscreen Title**: Shows current stream quality in parentheses
- **Console Logs**: Display stream switching activity

## Usage Examples

### Typical Use Cases
1. **Bandwidth Management**: Switch to low-res streams when network bandwidth is limited
2. **Performance Optimization**: Use low-res streams on slower hardware
3. **Monitoring vs Detail**: Use low-res for general monitoring, switch to main for detailed inspection
4. **Remote Access**: Start with low-res streams for faster initial loading

### Operation Modes

#### Grid Mode (Default)
- Press 'L' to toggle **all cameras** between main and low-res streams
- Each camera overlay shows its current stream quality
- Status bar shows "L:Quality" reminder

#### Fullscreen Mode
- Press 'F' to enter fullscreen mode
- Press 'L' to toggle **only the current camera** between streams
- Use arrow keys (← →) to navigate between cameras
- Controls overlay shows "L: Quality" option

## Technical Details

### Stream Switching Process
1. User presses 'L' key
2. Camera(s) mark connection as unhealthy to force reconnection
3. Current capture is released
4. New connection established with the alternate stream URL
5. Visual overlay updates to show new stream quality

### Performance Considerations
- Stream switching causes a brief interruption (~2-5 seconds) as cameras reconnect
- Low-res streams typically offer:
  - Lower bandwidth usage
  - Reduced CPU processing
  - Faster frame rates
  - Improved responsiveness on slower networks

### Error Handling
- If a low-res URL is not configured, the camera will log a warning and continue using the main stream
- Failed stream switches will be logged with appropriate error messages
- Cameras automatically retry connection if switching fails

## Benefits

### Network Efficiency
- Dynamically adjust bandwidth usage based on current needs
- Reduce network congestion during peak usage times

### Hardware Optimization
- Lower CPU usage with smaller frame processing
- Better performance on resource-constrained systems

### User Experience
- Seamless switching between stream qualities
- Visual feedback on current stream status
- Per-camera control in fullscreen mode
- Bulk control in grid mode

## Troubleshooting

### Common Issues
1. **Stream won't switch**: Check that low-res URL is correctly configured in config.ini
2. **Connection failures**: Verify that both stream URLs are accessible
3. **Poor performance**: Ensure low-res stream actually has lower resolution/bitrate than main stream

### Log Messages
- `"Camera X switching to LOW-RES stream"` - Successful switch to low-res
- `"Camera X switching to MAIN stream"` - Successful switch to main
- `"Camera X has no low-res URL configured"` - Low-res URL missing in config
- `"Failed to switch streams for cameras: X"` - Stream switching failed

This dual stream feature provides flexible quality management for optimal viewing experience across different network conditions and hardware capabilities.
