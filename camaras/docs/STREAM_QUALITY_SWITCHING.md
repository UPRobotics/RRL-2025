# Stream Quality Switching Feature

## Overview
The robust camera viewer now supports dual-stream quality switching, allowing you to switch all cameras between high-resolution (1080p) and low-resolution (480p) streams in real-time.

## Configuration

### Config File Setup
Each camera can have two stream URLs configured in `config.ini`:

```ini
[cameras]
# High resolution streams (main streams)
camera2_url = rtsp://192.168.0.201:554/stream1
camera3_url = rtsp://192.168.0.202:554/stream1

# Low resolution streams (for quality switching)
camera2_url_lowres = rtsp://192.168.0.201:554/stream2
camera3_url_lowres = rtsp://192.168.0.202:554/stream2
```

### Camera Stream Configuration Examples

#### IP Camera with Multiple Streams
```ini
# Main stream (1080p)
camera1_url = rtsp://192.168.0.100:554/stream1

# Low-res stream (480p)
camera1_url_lowres = rtsp://192.168.0.100:554/stream2
```

#### Camera with Authentication
```ini
# Main stream with credentials
camera1_url = rtsp://admin:password@192.168.0.100:554/profile0

# Low-res stream with credentials
camera1_url_lowres = rtsp://admin:password@192.168.0.100:554/profile1
```

## Usage

### Keyboard Controls
- **'L' or 'l'** - Switch all cameras between high-res and low-res streams
- All other controls remain the same

### Stream Quality Switching
1. Press 'L' to switch all cameras to low-resolution streams
2. Press 'L' again to switch all cameras back to high-resolution streams
3. The system will automatically restart streams with the new quality setting

### Stream Quality Indicators
- **Statistics View ('s' key)**: Shows current stream quality for each camera
- **Health View ('h' key)**: Shows camera health status and current stream quality
- **Console Logs**: Display stream switching events and current quality

## Benefits

### High-Resolution Mode (Default)
- **Quality**: 1080p streams for maximum detail
- **Use Case**: Normal viewing, recording, detailed monitoring
- **Bandwidth**: Higher bandwidth requirement

### Low-Resolution Mode
- **Quality**: 480p streams for reduced bandwidth
- **Use Case**: Remote monitoring, bandwidth-limited networks, performance optimization
- **Bandwidth**: Significantly reduced bandwidth requirement
- **Performance**: Lower CPU usage, faster response times

## Technical Implementation

### Automatic Stream Switching
- All cameras switch simultaneously when 'L' is pressed
- Cameras automatically restart with new stream URLs
- Frame buffers are cleared during transition for immediate quality change
- No manual configuration required during runtime

### Fallback Behavior
- If a camera doesn't have a low-res stream configured, it will continue using the high-res stream
- Warning messages are displayed for cameras without dual-stream configuration
- System continues to function normally with mixed stream qualities

### Ultra-Low Latency Maintained
- Stream switching preserves ultra-low latency settings
- FFmpeg processes are optimized for both stream qualities
- Frame dropping and buffering strategies remain consistent

## Troubleshooting

### Common Issues

#### No Low-Res Stream Available
**Symptoms**: Warning message "No cameras have low-res streams configured"
**Solution**: Add `camera<N>_url_lowres` entries to your config.ini

#### Stream Switching Slow
**Symptoms**: Delay when switching between qualities
**Solution**: This is normal - cameras need to restart with new URLs

#### Mixed Stream Qualities
**Symptoms**: Some cameras don't switch quality
**Solution**: Check that all cameras have both high-res and low-res URLs configured

### Log Messages
- `"Switching all cameras to low-res streams"` - Quality switch initiated
- `"Camera X switching to low-res stream"` - Individual camera switching
- `"All cameras switched to low-res streams"` - Switch completed

## Performance Considerations

### Bandwidth Usage
- High-res streams: ~2-8 Mbps per camera (depends on resolution and quality)
- Low-res streams: ~0.5-2 Mbps per camera
- Switch to low-res for network-constrained environments

### CPU Usage
- Low-res streams reduce CPU usage for decoding
- Recommended for systems with limited processing power
- Helps maintain smooth playback with many cameras

### Memory Usage
- Lower resolution streams use less memory for frame buffers
- Helps with system stability when running many cameras

## Integration with Existing Features

### Works With All Modes
- ✅ Grid view mode
- ✅ Fullscreen mode
- ✅ Camera rotation
- ✅ Statistics and health monitoring
- ✅ Auto-restart functionality

### Persistent Settings
- Stream quality preference is not persistent (resets to high-res on restart)
- Camera rotations remain persistent and independent of stream quality
- Future versions may add quality preference persistence

## Camera Compatibility

### Supported Camera Types
- IP cameras with multiple stream profiles
- RTSP cameras with stream1/stream2 endpoints
- Cameras with profile0/profile1 configurations
- Any camera that provides multiple resolution streams

### Stream URL Formats
```
rtsp://ip:port/stream1          # High-res
rtsp://ip:port/stream2          # Low-res

rtsp://user:pass@ip:port/profile0  # High-res with auth
rtsp://user:pass@ip:port/profile1  # Low-res with auth
```

## Future Enhancements

### Planned Features
- Individual camera quality switching
- Automatic quality adjustment based on network conditions
- Stream quality persistence across restarts
- Bandwidth monitoring and automatic switching
- Quality indicators in the UI overlay
