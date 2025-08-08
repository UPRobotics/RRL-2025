# Camera Freeze Fix - Timeout Handling

## Problem Solved
When cameras disconnect and reconnect, OpenCV's FFMPEG backend was hanging on `cap.read()` calls, causing the camera to freeze and display old frames. The warning message:
```
[ WARN:0@201.055] global cap_ffmpeg_impl.hpp:453 _opencv_ffmpeg_interrupt_callback Stream timeout triggered after 30052.128195 ms
```

## Root Cause
- OpenCV FFMPEG backend doesn't handle network interruptions gracefully
- `cap.read()` calls can hang indefinitely when stream is interrupted
- No built-in timeout handling for individual frame reads
- Connection objects weren't being properly cleaned up on reconnection

## Solutions Implemented

### 1. **Multiple Timeout Mechanisms**
- **Signal-based timeout**: Uses `SIGALRM` for Unix/Linux systems (primary method)
- **Threading-based timeout**: Fallback using `TimeoutFrameReader` class
- **Direct read**: Last resort without timeout for maximum compatibility

### 2. **Configurable Timeouts**
Added to `config.ini`:
```ini
# Health check timeout in seconds
health_check_timeout = 10

# Connection timeout in milliseconds for RTSP streams  
connection_timeout_ms = 5000

# Read timeout in milliseconds for individual frames
read_timeout_ms = 5000

# Frame read timeout in seconds
frame_read_timeout = 5
```

### 3. **Enhanced Connection Management**
- **Proper cleanup**: Release capture objects completely before reconnection
- **Connection timeouts**: Set `CAP_PROP_OPEN_TIMEOUT_MSEC` and `CAP_PROP_READ_TIMEOUT_MSEC`
- **Graceful degradation**: Multiple fallback methods for different systems

### 4. **Robust Reconnection Logic**
- **Timeout detection**: Monitor time since last successful frame read
- **Forced reconnection**: Automatically reconnect when timeouts occur
- **Recovery logging**: Clear messages when cameras recover
- **State management**: Proper connection health tracking

### 5. **Cross-Platform Compatibility**
- **Signal support detection**: Checks if `signal` module is available and working
- **Threading fallback**: `TimeoutFrameReader` for systems without signal support
- **Error handling**: Graceful fallback to direct reads if all timeout methods fail

## Key Improvements

### Before:
- Camera would freeze after reconnection
- No timeout handling for frame reads
- Poor cleanup of connection objects
- Single timeout mechanism that might not work on all systems

### After:
- **Automatic recovery** from connection interruptions
- **Multiple timeout strategies** for maximum compatibility
- **Proper cleanup** prevents hanging connections
- **Configurable timeouts** for different network conditions
- **Clear status indication** between buffering and actual failures

## Configuration Recommendations

### For Stable Networks:
```ini
connection_timeout_ms = 5000
read_timeout_ms = 5000
frame_read_timeout = 5
health_check_timeout = 10
```

### For Unstable Networks:
```ini
connection_timeout_ms = 10000
read_timeout_ms = 10000
frame_read_timeout = 10
health_check_timeout = 20
```

### For Very Fast Recovery:
```ini
connection_timeout_ms = 3000
read_timeout_ms = 3000
frame_read_timeout = 3
health_check_timeout = 6
```

## Testing Scenarios

The fixes handle these scenarios:
1. **Camera power loss** → Shows "NO SIGNAL", auto-reconnects when back
2. **Network interruption** → Timeout detection, forced reconnection
3. **RTSP server restart** → Clean reconnection without hanging
4. **Gradual connection degradation** → Early detection and recovery
5. **Multiple simultaneous failures** → Independent recovery per camera

## Monitoring

Enhanced status display shows:
- **Frame age**: How old the current frame is
- **Health status**: OK/FAIL based on actual connectivity
- **Recovery messages**: Clear logs when cameras recover
- **Timeout indicators**: Different messages for different timeout types

The camera viewer now handles network issues gracefully and recovers automatically without user intervention.
