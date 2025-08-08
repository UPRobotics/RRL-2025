# Anti-Flickering Improvements

This document describes the improvements made to eliminate flickering between frames and "camera not connected" messages.

## Issues Addressed

### 1. **Aggressive Frame Validation**
- **Problem**: Frame validation was too strict, dropping valid frames with minor format issues
- **Solution**: Made validation more lenient, accepting frames with different data types and trying to convert them
- **Result**: Fewer dropped frames, less flickering

### 2. **Quick Health Check Failures**
- **Problem**: Camera health checks were too sensitive, marking cameras as unhealthy too quickly
- **Solution**: Extended health timeout to 2x the configured value for safety margin
- **Result**: More stable connection status

### 3. **Immediate "Not Connected" Messages**
- **Problem**: Showing "not connected" immediately when queue was empty, even if camera was healthy
- **Solution**: Added grace periods:
  - 10 seconds before showing "RECONNECTING..."
  - 30 seconds before showing "NO SIGNAL"
- **Result**: Reduced flickering to placeholder messages

### 4. **Short Last Good Frame Retention**
- **Problem**: Last good frames were not kept long enough during temporary connection issues
- **Solution**: Keep last good frames for up to 15 seconds with timestamps
- **Result**: Smoother display during brief connection hiccups

### 5. **Overly Aggressive Timeouts**
- **Problem**: Very short timeouts caused frequent false disconnections
- **Solution**: Adjusted timeouts:
  - Health check: 8 seconds (was 5)
  - Connection: 3000ms (was 2000ms)
  - Read: 2000ms (was 1500ms)
  - Frame read: 3 seconds (was 2)
- **Result**: More stable connections with less false positives

## Technical Details

### Frame Validation Improvements
```python
# Before: Strict validation
if len(frame.shape) != 3 or frame.shape[2] != 3:
    return None  # Drop frame

# After: Lenient validation
if len(frame.shape) == 3 and frame.shape[2] == 3:
    if frame.dtype != np.uint8:
        frame = frame.astype(np.uint8)  # Convert instead of dropping
    return frame
```

### Health Check Improvements
```python
# Before: Strict timeout
return (self.connection_healthy and 
        time.time() - self.last_frame_time < self.health_check_timeout)

# After: Extended timeout with safety margin
extended_timeout = self.health_check_timeout * 2
return (self.connection_healthy and 
        time.time() - self.last_frame_time < extended_timeout)
```

### Last Good Frame Management
```python
# Added frame aging system
if frame is not None:
    self.last_good_frames[camera_id] = frame
    self.last_good_frame_times[camera_id] = time.time()
elif camera_id in self.last_good_frames:
    frame_age = time.time() - self.last_good_frame_times[camera_id]
    if frame_age < 15.0:  # Use for up to 15 seconds
        frames[camera_id] = self.last_good_frames[camera_id]
```

## Configuration Changes

Updated `config.ini` with more conservative timeouts:
```ini
health_check_timeout = 8      # Was 5
connection_timeout_ms = 3000  # Was 2000
read_timeout_ms = 2000        # Was 1500
frame_read_timeout = 3        # Was 2
```

## Results

These improvements provide:
- **Smoother Display**: Less flickering between frames and error messages
- **Better Stability**: Cameras stay connected longer during minor network issues
- **Graceful Degradation**: Progressive error messages instead of immediate failures
- **Maintained Performance**: Optimizations still active, just less aggressive

## Monitoring

To monitor the effectiveness:
- Watch for reduced "RECONNECTING..." messages in logs
- Check drop rates with 's' key - should be lower
- Observe smoother video playback without brief black screens
- Network hiccups should not immediately show error messages

The system now balances ultra-low latency with connection stability for a better user experience.
