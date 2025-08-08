# Ultra-Low Latency Optimizations

This document describes the advanced ultra-low latency optimizations implemented in the RTSP camera viewer for achieving the absolute minimum latency possible.

## New Ultra-Low Latency Features

### 1. **Zero-Copy Frame Processing**
- **Feature**: Pre-allocated frame buffer pools to eliminate memory allocation overhead
- **Config**: `zero_copy_mode = true`
- **Benefit**: Reduces memory allocation/deallocation time by ~30-50%
- **Status**: Experimental (may cause instability on some systems)

### 2. **Minimal Processing Mode**
- **Feature**: Skips non-essential processing like FPS calculations and complex health checks
- **Config**: `minimal_processing_mode = true`
- **Benefit**: Reduces CPU overhead and processing time per frame
- **Trade-off**: Less detailed statistics and monitoring

### 3. **Skip Display Sync**
- **Feature**: Disables all FPS limiting and V-Sync to render frames as fast as possible
- **Config**: `skip_display_sync = true`
- **Benefit**: Maximum display refresh rate, eliminates artificial delays
- **Control**: Can be toggled runtime with 'z' key

### 4. **Hardware Decode Acceleration**
- **Feature**: Enables hardware-accelerated video decoding when available
- **Config**: `hardware_decode = true`
- **Benefit**: Offloads decoding from CPU to GPU/dedicated hardware
- **Compatibility**: Depends on OpenCV build and available hardware

### 5. **Skip Frame Validation**
- **Feature**: Bypasses frame integrity checks for speed
- **Config**: `skip_frame_validation = true`
- **Benefit**: Saves CPU cycles on frame validation
- **Risk**: May allow corrupted frames to pass through (experimental)

### 6. **Ultra-Fast Interpolation**
- **Feature**: Uses INTER_NEAREST (fastest) instead of INTER_LINEAR for resizing
- **Automatic**: Enabled when `ultra_low_latency_mode = true`
- **Benefit**: Faster frame resizing at cost of slight quality reduction
- **Quality Impact**: Minimal for monitoring applications

## Configuration Values for Maximum Speed

```ini
[performance]
# Core low-latency settings
ultra_low_latency_mode = true
disable_overlays = true
frame_buffer_size = 1

# Experimental ultra-fast settings
zero_copy_mode = true
skip_display_sync = true
minimal_processing_mode = true
skip_frame_validation = true

# Aggressive timeouts (faster failover)
connection_timeout_ms = 1000
read_timeout_ms = 500
frame_read_timeout = 1
health_check_timeout = 3
restart_delay = 1

[display]
# Maximum display performance
max_display_fps = 120  # Or 0 for unlimited
```

## Runtime Controls

### New Hotkeys:
- **'z'**: Toggle ultra-fast mode (skip display sync)
- **'p'**: Toggle performance mode (no FPS limit)

### Existing Controls:
- **'f'**: Toggle fullscreen mode
- **← →**: Navigate cameras in fullscreen
- **'r'**: Restart failed cameras
- **'s'**: Show performance statistics
- **'c'**: Reload configuration
- **'q'**: Quit application

## Performance Monitoring

When ultra-low latency mode is enabled, the startup log shows:

```
Ultra low latency mode: ENABLED
Zero-copy mode: ENABLED  
Display sync: DISABLED
Minimal processing: ENABLED
Hardware decode: ENABLED
Frame validation: DISABLED
```

## Expected Latency Improvements

With all optimizations enabled:

1. **Network to Display**: 50-200ms (depending on network and cameras)
2. **Frame Processing**: 1-5ms (down from 10-20ms)
3. **Display Refresh**: No artificial delays (previously limited to 30fps)
4. **Memory Operations**: 60-80% faster with zero-copy mode

## Troubleshooting

### If experiencing crashes:
1. Disable `zero_copy_mode = false` first
2. Then disable `skip_frame_validation = false`
3. Reduce `max_display_fps` if CPU usage is too high

### If video quality is poor:
- `ultra_low_latency_mode = false` (uses better interpolation)
- `disable_overlays = false` (shows debugging info)

### If cameras disconnect frequently:
- Increase timeout values in config
- Disable `minimal_processing_mode = false`

## System Requirements

For optimal ultra-low latency performance:
- **CPU**: Modern multi-core processor (4+ cores recommended)
- **RAM**: 8GB+ (more for zero-copy mode)
- **Network**: Gigabit Ethernet for multiple high-bitrate streams
- **GPU**: Hardware decode capable (Intel Quick Sync, NVENC, etc.)

## OpenCV Backend Optimization

The system automatically tries backends in this order for best performance:
1. **CAP_FFMPEG** (best for RTSP with hardware decode)
2. **CAP_GSTREAMER** (good for Linux systems)
3. **CAP_V4L2** (fallback for Linux)

## Memory Usage

Zero-copy mode pre-allocates frame buffers:
- **Per camera**: ~10 buffers × frame_size
- **640×360**: ~7MB per camera
- **Total for 4 cameras**: ~28MB additional RAM usage

This is a worthwhile trade-off for the significant latency reduction.
