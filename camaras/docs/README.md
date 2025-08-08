# Multi-Camera RTSP Viewer

A high-performance Python application for displaying multiple RTSP camera streams in real-time, optimized for high bitrate streams (4500kb/s, 1080p, 20fps).

## Features

- **Multi-threaded Architecture**: Each camera runs in its own thread to prevent crashes from affecting other cameras
- **Frame Dropping**: Automatically drops old frames to maintain low latency and get the latest possible frame
- **Crash Isolation**: Individual camera failures don't affect the entire application
- **Automatic Reconnection**: Failed cameras automatically attempt to reconnect
- **Real-time Performance Monitoring**: FPS, dropped frames, and connection health display
- **Optimized for High Bitrate**: Specifically tuned for 1080p@20fps 4500kb/s streams
- **Grid Display**: 2x2 grid layout showing all 4 cameras simultaneously

## Requirements

- Python 3.7+
- OpenCV (opencv-python)
- NumPy
- RTSP camera streams

## Installation

1. Clone or download the files to your directory
2. Install dependencies:
```bash
pip install opencv-python numpy
```

## Configuration

Edit the camera URLs in `camaras_advanced.py` or `config.ini`:

```python
camera_configs = {
    "1": "rtsp://admin:admin@192.168.0.4:8554/profile0",
    "2": "rtsp://admin:admin@192.168.0.5:8554/profile0", 
    "3": "rtsp://admin:admin@192.168.0.6:8554/profile0",
    "4": "rtsp://admin:admin@192.168.0.7:8554/profile0"
}
```

## Usage

### Quick Start
```bash
./run_cameras.sh
```

### Manual Start
```bash
python3 camaras_advanced.py
```

### Controls
- **'q' or ESC**: Quit application
- **'r'**: Restart failed cameras
- **'s'**: Show detailed statistics

## File Description

- `camaras.py`: Basic multi-threaded RTSP viewer
- `camaras_advanced.py`: Advanced version with optimizations for high bitrate streams
- `config.ini`: Configuration file for camera URLs and settings
- `run_cameras.sh`: Launcher script with dependency checking
- `requirements.txt`: Python package dependencies

## Performance Optimizations

### For High Bitrate Streams (4500kb/s)
- **Minimal Buffer Size**: `CAP_PROP_BUFFERSIZE = 1` for lowest latency
- **Frame Dropping**: Queue size of 1 to always get the latest frame
- **Immediate Resizing**: Frames resized immediately after capture to reduce memory
- **Backend Selection**: Automatic selection of best available OpenCV backend
- **Thread Isolation**: Each camera in separate thread prevents blocking

### Memory Optimization
- Frames resized immediately after capture
- Old frames dropped automatically
- Minimal queue sizes
- Efficient numpy array handling

### CPU Optimization
- Optimized frame processing
- Selective overlay rendering
- Efficient grid layout creation
- Configurable display FPS limiting

## Troubleshooting

### Camera Connection Issues
- Check RTSP URL format and credentials
- Verify network connectivity to cameras
- Use 'r' key to restart failed cameras
- Check firewall settings

### Performance Issues
- Reduce display resolution in config
- Lower max_display_fps setting
- Check CPU usage with `htop`
- Ensure sufficient network bandwidth

### High CPU Usage
- Reduce single_camera_width/height in config
- Lower target FPS
- Check for hardware acceleration availability

## Network Requirements

For 4 cameras at 4500kb/s each:
- **Total Bandwidth**: ~18 Mbps
- **Recommended**: 25+ Mbps available bandwidth
- **Latency**: <100ms for optimal performance

## Hardware Acceleration

The application attempts to use hardware acceleration when available:
- Intel Quick Sync Video (QSV)
- NVIDIA NVDEC/NVENC
- AMD VCE

## Monitoring

Real-time monitoring includes:
- Individual camera FPS
- Dropped frame count and percentage
- Connection health status
- Display FPS
- Automatic failure detection

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Camera 1      │    │   Camera 2      │    │   Camera 3      │
│   Thread        │    │   Thread        │    │   Thread        │
│                 │    │                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │Frame Queue  │ │    │ │Frame Queue  │ │    │ │Frame Queue  │ │
│ │(Size: 1)    │ │    │ │(Size: 1)    │ │    │ │(Size: 1)    │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │  Main Display   │
                    │     Thread      │
                    │                 │
                    │  ┌───┬───┐      │
                    │  │ 1 │ 2 │      │
                    │  ├───┼───┤      │
                    │  │ 3 │ 4 │      │
                    │  └───┴───┘      │
                    └─────────────────┘
```

## License

This project is open source. Feel free to modify and distribute as needed.

## Support

For issues or questions:
1. Check the troubleshooting section
2. Verify camera RTSP URLs are correct
3. Ensure network connectivity
4. Check system resources (CPU, memory, network)
