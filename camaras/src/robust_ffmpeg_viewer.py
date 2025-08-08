#!/usr/bin/env python3
"""
Robust FFmpeg Ultra-Low Latency Camera Viewer
Optimized for stability with 4 cameras, then performance
"""

import ffmpeg
import cv2
import threading
import queue
import time
import numpy as np
import subprocess
import logging
import os
import sys
import configparser
import select
import errno
import math
import json
import traceback
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum

# Constants for magic numbers
class Constants:
    """Application constants"""
    # Frame queue settings
    FRAME_QUEUE_SIZE = 1
    MIN_FRAME_BUFFER_SIZE = 2
    
    # Timing constants (seconds)
    DEFAULT_CONNECTION_TIMEOUT = 30.0
    DEFAULT_EOF_RETRY_DELAY = 1.5
    DEFAULT_HEALTH_CHECK_INTERVAL = 5.0

    STAGGERED_STARTUP_DELAY = 2.0
    INITIAL_CONNECTION_WAIT = 3.0
    PROCESS_TERMINATION_WAIT = 0.5
    
    # Retry and failure limits
    DEFAULT_MAX_CONSECUTIVE_FAILURES = 8
    DEFAULT_FRAME_TIMEOUT = 5.0
    DEFAULT_HEALTH_CHECK_FAILURES = 3
    
    # Performance constants
    ULTRA_LOW_LATENCY_WAIT = 1  # milliseconds
    ERROR_RECOVERY_DELAY = 1.0  # seconds
    
    # Display constants
    MIN_CAMERA_WIDTH = 320
    MIN_CAMERA_HEIGHT = 180
    DEFAULT_ASPECT_RATIO = 16/9
    MAX_WINDOW_WIDTH = 1920
    MAX_WINDOW_HEIGHT = 1080
    FULLSCREEN_WIDTH = 1920
    FULLSCREEN_HEIGHT = 1080
    
    # Cache optimization
    WINDOW_SIZE_CACHE_DURATION = 1.0  # seconds
    
    # Quality switching
    QUALITY_HIGH_RES = "high-res"
    QUALITY_LOW_RES = "low-res"
    
    # Rotation constants
    ROTATION_DEGREES = [0, 90, 180, 270]
    ROTATION_STEPS = 4
    
    # Hardware acceleration constants
    HW_ACCEL_NONE = "none"
    HW_ACCEL_VAAPI = "vaapi"
    HW_ACCEL_NVENC = "nvenc"
    HW_ACCEL_QSV = "qsv"
    HW_ACCEL_CUDA = "cuda"
    
    # Default hardware acceleration device paths
    DEFAULT_VAAPI_DEVICE = "/dev/dri/renderD128"
    DEFAULT_NVIDIA_DEVICE = "/dev/nvidia0"

class HardwareAccelerationConfig:
    """Hardware acceleration configuration and detection"""
    
    def __init__(self):
        self.hw_accel_type = Constants.HW_ACCEL_NONE
        self.hw_accel_device = None
        self.hw_accel_available = False
        self.hw_accel_encoder = None
        self.hw_accel_decoder = None
        self.detected_capabilities = {}
        
        # Try to detect and enable hardware acceleration by default
        self.detect_hardware_acceleration()
        
    def detect_hardware_acceleration(self) -> bool:
        """Detect available hardware acceleration capabilities"""
        logger.info("Detecting hardware acceleration capabilities...")
        
        # Check for VAAPI (Intel/AMD)
        if self._check_vaapi():
            logger.info("VAAPI hardware acceleration detected")
            self.detected_capabilities['vaapi'] = True
        
        # Check for NVIDIA NVENC/CUDA
        if self._check_nvidia():
            logger.info("NVIDIA hardware acceleration detected")
            self.detected_capabilities['nvidia'] = True
        
        # Check for Intel Quick Sync Video
        if self._check_qsv():
            logger.info("Intel Quick Sync Video detected")
            self.detected_capabilities['qsv'] = True
        
        # Set default if any hardware acceleration is available
        if self.detected_capabilities:
            self.hw_accel_available = True
            # Prefer VAAPI as it's most compatible
            if 'vaapi' in self.detected_capabilities:
                self.hw_accel_type = Constants.HW_ACCEL_VAAPI
                self.hw_accel_device = Constants.DEFAULT_VAAPI_DEVICE
                self.hw_accel_decoder = "h264_vaapi"
                self.hw_accel_encoder = "h264_vaapi"
            elif 'nvidia' in self.detected_capabilities:
                self.hw_accel_type = Constants.HW_ACCEL_NVENC
                self.hw_accel_device = Constants.DEFAULT_NVIDIA_DEVICE
                self.hw_accel_decoder = "h264_cuvid"
                self.hw_accel_encoder = "h264_nvenc"
            elif 'qsv' in self.detected_capabilities:
                self.hw_accel_type = Constants.HW_ACCEL_QSV
                self.hw_accel_decoder = "h264_qsv"
                self.hw_accel_encoder = "h264_qsv"
            
            logger.info(f"Selected hardware acceleration: {self.hw_accel_type}")
            return True
        else:
            logger.info("No hardware acceleration detected, using software decoding")
            return False
    
    def _check_vaapi(self) -> bool:
        """Check for VAAPI support"""
        try:
            # Check if VAAPI device exists
            if not os.path.exists(Constants.DEFAULT_VAAPI_DEVICE):
                return False
            
            # Check if FFmpeg supports VAAPI
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-hwaccels'], 
                capture_output=True, text=True, timeout=10
            )
            
            if result.returncode == 0 and 'vaapi' in result.stdout.lower():
                # Test VAAPI decoding capability
                test_result = subprocess.run([
                    'ffmpeg', '-hide_banner', '-f', 'lavfi', '-i', 'testsrc=duration=1:size=320x240:rate=1',
                    '-vaapi_device', Constants.DEFAULT_VAAPI_DEVICE,
                    '-vf', 'format=nv12,hwupload',
                    '-c:v', 'h264_vaapi',
                    '-t', '1',
                    '-f', 'null', '-'
                ], capture_output=True, timeout=15)
                
                return test_result.returncode == 0
        except Exception as e:
            logger.debug(f"VAAPI detection error: {e}")
        
        return False
    
    def _check_nvidia(self) -> bool:
        """Check for NVIDIA hardware acceleration support"""
        try:
            # Check if nvidia-smi exists and works
            result = subprocess.run(['nvidia-smi'], capture_output=True, timeout=10)
            if result.returncode != 0:
                return False
            
            # Check if FFmpeg supports NVENC
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-encoders'], 
                capture_output=True, text=True, timeout=10
            )
            
            if result.returncode == 0 and 'nvenc' in result.stdout.lower():
                # Test NVENC encoding capability
                test_result = subprocess.run([
                    'ffmpeg', '-hide_banner', '-f', 'lavfi', '-i', 'testsrc=duration=1:size=320x240:rate=1',
                    '-c:v', 'h264_nvenc',
                    '-t', '1',
                    '-f', 'null', '-'
                ], capture_output=True, timeout=15)
                
                return test_result.returncode == 0
        except Exception as e:
            logger.debug(f"NVIDIA detection error: {e}")
        
        return False
    
    def _check_qsv(self) -> bool:
        """Check for Intel Quick Sync Video support"""
        try:
            # Check if FFmpeg supports QSV
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-hwaccels'], 
                capture_output=True, text=True, timeout=10
            )
            
            if result.returncode == 0 and 'qsv' in result.stdout.lower():
                # Test QSV decoding capability
                test_result = subprocess.run([
                    'ffmpeg', '-hide_banner', '-f', 'lavfi', '-i', 'testsrc=duration=1:size=320x240:rate=1',
                    '-c:v', 'h264_qsv',
                    '-t', '1',
                    '-f', 'null', '-'
                ], capture_output=True, timeout=15)
                
                return test_result.returncode == 0
        except Exception as e:
            logger.debug(f"QSV detection error: {e}")
        
        return False
    
    def configure_from_config(self, config: configparser.ConfigParser) -> None:
        """Configure hardware acceleration from config file"""
        # Default to enabled - hardware acceleration should be the default behavior
        hw_accel_enabled = True
        
        if 'performance' in config:
            # Check if hardware acceleration is explicitly disabled in config
            hw_accel_enabled = config.getboolean('performance', 'enable_hardware_acceleration', fallback=True)
            # Get device path from config
            hw_accel_device = config.get('performance', 'hw_accel_device', fallback=Constants.DEFAULT_VAAPI_DEVICE)
        else:
            # No performance section - use defaults
            hw_accel_device = Constants.DEFAULT_VAAPI_DEVICE
        
        if not hw_accel_enabled:
            logger.info("Hardware acceleration explicitly disabled in config")
            self.hw_accel_type = Constants.HW_ACCEL_NONE
            return
        
        # Auto-detect hardware acceleration capabilities
        if not self.detect_hardware_acceleration():
            logger.warning("Hardware acceleration requested but not available, falling back to software")
            self.hw_accel_type = Constants.HW_ACCEL_NONE
            return
        
        # Override with config device if specified and different from default
        if hw_accel_device != Constants.DEFAULT_VAAPI_DEVICE:
            self.hw_accel_device = hw_accel_device
            logger.info(f"Using configured hardware acceleration device: {hw_accel_device}")
        
        logger.info(f"Hardware acceleration enabled by default: {self.hw_accel_type}")
    
    def get_ffmpeg_input_args(self) -> dict:
        """Get FFmpeg input arguments for hardware acceleration"""
        args = {}
        
        if self.hw_accel_type == Constants.HW_ACCEL_VAAPI and self.hw_accel_device:
            args['hwaccel'] = 'vaapi'
            args['hwaccel_device'] = self.hw_accel_device
            args['hwaccel_output_format'] = 'vaapi'
        elif self.hw_accel_type == Constants.HW_ACCEL_NVENC:
            args['hwaccel'] = 'cuda'
            args['hwaccel_output_format'] = 'cuda'
        elif self.hw_accel_type == Constants.HW_ACCEL_QSV:
            args['hwaccel'] = 'qsv'
            args['hwaccel_output_format'] = 'qsv'
        
        return args
    
    def get_ffmpeg_filter_args(self) -> dict:
        """Get FFmpeg filter arguments for hardware acceleration"""
        args = {}
        
        if self.hw_accel_type == Constants.HW_ACCEL_VAAPI:
            # VAAPI requires download to system memory for OpenCV
            args['vf'] = 'scale_vaapi=format=nv12,hwdownload,format=nv12'
        elif self.hw_accel_type == Constants.HW_ACCEL_NVENC:
            # CUDA requires download to system memory for OpenCV
            args['vf'] = 'scale_cuda=format=nv12,hwdownload,format=nv12'
        elif self.hw_accel_type == Constants.HW_ACCEL_QSV:
            # QSV requires download to system memory for OpenCV
            args['vf'] = 'scale_qsv=format=nv12,hwdownload,format=nv12'
        
        return args
    
    def is_enabled(self) -> bool:
        """Check if hardware acceleration is enabled and available"""
        return self.hw_accel_available and self.hw_accel_type != Constants.HW_ACCEL_NONE

class ConnectionState(Enum):
    """Connection states for cameras"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    FAILED = "failed"

class ViewMode(Enum):
    """View modes for display"""
    GRID = "grid"
    FULLSCREEN = "fullscreen"
    QUAD_GRID = "quad_grid"

@dataclass
class CameraStats:
    """Camera statistics data structure"""
    fps: float = 0.0
    fps_counter: int = 0
    last_fps_time: float = 0.0
    total_frames: int = 0
    dropped_frames: int = 0
    connection_attempts: int = 0
    successful_connections: int = 0
    eof_errors: int = 0
    consecutive_failures: int = 0
    last_frame_time: float = 0.0

@dataclass
class PerformanceMetrics:
    """Performance metrics for monitoring"""
    timestamp: float
    camera_id: str
    connection_time: float
    frame_processing_time: float
    memory_usage: Optional[float] = None
    cpu_usage: Optional[float] = None

# Configure structured logging with performance metrics
class PerformanceLogger:
    """Enhanced logger with performance metrics"""
    
    def __init__(self, logger_name: str = __name__):
        self.logger = logging.getLogger(logger_name)
        self.metrics_history: List[PerformanceMetrics] = []
        self.max_metrics_history = 1000
        
    def log_performance(self, metrics: PerformanceMetrics):
        """Log performance metrics"""
        self.metrics_history.append(metrics)
        if len(self.metrics_history) > self.max_metrics_history:
            self.metrics_history.pop(0)
        
        self.logger.debug(f"Performance - Camera {metrics.camera_id}: "
                         f"conn_time={metrics.connection_time:.2f}s, "
                         f"frame_time={metrics.frame_processing_time:.3f}ms")
    
    def get_average_metrics(self, camera_id: str, window_seconds: float = 60.0) -> Optional[Dict[str, float]]:
        """Get average metrics for a camera within time window"""
        current_time = time.time()
        recent_metrics = [m for m in self.metrics_history 
                         if m.camera_id == camera_id and 
                         current_time - m.timestamp <= window_seconds]
        
        if not recent_metrics:
            return None
            
        return {
            'avg_connection_time': sum(m.connection_time for m in recent_metrics) / len(recent_metrics),
            'avg_frame_time': sum(m.frame_processing_time for m in recent_metrics) / len(recent_metrics),
            'metrics_count': len(recent_metrics)
        }
    
    def info(self, message: str):
        """Log info message"""
        self.logger.info(message)
    
    def warning(self, message: str):
        """Log warning message"""
        self.logger.warning(message)
    
    def error(self, message: str):
        """Log error message"""
        self.logger.error(message)
    
    def debug(self, message: str):
        """Log debug message"""
        self.logger.debug(message)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = PerformanceLogger()

class RobustFfmpegCamera:
    """Robust camera with improved error handling and dual stream support"""
    
    def __init__(self, camera_id: str, rtsp_url: str, rtsp_url_lowres: Optional[str] = None, 
                 target_size: Tuple[int, int] = (640, 360), initial_quality: str = Constants.QUALITY_HIGH_RES, 
                 connection_timeout: float = Constants.DEFAULT_CONNECTION_TIMEOUT, 
                 max_consecutive_failures: int = Constants.DEFAULT_MAX_CONSECUTIVE_FAILURES, 
                 eof_retry_delay: float = Constants.DEFAULT_EOF_RETRY_DELAY,
                 health_check_interval: float = Constants.DEFAULT_HEALTH_CHECK_INTERVAL,
                 frame_timeout: float = Constants.DEFAULT_FRAME_TIMEOUT,
                 hw_accel_config: Optional[HardwareAccelerationConfig] = None):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.rtsp_url_lowres = rtsp_url_lowres
        self.target_size = target_size
        
        # Configuration parameters
        self.connection_timeout = connection_timeout
        self.max_consecutive_failures = max_consecutive_failures
        self.eof_retry_delay = eof_retry_delay
        self.health_check_interval = health_check_interval
        self.frame_timeout = frame_timeout
        
        # Hardware acceleration configuration
        self.hw_accel_config = hw_accel_config or HardwareAccelerationConfig()
        
        # Stream quality management
        self._initialize_stream_quality(initial_quality)
        
        # Threading
        self.running = False
        self.thread = None
        self.process = None
        self.thread_stop_event = threading.Event()
        
        # Connection state tracking
        self.connection_state = ConnectionState.DISCONNECTED
        self.connection_start_time = None
        
        # Frame queue with minimal buffer for ultra-low latency
        self.frame_queue = queue.Queue(maxsize=Constants.FRAME_QUEUE_SIZE)
        
        # Last frame caching to prevent flickering
        self.last_frame = None
        self.last_frame_lock = threading.Lock()
        
        # Statistics with thread safety
        self.stats_lock = threading.Lock()
        self.stats = CameraStats()
        self.stats.last_frame_time = time.time()
        self.stats.last_fps_time = time.time()
        
        # Performance tracking
        self.performance_start_time = None
        
        # Frame buffer size calculation
        self.frame_buffer_size = self.target_size[0] * self.target_size[1] * 3  # RGB
        
        quality_info = f"starting with {initial_quality} quality"
        if initial_quality == Constants.QUALITY_LOW_RES and rtsp_url_lowres is None:
            quality_info += " (fallback to high-res - no low-res URL configured)"
        
        hw_accel_info = ""
        if self.hw_accel_config.is_enabled():
            hw_accel_info = f" with {self.hw_accel_config.hw_accel_type} hardware acceleration"
        
        logger.info(f"RobustFfmpegCamera {self.camera_id} initialized - {quality_info}{hw_accel_info}")
        
    def _initialize_stream_quality(self, initial_quality: str) -> None:
        """Initialize stream quality settings"""
        if initial_quality == Constants.QUALITY_LOW_RES and self.rtsp_url_lowres is not None:
            self.current_stream_url = self.rtsp_url_lowres
            self.use_lowres = True
        else:
            self.current_stream_url = self.rtsp_url
            self.use_lowres = False
        
    def start(self) -> None:
        """Start the camera stream"""
        if self._is_already_running():
            logger.warning(f"Camera {self.camera_id} already running")
            return
            
        self._initialize_for_start()
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()
        logger.info(f"Camera {self.camera_id} started")
        
    def _is_already_running(self) -> bool:
        """Check if camera thread is already running"""
        return self.thread and self.thread.is_alive()
    
    def _initialize_for_start(self) -> None:
        """Initialize camera state for starting"""
        self.running = True
        self.thread_stop_event.clear()
        self.connection_state = ConnectionState.DISCONNECTED
        with self.stats_lock:
            self.stats.consecutive_failures = 0
        self.performance_start_time = time.time()
        
    def stop(self) -> None:
        """Stop the camera stream"""
        if not self.running:
            return
            
        logger.info(f"Stopping camera {self.camera_id}")
        self.running = False
        self.thread_stop_event.set()
        self.connection_state = ConnectionState.DISCONNECTED
        
        self._cleanup_process()
        self._wait_for_thread_completion()
        
    def _cleanup_process(self) -> None:
        """Clean up FFmpeg process"""
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                logger.warning(f"Camera {self.camera_id} process didn't terminate gracefully, killing...")
                try:
                    self.process.kill()
                    self.process.wait(timeout=3.0)
                except Exception as e:
                    logger.error(f"Camera {self.camera_id} process couldn't be killed: {e}")
            except Exception as e:
                logger.warning(f"Camera {self.camera_id} process cleanup error: {e}")
            finally:
                self.process = None
                
    def _wait_for_thread_completion(self) -> None:
        """Wait for thread to complete"""
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5.0)
            if self.thread.is_alive():
                logger.warning(f"Camera {self.camera_id} thread didn't stop gracefully")
            
        with self.stats_lock:
            total_frames = self.stats.total_frames
            dropped_frames = self.stats.dropped_frames
            eof_errors = self.stats.eof_errors
            successful_connections = self.stats.successful_connections
            connection_attempts = self.stats.connection_attempts
            
        logger.info(f"Camera {self.camera_id} stopped - {total_frames} frames, {dropped_frames} dropped, {eof_errors} EOF errors, {successful_connections}/{connection_attempts} successful connections")
        
    def _stream_loop(self):
        """Main streaming loop with robust error handling and thread management"""
        
        while self.running and not self.thread_stop_event.is_set():
            try:
                with self.stats_lock:
                    self.stats.connection_attempts += 1
                    consecutive_failures = self.stats.consecutive_failures
                    eof_errors = self.stats.eof_errors
                
                # Progressive backoff with exponential delay for persistent failures
                if consecutive_failures > 0:
                    # Special handling for EOF errors - shorter delay
                    if eof_errors > 0:
                        delay = min(consecutive_failures * self.eof_retry_delay, 8)  # Configurable EOF retry delay
                    else:
                        delay = min(2 ** min(consecutive_failures, 4), 15)  # Exponential backoff
                    
                    logger.info(f"Camera {self.camera_id} reconnecting in {delay}s (failure {consecutive_failures}, EOF errors: {eof_errors})")
                    
                    # Interruptible sleep
                    if self.thread_stop_event.wait(delay):
                        break  # Stop event was set
                
                # Connection timeout handling
                self.connection_state = ConnectionState.CONNECTING
                self.connection_start_time = time.time()
                
                # Ensure clean slate - force kill any existing process
                self._cleanup_process()
                
                # Create FFmpeg process with enhanced error handling
                if not self._create_ffmpeg_process():
                    with self.stats_lock:
                        self.stats.consecutive_failures += 1
                    continue
                
                # Process frames with enhanced error handling
                frame_count = self._process_frames()
                
                # Analyze disconnection reason
                self._analyze_disconnection(frame_count)
                
            except Exception as e:
                with self.stats_lock:
                    self.stats.consecutive_failures += 1
                    consecutive_failures = self.stats.consecutive_failures
                    
                logger.error(f"Camera {self.camera_id} unexpected error in stream loop: {e}")
                self._cleanup_process()
                
                if consecutive_failures > self.max_consecutive_failures:
                    logger.error(f"Camera {self.camera_id} failed {self.max_consecutive_failures} times, giving up")
                    break
                    
        logger.info(f"Camera {self.camera_id} stream loop ended")
        self.connection_state = ConnectionState.DISCONNECTED
        
    def _cleanup_process(self):
        """Clean up the FFmpeg process aggressively"""
        if self.process:
            try:
                # First, try graceful termination
                if self.process.poll() is None:  # Process is still running
                    self.process.terminate()
                    
                    # Wait briefly for graceful termination
                    try:
                        self.process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        # Force kill if it doesn't terminate
                        logger.warning(f"Camera {self.camera_id} process didn't terminate, force killing...")
                        self.process.kill()
                        try:
                            self.process.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            logger.error(f"Camera {self.camera_id} process couldn't be killed")
                            
            except Exception as e:
                logger.warning(f"Camera {self.camera_id} process cleanup error: {e}")
            finally:
                self.process = None
                
        # Clear frame queue and cache
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        
        # Reset connection state
        self.connection_start_time = None
        
        with self.last_frame_lock:
            self.last_frame = None
            
    def _create_ffmpeg_process(self):
        """Create FFmpeg process with adaptive parameters, hardware acceleration, and better error handling"""
        try:
            logger.info(f"Camera {self.camera_id} creating FFmpeg process...")
            
            # Adaptive FFmpeg parameters based on connection attempts
            base_args = {
                'rtsp_transport': 'tcp',
                'fflags': 'nobuffer',
                'max_delay': 0,
            }
            
            # Add hardware acceleration input arguments
            if self.hw_accel_config.is_enabled():
                hw_input_args = self.hw_accel_config.get_ffmpeg_input_args()
                base_args.update(hw_input_args)
                logger.debug(f"Camera {self.camera_id} using hardware acceleration: {self.hw_accel_config.hw_accel_type}")
            
            # Adjust parameters based on connection history
            with self.stats_lock:
                consecutive_failures = self.stats.consecutive_failures
                
            if consecutive_failures > 2:
                # More conservative settings for problematic cameras
                base_args.update({
                    'probesize': 65536,  # 64KB - larger for better detection
                    'analyzeduration': 2000000,  # 2 seconds - more time for analysis
                    'buffer_size': 64000,  # Larger buffer for unstable connections
                    'rtsp_flags': 'prefer_tcp',
                })
            else:
                # Optimized settings for stable cameras
                base_args.update({
                    'probesize': 32768,  # 32KB - minimal for fast startup
                    'analyzeduration': 1000000,  # 1 second - fast analysis
                })
            
            # Add quality-specific optimizations
            if self.use_lowres:
                base_args['threads'] = 1  # Single thread for low-res
            else:
                base_args['threads'] = 2  # Multi-thread for high-res
            
            # Create FFmpeg input
            input_stream = ffmpeg.input(self.current_stream_url, **base_args)
            
            # Apply hardware acceleration filters and scaling
            if self.hw_accel_config.is_enabled():
                try:
                    # Get hardware acceleration filter arguments
                    hw_filter_args = self.hw_accel_config.get_ffmpeg_filter_args()
                    
                    if 'vf' in hw_filter_args:
                        # Use hardware-accelerated scaling with format conversion
                        if self.hw_accel_config.hw_accel_type == Constants.HW_ACCEL_VAAPI:
                            video_stream = input_stream.video.filter(
                                'scale_vaapi', 
                                w=self.target_size[0], 
                                h=self.target_size[1],
                                format='nv12'
                            ).filter('hwdownload').filter('format', 'nv12')
                        elif self.hw_accel_config.hw_accel_type == Constants.HW_ACCEL_NVENC:
                            video_stream = input_stream.video.filter(
                                'scale_cuda', 
                                w=self.target_size[0], 
                                h=self.target_size[1],
                                format='nv12'
                            ).filter('hwdownload').filter('format', 'nv12')
                        elif self.hw_accel_config.hw_accel_type == Constants.HW_ACCEL_QSV:
                            video_stream = input_stream.video.filter(
                                'scale_qsv', 
                                w=self.target_size[0], 
                                h=self.target_size[1],
                                format='nv12'
                            ).filter('hwdownload').filter('format', 'nv12')
                        else:
                            # Fallback to software scaling
                            video_stream = input_stream.video.filter('scale', self.target_size[0], self.target_size[1])
                    else:
                        # Fallback to software scaling
                        video_stream = input_stream.video.filter('scale', self.target_size[0], self.target_size[1])
                        
                    # Convert to RGB format for OpenCV (NV12 -> RGB conversion)
                    video_stream = video_stream.filter('format', 'rgb24')
                    
                except Exception as e:
                    logger.warning(f"Camera {self.camera_id} hardware acceleration failed, falling back to software: {e}")
                    # Fallback to software decoding
                    video_stream = input_stream.video.filter('scale', self.target_size[0], self.target_size[1])
            else:
                # Software decoding and scaling
                video_stream = input_stream.video.filter('scale', self.target_size[0], self.target_size[1])
            
            # Create the FFmpeg process
            process = (
                video_stream
                .output(
                    'pipe:',
                    format='rawvideo',
                    pix_fmt='rgb24',
                    loglevel='error'  # Reduce log noise
                )
                .run_async(
                    pipe_stdout=True, 
                    pipe_stderr=subprocess.PIPE,
                    quiet=False
                )
            )
            
            # Check if process started successfully
            if process.poll() is not None:
                logger.error(f"Camera {self.camera_id} FFmpeg process failed to start")
                
                # If hardware acceleration failed, try software fallback
                if self.hw_accel_config.is_enabled():
                    logger.warning(f"Camera {self.camera_id} retrying with software decoding...")
                    return self._create_ffmpeg_process_software_fallback()
                    
                return False
            
            self.process = process
            self.connection_state = ConnectionState.CONNECTED
            self.connection_start_time = None  # Reset timeout - connection is established
            with self.stats_lock:
                self.stats.successful_connections += 1
                
            accel_info = f" ({self.hw_accel_config.hw_accel_type})" if self.hw_accel_config.is_enabled() else " (software)"
            logger.info(f"Camera {self.camera_id} FFmpeg process started successfully{accel_info}")
            
            return True
            
        except Exception as e:
            logger.error(f"Camera {self.camera_id} failed to create FFmpeg process: {e}")
            
            # If hardware acceleration failed, try software fallback
            if self.hw_accel_config.is_enabled():
                logger.warning(f"Camera {self.camera_id} retrying with software decoding...")
                return self._create_ffmpeg_process_software_fallback()
                
            return False
    
    def _create_ffmpeg_process_software_fallback(self):
        """Create FFmpeg process with software decoding as fallback"""
        try:
            logger.info(f"Camera {self.camera_id} creating software fallback FFmpeg process...")
            
            # Basic software decoding parameters
            base_args = {
                'rtsp_transport': 'tcp',
                'fflags': 'nobuffer',
                'max_delay': 0,
            }
            
            # Adjust parameters based on connection history
            with self.stats_lock:
                consecutive_failures = self.stats.consecutive_failures
                
            if consecutive_failures > 2:
                base_args.update({
                    'probesize': 65536,
                    'analyzeduration': 2000000,
                    'buffer_size': 64000,
                    'rtsp_flags': 'prefer_tcp',
                })
            else:
                base_args.update({
                    'probesize': 32768,
                    'analyzeduration': 1000000,
                })
            
            # Add quality-specific optimizations
            if self.use_lowres:
                base_args['threads'] = 1
            else:
                base_args['threads'] = 2
            
            # Create the software-only FFmpeg process
            process = (
                ffmpeg
                .input(self.current_stream_url, **base_args)
                .video
                .filter('scale', self.target_size[0], self.target_size[1])
                .output(
                    'pipe:',
                    format='rawvideo',
                    pix_fmt='rgb24',
                    loglevel='error'
                )
                .run_async(
                    pipe_stdout=True, 
                    pipe_stderr=subprocess.PIPE,
                    quiet=False
                )
            )
            
            # Check if process started successfully
            if process.poll() is not None:
                logger.error(f"Camera {self.camera_id} software fallback FFmpeg process failed to start")
                return False
            
            self.process = process
            self.connection_state = ConnectionState.CONNECTED
            self.connection_start_time = None
            with self.stats_lock:
                self.stats.successful_connections += 1
            
            logger.info(f"Camera {self.camera_id} software fallback FFmpeg process started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Camera {self.camera_id} software fallback FFmpeg process failed: {e}")
            return False
            
    def _process_frames(self):
        """Process frames with enhanced error handling"""
        frame_count = 0
        last_frame_time = time.time()
        connection_check_interval = 5.0  # Check connection every 5 seconds
        
        while self.running and not self.thread_stop_event.is_set():
            # Check connection timeout (only during initial connection phase)
            current_time = time.time()
            if (self.connection_start_time is not None and 
                self.connection_state == ConnectionState.CONNECTING and 
                (current_time - self.connection_start_time) > self.connection_timeout):
                logger.warning(f"Camera {self.camera_id} connection timeout after {self.connection_timeout}s")
                break
                
            # Check if process is still alive
            if self.process is None or self.process.poll() is not None:
                logger.warning(f"Camera {self.camera_id} process terminated unexpectedly")
                break
                
            # Check for frame timeout
            if frame_count > 0 and (current_time - last_frame_time) > 10.0:
                logger.warning(f"Camera {self.camera_id} no frames received for 10 seconds")
                break
                
            try:
                # Use select to avoid blocking reads with timeout
                ready, _, _ = select.select([self.process.stdout], [], [], 1.0)
                
                if not ready:
                    continue  # Timeout, try again
                    
                # Read frame data
                try:
                    raw_frame = self.process.stdout.read(self.frame_buffer_size)
                except (OSError, IOError) as e:
                    logger.warning(f"Camera {self.camera_id} read error: {e}")
                    break
                
                if len(raw_frame) == 0:
                    logger.warning(f"Camera {self.camera_id} received EOF")
                    with self.stats_lock:
                        self.stats.eof_errors += 1
                    break
                    
                if len(raw_frame) != self.frame_buffer_size:
                    # Skip partial frames
                    continue
                    
                # Process frame
                if self._process_single_frame(raw_frame):
                    frame_count += 1
                    last_frame_time = current_time
                    
                    # Reset consecutive failures on successful frame processing
                    if frame_count == 1:
                        with self.stats_lock:
                            self.stats.consecutive_failures = 0
                    
                    if frame_count % 100 == 0:
                        with self.stats_lock:
                            current_fps = self.stats.fps
                        logger.debug(f"Camera {self.camera_id} processed {frame_count} frames, FPS: {current_fps}")
                        
            except Exception as e:
                logger.warning(f"Camera {self.camera_id} frame processing error: {e}")
                if frame_count == 0:
                    break  # No frames received, likely connection issue
                continue
                
        return frame_count
        
    def _process_single_frame(self, raw_frame):
        """Process a single frame with error handling and optimized memory usage"""
        try:
            # Convert to numpy array - make it writable for OpenCV
            frame = np.frombuffer(raw_frame, dtype=np.uint8).copy()  # .copy() makes it writable
            frame = frame.reshape((self.target_size[1], self.target_size[0], 3))
            
            # Convert RGB to BGR for OpenCV (in-place when possible)
            cv2.cvtColor(frame, cv2.COLOR_RGB2BGR, dst=frame)
            
            # Add overlay (currently disabled for performance)
            # self._add_overlay(frame)
            
            # Update statistics with thread safety
            self._update_frame_stats()
            self._update_fps()
            
            # Ultra-low latency: aggressive frame dropping
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                    with self.stats_lock:
                        self.stats.dropped_frames += 1
                except queue.Empty:
                    break
                    
            # Add new frame
            try:
                self.frame_queue.put_nowait(frame)
                # Cache the latest frame to prevent flickering (avoid copy)
                with self.last_frame_lock:
                    self.last_frame = frame
            except queue.Full:
                # Force drop and replace for ultra-low latency
                try:
                    self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait(frame)
                    with self.stats_lock:
                        self.stats.dropped_frames += 1
                    with self.last_frame_lock:
                        self.last_frame = frame
                except queue.Empty:
                    pass
                    
            return True
            
        except Exception as e:
            logger.warning(f"Camera {self.camera_id} frame conversion error: {e}")
            return False
            
    def _analyze_disconnection(self, frame_count):
        """Analyze why the connection was lost and adjust strategy"""
        if not self.running:
            return
            
        return_code = self.process.poll() if self.process else -1
        stderr_output = ""
        
        if self.process and self.process.stderr:
            try:
                stderr_data = self.process.stderr.read()
                if stderr_data:
                    stderr_output = stderr_data.decode('utf-8', errors='ignore')
            except (OSError, IOError, UnicodeDecodeError) as e:
                logger.debug(f"Camera {self.camera_id} stderr read error: {e}")
                pass
        
        # Analyze the failure
        if frame_count > 50:  # If we got a good number of frames, reset failure count
            with self.stats_lock:
                self.stats.consecutive_failures = 0
            logger.info(f"Camera {self.camera_id} had good connection ({frame_count} frames), resetting failure count")
        elif frame_count > 0:
            # Partial success - don't increment failure count as much
            logger.info(f"Camera {self.camera_id} partial success ({frame_count} frames)")
        else:
            # Complete failure
            with self.stats_lock:
                self.stats.consecutive_failures += 1
            logger.warning(f"Camera {self.camera_id} complete failure (0 frames)")
        
        # Log detailed error information
        if stderr_output:
            if "Connection refused" in stderr_output:
                logger.error(f"Camera {self.camera_id} connection refused - check camera availability")
            elif "Connection timed out" in stderr_output:
                logger.error(f"Camera {self.camera_id} connection timed out - check network")
            elif "EOF" in stderr_output or return_code == 255:
                logger.error(f"Camera {self.camera_id} EOF error (code: {return_code}) - stream ended unexpectedly")
                with self.stats_lock:
                    self.stats.eof_errors += 1
            else:
                logger.warning(f"Camera {self.camera_id} error (code: {return_code}): {stderr_output[:300]}...")
        
        # Clear state for next attempt
        self.connection_state = ConnectionState.DISCONNECTED
        self.connection_start_time = None  # Reset connection timeout
                    
        logger.info(f"Camera {self.camera_id} stream loop ended")
        
    def _update_fps(self) -> None:
        """Update FPS counter with thread safety"""
        with self.stats_lock:
            self.stats.fps_counter += 1
            current_time = time.time()
            if current_time - self.stats.last_fps_time >= 1.0:
                self.stats.fps = self.stats.fps_counter
                self.stats.fps_counter = 0
                self.stats.last_fps_time = current_time
                
    def _update_frame_stats(self) -> None:
        """Update frame statistics"""
        with self.stats_lock:
            self.stats.total_frames += 1
            self.stats.last_frame_time = time.time()
            
    def _log_performance_metrics(self, frame_start_time: float) -> None:
        """Log performance metrics"""
        if self.performance_start_time is None:
            return
            
        current_time = time.time()
        connection_time = current_time - self.performance_start_time
        frame_processing_time = (current_time - frame_start_time) * 1000  # Convert to milliseconds
        
        metrics = PerformanceMetrics(
            timestamp=current_time,
            camera_id=self.camera_id,
            connection_time=connection_time,
            frame_processing_time=frame_processing_time
        )
        
        logger.log_performance(metrics)
            
    def _add_overlay(self, frame):
        """Add informational overlay (disabled for ultra-low latency)"""
        # For ultra-low latency, skip overlays to save processing time
        # Uncomment below if you want overlays back
        pass
        
        # Original overlay code (commented for performance):
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # font_scale = 0.4
        # color = (0, 255, 0)
        # thickness = 1
        # 
        # # Create semi-transparent background
        # overlay = frame.copy()
        # cv2.rectangle(overlay, (5, 5), (180, 70), (0, 0, 0), -1)
        # frame[:] = cv2.addWeighted(frame, 0.8, overlay, 0.2, 0)
        # 
        # # Add text
        # texts = [
        #     f"Cam {self.camera_id}",
        #     f"FPS: {self.fps}",
        #     f"Frames: {self.total_frames}",
        #     f"Dropped: {self.dropped_frames}"
        # ]
        # 
        # for i, text in enumerate(texts):
        #     y = 18 + i * 12
        #     cv2.putText(frame, text, (8, y), font, font_scale, color, thickness)
            
    def get_frame(self):
        """Get latest frame with fallback to prevent flickering and timeout protection"""
        try:
            # Try to get the newest frame (already non-blocking)
            frame = self.frame_queue.get_nowait()
            # Update cached frame (avoid unnecessary copy)
            with self.last_frame_lock:
                self.last_frame = frame
            return frame
        except queue.Empty:
            # Return cached frame to prevent flickering
            with self.last_frame_lock:
                if self.last_frame is not None:
                    return self.last_frame  # Return reference, not copy
            return None
        except Exception as e:
            logger.debug(f"Camera {self.camera_id} get_frame error: {e}")
            # Return cached frame as fallback
            with self.last_frame_lock:
                if self.last_frame is not None:
                    return self.last_frame
            return None
            
    def restart(self):
        """Restart the camera stream by destroying and recreating the thread"""
        logger.info(f"Camera {self.camera_id} manual restart requested")
        
        # Stop the current thread and process
        was_running = self.running
        self.stop()
        
        # Reset error counters for fresh start
        with self.stats_lock:
            self.stats.consecutive_failures = 0
            self.stats.eof_errors = 0
        
        # Wait a moment to ensure cleanup
        time.sleep(0.5)
        
        # Restart if it was running before
        if was_running:
            self.start()
            
        logger.info(f"Camera {self.camera_id} restart completed")
    
    def _should_switch_quality(self) -> bool:
        """Determine if we should switch quality based on performance metrics"""
        if self.rtsp_url_lowres is None:
            return False
            
        with self.stats_lock:
            consecutive_failures = self.stats.consecutive_failures
            eof_errors = self.stats.eof_errors
            
        # Switch to low-res if having connection issues
        if consecutive_failures >= 3 and not self.use_lowres:
            logger.info(f"Camera {self.camera_id} switching to low-res due to connection issues")
            return True
            
        # Switch back to high-res if stable for a while
        if consecutive_failures == 0 and eof_errors < 2 and self.use_lowres:
            logger.info(f"Camera {self.camera_id} switching back to high-res - connection stable")
            return True
            
        return False
    
    def _adaptive_quality_check(self) -> None:
        """Check if quality should be switched adaptively"""
        if self._should_switch_quality():
            self.switch_stream_quality(not self.use_lowres)
            
    def switch_stream_quality(self, use_lowres: bool) -> None:
        """Switch between high-res and low-res streams"""
        if self.rtsp_url_lowres is None:
            logger.warning(f"Camera {self.camera_id} has no low-res stream configured")
            return
            
        if use_lowres == self.use_lowres:
            return  # Already using requested quality
            
        self.use_lowres = use_lowres
        new_url = self.rtsp_url_lowres if use_lowres else self.rtsp_url
        
        if new_url != self.current_stream_url:
            self.current_stream_url = new_url
            quality_name = Constants.QUALITY_LOW_RES if use_lowres else Constants.QUALITY_HIGH_RES
            logger.info(f"Camera {self.camera_id} switching to {quality_name} stream")
            
            # Restart stream with new URL
            self.restart()
    
    def get_current_stream_quality(self) -> str:
        """Get current stream quality"""
        return Constants.QUALITY_LOW_RES if self.use_lowres else Constants.QUALITY_HIGH_RES
    
    def is_healthy(self):
        """Check if camera is healthy with enhanced state tracking"""
        if not self.running:
            return False
        if self.connection_state == ConnectionState.FAILED:
            return False
        if not self.process:
            return False
        if self.process.poll() is not None:  # Process has terminated
            return False
        
        # Thread-safe access to last_frame_time
        with self.stats_lock:
            last_frame_time = self.stats.last_frame_time
            consecutive_failures = self.stats.consecutive_failures
        
        if time.time() - last_frame_time > self.frame_timeout:
            return False
        if consecutive_failures > Constants.DEFAULT_HEALTH_CHECK_FAILURES:
            return False
        return True
        
    def get_connection_state(self) -> ConnectionState:
        """Get current connection state"""
        return self.connection_state
    
    def get_stats_dict(self) -> Dict[str, Any]:
        """Get comprehensive statistics dictionary"""
        with self.stats_lock:
            return {
                'fps': self.stats.fps,
                'total_frames': self.stats.total_frames,
                'dropped_frames': self.stats.dropped_frames,
                'connection_attempts': self.stats.connection_attempts,
                'successful_connections': self.stats.successful_connections,
                'eof_errors': self.stats.eof_errors,
                'consecutive_failures': self.stats.consecutive_failures,
                'last_frame_time': self.stats.last_frame_time,
                'stream_quality': self.get_current_stream_quality(),
                'connection_state': self.connection_state.value,
                'is_healthy': self.is_healthy()
            }
    
    def get_error_stats(self) -> Dict[str, Any]:
        """Get error statistics for health monitoring"""
        with self.stats_lock:
            return {
                'eof_errors': self.stats.eof_errors,
                'consecutive_failures': self.stats.consecutive_failures,
                'dropped_frames': self.stats.dropped_frames,
                'connection_attempts': self.stats.connection_attempts,
                'successful_connections': self.stats.successful_connections,
                'total_connections': self.stats.connection_attempts,
                'last_error_time': getattr(self.stats, 'last_error_time', 0),
                'is_healthy': self.is_healthy()
            }
    
    def get_performance_metrics(self) -> Optional[Dict[str, float]]:
        """Get performance metrics for this camera"""
        try:
            # Get recent metrics from the performance logger
            recent_metrics = logger.get_average_metrics(self.camera_id, window_seconds=30.0)
            return recent_metrics
        except Exception as e:
            logger.debug(f"Error getting performance metrics for camera {self.camera_id}: {e}")
            return None

class RobustViewer:
    """Robust viewer for multiple cameras (1-8, auto-scaling)"""
    
    def __init__(self, config_file: str = "config.ini"):
        self.config = configparser.ConfigParser()
        self.config.read(config_file)
        
        # Initialize hardware acceleration (enabled by default)
        self.hw_accel_config = HardwareAccelerationConfig()
        self.hw_accel_config.configure_from_config(self.config)
        
        # Log hardware acceleration status
        if self.hw_accel_config.is_enabled():
            logger.info(f"Hardware acceleration enabled: {self.hw_accel_config.hw_accel_type}")
        else:
            logger.info("Hardware acceleration disabled - using software decoding")
        
        # Load camera configurations
        self.cameras: Dict[str, RobustFfmpegCamera] = {}
        self.camera_configs = self._load_camera_configs()
        
        # Load initial quality setting from config
        self.initial_quality = self.config.get('performance', 'initial_quality_mode', fallback=Constants.QUALITY_HIGH_RES)
        # Validate the setting
        if self.initial_quality not in [Constants.QUALITY_HIGH_RES, Constants.QUALITY_LOW_RES]:
            logger.warning(f"Invalid initial_quality_mode '{self.initial_quality}', defaulting to '{Constants.QUALITY_HIGH_RES}'")
            self.initial_quality = Constants.QUALITY_HIGH_RES
            
        # Load error handling configuration with constants
        self.connection_timeout = self.config.getfloat('performance', 'connection_timeout', fallback=Constants.DEFAULT_CONNECTION_TIMEOUT)
        self.max_consecutive_failures = self.config.getint('performance', 'max_consecutive_failures', fallback=Constants.DEFAULT_MAX_CONSECUTIVE_FAILURES)
        self.eof_retry_delay = self.config.getfloat('performance', 'eof_retry_delay', fallback=Constants.DEFAULT_EOF_RETRY_DELAY)
        self.health_check_interval = self.config.getfloat('performance', 'health_check_interval', fallback=Constants.DEFAULT_HEALTH_CHECK_INTERVAL)
        
        # Camera rotation state (0=0°, 1=90°, 2=180°, 3=270°)
        self.camera_rotations: Dict[str, int] = {camera_id: 0 for camera_id in self.camera_configs.keys()}
        
        # File to store camera settings persistence (rotations and quad positions)
        self.settings_file = "camera_settings.json"
        
        # Load saved camera settings (rotations and quad positions)
        self._load_camera_settings()
        
        # Calculate optimal grid layout and sizing
        self.num_cameras = len(self.camera_configs)
        self.grid_rows, self.grid_cols = self._calculate_grid_layout(self.num_cameras)
        self.single_cam_size = self._calculate_camera_size(self.grid_rows, self.grid_cols)
        self.window_size = self._calculate_window_size(self.grid_rows, self.grid_cols, self.single_cam_size)
        
        # View mode management
        self.view_mode = ViewMode.GRID
        self.fullscreen_camera_index = 0  # Index of camera in fullscreen mode
        self.camera_id_list = sorted(self.camera_configs.keys(), key=lambda x: int(x))  # Sorted list for navigation
        
        # 2x2 Quad Grid layout management
        # Map quad positions to camera indices: top-left, top-right, bottom-left, bottom-right
        # Default layout: top_left=cam6, top_right=cam3, bottom_left=cam5, bottom_right=cam1
        self.quad_positions = {
            'top_left': 0,      # Numpad 7 - will be set to correct camera index
            'top_right': 1,     # Numpad 9 - will be set to correct camera index
            'bottom_left': 2,   # Numpad 1 - will be set to correct camera index
            'bottom_right': 3   # Numpad 3 - will be set to correct camera index
        }
        
        # Set initial quad positions to desired cameras (if they exist)
        self._set_initial_quad_positions()
        
        self.running = False
        # Use a fixed window name to avoid window recreation issues
        self.window_name = "Ultra-Low Latency Camera Viewer"
        
        # Cache for display optimization
        self.cached_window_size = None
        self.cached_camera_sizes = None
        self.last_window_check = 0
        
        logger.info(f"RobustViewer initialized with {self.num_cameras} cameras in {self.grid_rows}x{self.grid_cols} grid")
        logger.info(f"Camera size: {self.single_cam_size}, Window size: {self.window_size}")
        logger.info(f"Initial quality mode: {self.initial_quality}")
        
        # Log hardware acceleration status
        if self.hw_accel_config.is_enabled():
            logger.info(f"Hardware acceleration enabled: {self.hw_accel_config.hw_accel_type}")
            if self.hw_accel_config.hw_accel_device:
                logger.info(f"Hardware acceleration device: {self.hw_accel_config.hw_accel_device}")
        else:
            logger.info("Hardware acceleration disabled - using software decoding")
        
        # Log loaded rotations
        rotated_cameras = [f"Camera {cam_id}: {rot * 90}°" for cam_id, rot in self.camera_rotations.items() if rot != 0]
        if rotated_cameras:
            logger.info(f"Loaded camera rotations: {', '.join(rotated_cameras)}")
        
    def _load_camera_settings(self):
        """Load camera settings (rotations and quad positions) from file"""
        try:
            # First try the new settings file
            if os.path.exists(self.settings_file):
                with open(self.settings_file, 'r') as f:
                    saved_settings = json.load(f)
                    
                # Load rotations
                if 'rotations' in saved_settings:
                    for camera_id, rotation in saved_settings['rotations'].items():
                        if camera_id in self.camera_rotations:
                            # Validate rotation value
                            if isinstance(rotation, int) and 0 <= rotation <= 3:
                                self.camera_rotations[camera_id] = rotation
                                logger.debug(f"Loaded rotation for Camera {camera_id}: {rotation * 90}°")
                            else:
                                logger.warning(f"Invalid rotation value for Camera {camera_id}: {rotation}")
                
                # Load quad positions
                if 'quad_positions' in saved_settings:
                    for position, camera_id in saved_settings['quad_positions'].items():
                        if position in self.quad_positions:
                            # Convert camera_id to index in camera_id_list
                            try:
                                camera_index = self.camera_id_list.index(str(camera_id))
                                self.quad_positions[position] = camera_index
                                logger.debug(f"Loaded quad position {position}: Camera {camera_id}")
                            except ValueError:
                                logger.warning(f"Camera {camera_id} not found for quad position {position}")
                
                logger.info(f"Camera settings loaded from {self.settings_file}")
            
            # Fallback: try to load from old rotation file
            elif os.path.exists("camera_rotations.json"):
                with open("camera_rotations.json", 'r') as f:
                    saved_rotations = json.load(f)
                    
                # Apply saved rotations only for cameras that exist in current config
                for camera_id, rotation in saved_rotations.items():
                    if camera_id in self.camera_rotations:
                        # Validate rotation value
                        if isinstance(rotation, int) and 0 <= rotation <= 3:
                            self.camera_rotations[camera_id] = rotation
                            logger.debug(f"Loaded rotation for Camera {camera_id}: {rotation * 90}°")
                        else:
                            logger.warning(f"Invalid rotation value for Camera {camera_id}: {rotation}")
                            
                logger.info(f"Camera rotations loaded from legacy file camera_rotations.json")
            else:
                logger.debug(f"No settings file found, using defaults")
                
        except Exception as e:
            logger.warning(f"Failed to load camera settings: {e}")
            # Continue with default settings
            
    def _save_camera_settings(self):
        """Save camera settings (rotations and quad positions) to file"""
        try:
            # Convert quad positions from indices back to camera IDs
            quad_positions_by_camera_id = {}
            for position, camera_index in self.quad_positions.items():
                if camera_index < len(self.camera_id_list):
                    camera_id = self.camera_id_list[camera_index]
                    quad_positions_by_camera_id[position] = camera_id
            
            settings = {
                'rotations': self.camera_rotations,
                'quad_positions': quad_positions_by_camera_id
            }
            
            with open(self.settings_file, 'w') as f:
                json.dump(settings, f, indent=2)
            logger.debug(f"Camera settings saved to {self.settings_file}")
        except Exception as e:
            logger.warning(f"Failed to save camera settings: {e}")
        
    def _load_camera_configs(self):
        """Load camera configurations from config file"""
        configs = {}
        if 'cameras' in self.config:
            for key, value in self.config['cameras'].items():
                if key.startswith('camera') and key.endswith('_url') and not key.endswith('_lowres'):
                    camera_num = key.replace('camera', '').replace('_url', '')
                    
                    # Load high-res URL
                    configs[camera_num] = {'url': value}
                    
                    # Look for corresponding low-res URL
                    lowres_key = f'camera{camera_num}_url_lowres'
                    if lowres_key in self.config['cameras']:
                        configs[camera_num]['url_lowres'] = self.config['cameras'][lowres_key]
                    else:
                        configs[camera_num]['url_lowres'] = None
                        
        return configs
    
    def _calculate_grid_layout(self, num_cameras):
        """Calculate optimal grid layout for given number of cameras"""
        if num_cameras <= 0:
            return 1, 1
        elif num_cameras == 1:
            return 1, 1
        elif num_cameras == 2:
            return 1, 2
        elif num_cameras <= 4:
            return 2, 2
        elif num_cameras <= 6:
            return 2, 3
        elif num_cameras <= 8:
            return 2, 4
        else:
            # For more than 8 cameras, calculate square-ish grid
            rows = int(np.ceil(np.sqrt(num_cameras)))
            cols = int(np.ceil(num_cameras / rows))
            return rows, cols
    
    def _calculate_camera_size(self, rows: int, cols: int, window_width: Optional[int] = None, window_height: Optional[int] = None) -> Tuple[int, int]:
        """Calculate individual camera frame size based on grid layout and actual window size"""
        # Use actual window size if provided, otherwise use defaults
        if window_width is None or window_height is None:
            max_window_width = Constants.MAX_WINDOW_WIDTH
            max_window_height = Constants.MAX_WINDOW_HEIGHT
        else:
            max_window_width = window_width
            max_window_height = window_height
        
        # Calculate camera size to fit within window
        cam_width = max_window_width // cols
        cam_height = max_window_height // rows
        
        # Maintain 16:9 aspect ratio, prefer smaller dimension
        if cam_width / cam_height > Constants.DEFAULT_ASPECT_RATIO:
            cam_width = int(cam_height * Constants.DEFAULT_ASPECT_RATIO)
        else:
            cam_height = int(cam_width * (9/16))
        
        # Ensure minimum size for readability
        cam_width = max(Constants.MIN_CAMERA_WIDTH, cam_width)
        cam_height = max(Constants.MIN_CAMERA_HEIGHT, cam_height)
        
        return (cam_width, cam_height)
    
    def _calculate_window_size(self, rows: int, cols: int, cam_size: Tuple[int, int]) -> Tuple[int, int]:
        """Calculate total window size"""
        width = cols * cam_size[0]
        height = rows * cam_size[1]
        return (width, height)
        
    def start(self):
        """Start the viewer"""
        logger.info(f"Starting scalable {self.num_cameras}-camera viewer...")
        
        # Check FFmpeg availability
        try:
            result = subprocess.run(['ffmpeg', '-version'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                raise Exception("FFmpeg returned non-zero exit code")
        except Exception as e:
            logger.error(f"FFmpeg not available: {e}")
            return False
            
        # Initialize cameras with staggered startup
        for i, (camera_id, camera_config) in enumerate(self.camera_configs.items()):
            logger.info(f"Initializing camera {camera_id}...")
            rtsp_url = camera_config['url']
            rtsp_url_lowres = camera_config['url_lowres']
            camera = RobustFfmpegCamera(
                camera_id, rtsp_url, rtsp_url_lowres, self.single_cam_size, self.initial_quality,
                self.connection_timeout, self.max_consecutive_failures, self.eof_retry_delay,
                self.health_check_interval, Constants.DEFAULT_FRAME_TIMEOUT, self.hw_accel_config
            )
            self.cameras[camera_id] = camera
            camera.start()
            
            # Stagger startup minimally for ultra-low latency
            if i < len(self.camera_configs) - 1:
                logger.info(f"Waiting {Constants.STAGGERED_STARTUP_DELAY} seconds before starting next camera...")
                time.sleep(Constants.STAGGERED_STARTUP_DELAY)
                
        # Reduce waiting time for ultra-low latency
        logger.info("Waiting for cameras to establish connections...")
        time.sleep(Constants.INITIAL_CONNECTION_WAIT)
        
        # Start display loop
        self.running = True
        self._display_loop()
        
        return True
        
    def stop(self):
        """Stop the viewer"""
        logger.info("Stopping robust viewer...")
        self.running = False
        
        # Save camera settings before stopping
        self._save_camera_settings()
        
        for camera in self.cameras.values():
            camera.stop()
            
        cv2.destroyAllWindows()
        
    def _display_loop(self):
        """Main display loop"""
        # Create window without decorations - use WINDOW_NORMAL and then set properties
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # Remove window decorations and make it fullscreen-like
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        logger.info("=== Controls ===")
        logger.info("ESC or 'q' - Quit")
        logger.info("'h' - Show health status")
        logger.info("'H' - Show hardware acceleration status")
        logger.info("'P' or 'p' - Restart all cameras")
        logger.info("SPACE - Toggle Grid/2x2 Quad Grid/Fullscreen view")
        logger.info("LEFT/RIGHT arrows - Switch camera in fullscreen")
        logger.info("'R' or 'r' - Rotate current camera 90° clockwise (fullscreen only)")
        logger.info("'T' or 't' - Reset all camera rotations to 0°")
        logger.info("'L' or 'l' - Switch all cameras between high-res/low-res streams")
        logger.info("Numpad 7/9/1/3 - Switch cameras in 2x2 quad grid (top-left/top-right/bottom-left/bottom-right)")
        logger.info("==================")
        
        last_health_check = time.time()
        
        while self.running:
            try:
                # Collect frames from all cameras with timeout protection
                frames = {}
                healthy_cameras = 0
                
                for camera_id, camera in self.cameras.items():
                    try:
                        # Add timeout protection to prevent blocking
                        frame = camera.get_frame()
                        if frame is not None:
                            frames[camera_id] = frame
                        
                        if camera.is_healthy():
                            healthy_cameras += 1
                    except Exception as e:
                        logger.debug(f"Error getting frame from camera {camera_id}: {e}")
                        # Skip this camera for this frame, don't let it block the UI
                
                # Periodic health check and auto-restart with timeout protection
                current_time = time.time()
                if current_time - last_health_check > self.health_check_interval:
                    for camera_id, camera in self.cameras.items():
                        try:
                            if not camera.is_healthy():
                                logger.warning(f"Camera {camera_id} unhealthy, triggering restart")
                                camera.restart()
                            # Also perform adaptive quality check
                            camera._adaptive_quality_check()
                        except Exception as e:
                            logger.debug(f"Error during health check for camera {camera_id}: {e}")
                    last_health_check = current_time
                        
                # Create display with error protection
                display_frame = None
                try:
                    display_frame = self._create_display(frames, healthy_cameras)
                except Exception as e:
                    logger.error(f"Error creating display: {e}")
                    # Create a fallback display
                    display_frame = self._create_error_display(f"Display Error: {str(e)}")
                
                if display_frame is not None:
                    cv2.imshow(self.window_name, display_frame)
                    
                # Handle keyboard input with minimal wait for ultra-low latency
                key = cv2.waitKey(Constants.ULTRA_LOW_LATENCY_WAIT) & 0xFF
                
                # Debug: Log key presses (remove this line in production)
                if key != 255:  # 255 means no key pressed
                    logger.debug(f"Key pressed: {key} (char: {chr(key) if 32 <= key <= 126 else 'N/A'})")
                
                if key == ord('q') or key == 27:  # ESC
                    break
                elif key == ord('h'):
                    # Run health display in a separate thread to avoid blocking
                    health_thread = threading.Thread(target=self._show_health, daemon=True)
                    health_thread.start()
                elif key == ord('H'):  # Shift+H for hardware acceleration status
                    # Run hardware acceleration status display in a separate thread
                    hw_status_thread = threading.Thread(target=self._show_hardware_acceleration_status, daemon=True)
                    hw_status_thread.start()
                elif key == ord('P') or key == ord('p'):  # P or p for restart all cameras
                    logger.info("Manual restart of all cameras requested")
                    for camera_id, camera in self.cameras.items():
                        logger.info(f"Restarting camera {camera_id}")
                        camera.restart()
                elif key == ord('R') or key == ord('r'):  # R or r for rotation
                    logger.info("Rotation key pressed")
                    self._rotate_current_camera()
                elif key == ord('T') or key == ord('t'):  # T or t for reset all rotations
                    self._reset_all_rotations()
                elif key == ord(' '):  # SPACE - Toggle view mode
                    self._toggle_view_mode()
                elif key in [81, 2, 63234]:  # LEFT arrow (different codes on different systems)
                    self._previous_camera()
                elif key in [83, 3, 63235]:  # RIGHT arrow (different codes on different systems)
                    self._next_camera()
                elif key == ord('L') or key == ord('l'):  # L or l for quality switch
                    logger.info("Stream quality switch key pressed")
                    self._switch_stream_quality()
                # Numpad controls for 2x2 quad grid
                elif key in [55, 151]:  # Numpad 7 (different codes on different systems)
                    self._set_quad_position('top_left')
                elif key in [57, 153]:  # Numpad 9 (different codes on different systems)
                    self._set_quad_position('top_right')
                elif key in [49, 145]:  # Numpad 1 (different codes on different systems)
                    self._set_quad_position('bottom_left')
                elif key in [51, 147]:  # Numpad 3 (different codes on different systems)
                    self._set_quad_position('bottom_right')
                    
            except KeyboardInterrupt:
                logger.info("Interrupted by user")
                break
            except Exception as e:
                logger.error(f"Display loop error: {e}")
                time.sleep(Constants.ERROR_RECOVERY_DELAY)  # Prevent rapid error loops
                
        self.stop()
        
    def _toggle_view_mode(self):
        """Toggle between grid, quad grid, and fullscreen view"""
        if self.view_mode == ViewMode.GRID:
            self.view_mode = ViewMode.QUAD_GRID
            logger.info(f"Switched to 2x2 quad grid mode")
        elif self.view_mode == ViewMode.QUAD_GRID:
            self.view_mode = ViewMode.FULLSCREEN
            current_camera = self.camera_id_list[self.fullscreen_camera_index]
            logger.info(f"Switched to fullscreen mode - Camera {current_camera}")
        else:
            self.view_mode = ViewMode.GRID
            logger.info(f"Switched to grid mode - {self.num_cameras} cameras")
        
        # Invalidate display cache when view mode changes
        self.cached_window_size = None
        self.cached_camera_sizes = None
    
    def _next_camera(self):
        """Switch to next camera in fullscreen mode"""
        if self.view_mode == ViewMode.FULLSCREEN:
            self.fullscreen_camera_index = (self.fullscreen_camera_index + 1) % len(self.camera_id_list)
            current_camera = self.camera_id_list[self.fullscreen_camera_index]
            logger.info(f"Switched to Camera {current_camera}")
    
    def _previous_camera(self):
        """Switch to previous camera in fullscreen mode"""
        if self.view_mode == ViewMode.FULLSCREEN:
            self.fullscreen_camera_index = (self.fullscreen_camera_index - 1) % len(self.camera_id_list)
            current_camera = self.camera_id_list[self.fullscreen_camera_index]
            logger.info(f"Switched to Camera {current_camera}")
    
    def _set_quad_position(self, position: str):
        """Set camera for a specific quad position"""
        if position in self.quad_positions:
            # Cycle to next camera for this position
            current_index = self.quad_positions[position]
            next_index = (current_index + 1) % len(self.camera_id_list)
            self.quad_positions[position] = next_index
            
            camera_id = self.camera_id_list[next_index]
            position_name = position.replace('_', ' ').title()
            logger.info(f"Set {position_name} to Camera {camera_id}")
            
            # Save settings immediately
            self._save_camera_settings()
            
            # If not already in quad grid mode, switch to it
            if self.view_mode != ViewMode.QUAD_GRID:
                self.view_mode = ViewMode.QUAD_GRID
                logger.info("Switched to 2x2 quad grid mode")
        
    def _create_display(self, frames, healthy_cameras):
        """Create display based on current view mode"""
        if self.view_mode == ViewMode.FULLSCREEN:
            return self._create_fullscreen_display(frames)
        elif self.view_mode == ViewMode.QUAD_GRID:
            return self._create_quad_grid_display(frames)
        else:
            return self._create_grid_display(frames, healthy_cameras)
    
    def _create_quad_grid_display(self, frames):
        """Create 2x2 quad grid display for 4 cameras"""
        # Fixed size for 2x2 grid
        quad_width = Constants.FULLSCREEN_WIDTH // 2
        quad_height = Constants.FULLSCREEN_HEIGHT // 2
        
        # Create the main display frame
        display = np.zeros((Constants.FULLSCREEN_HEIGHT, Constants.FULLSCREEN_WIDTH, 3), dtype=np.uint8)
        
        # Position mapping for 2x2 grid
        positions = {
            'top_left': (0, 0),
            'top_right': (0, quad_width),
            'bottom_left': (quad_height, 0),
            'bottom_right': (quad_height, quad_width)
        }
        
        # Place cameras in each position
        for position, (y, x) in positions.items():
            camera_index = self.quad_positions[position]
            
            # Get camera ID and frame
            if camera_index < len(self.camera_id_list):
                camera_id = self.camera_id_list[camera_index]
                
                if camera_id in frames and frames[camera_id] is not None:
                    frame = frames[camera_id]
                    
                    try:
                        # Apply rotation
                        frame = self._rotate_frame(frame, camera_id)
                        
                        # Skip if frame is None after rotation
                        if frame is None:
                            self._draw_quad_placeholder(display, x, y, quad_width, quad_height, 
                                                      camera_id, position)
                            continue
                        
                        # Resize frame to fit quad
                        resized_frame = cv2.resize(frame, (quad_width, quad_height))
                        
                        # Place frame in position
                        display[y:y+quad_height, x:x+quad_width] = resized_frame
                        
                        # Add camera label
                        label = f"Camera {camera_id}"
                        label_pos = (x + 10, y + 30)
                        cv2.putText(display, label, label_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                                   1.0, (0, 255, 0), 2)
                        
                        # Add position indicator
                        position_name = position.replace('_', ' ').title()
                        numpad_key = self._get_numpad_key_for_position(position)
                        position_label = f"{position_name} (Numpad {numpad_key})"
                        position_pos = (x + 10, y + quad_height - 20)
                        cv2.putText(display, position_label, position_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.6, (255, 255, 255), 1)
                    except Exception as e:
                        logger.debug(f"Error processing frame for camera {camera_id}: {e}")
                        # Show placeholder on error
                        self._draw_quad_placeholder(display, x, y, quad_width, quad_height, 
                                                  camera_id, position)
                else:
                    # No frame available - show placeholder
                    self._draw_quad_placeholder(display, x, y, quad_width, quad_height, 
                                              camera_id, position)
            else:
                # No camera available for this position
                self._draw_quad_placeholder(display, x, y, quad_width, quad_height, 
                                          "N/A", position)
        
        # Add instructions
        instructions = [
            "2x2 Quad Grid Mode",
            "Numpad 7/9/1/3 - Switch cameras",
            "SPACE - Toggle view mode",
            "R - Rotate camera (in fullscreen only)"
        ]
        
        for i, instruction in enumerate(instructions):
            y_pos = 30 + i * 25
            cv2.putText(display, instruction, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 255), 1)
        
        return display
    
    def _get_numpad_key_for_position(self, position: str) -> str:
        """Get numpad key for position"""
        key_map = {
            'top_left': '7',
            'top_right': '9',
            'bottom_left': '1',
            'bottom_right': '3'
        }
        return key_map.get(position, '?')
    
    def _draw_quad_placeholder(self, display, x, y, width, height, camera_id, position):
        """Draw placeholder for quad position"""
        # Fill with dark gray
        display[y:y+height, x:x+width] = (40, 40, 40)
        
        # Add border
        cv2.rectangle(display, (x, y), (x+width, y+height), (100, 100, 100), 2)
        
        # Add text
        font_scale = 1.5
        thickness = 2
        color = (200, 200, 200)
        
        # Camera ID
        camera_text = f"Camera {camera_id}"
        camera_size = cv2.getTextSize(camera_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
        camera_pos = (x + (width - camera_size[0]) // 2, y + height // 2 - 20)
        cv2.putText(display, camera_text, camera_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale, color, thickness)
        
        # Status
        status_text = "NO SIGNAL"
        status_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, thickness)[0]
        status_pos = (x + (width - status_size[0]) // 2, y + height // 2 + 30)
        cv2.putText(display, status_text, status_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale * 0.8, (0, 0, 255), thickness)
        
        # Position and numpad key
        position_name = position.replace('_', ' ').title()
        numpad_key = self._get_numpad_key_for_position(position)
        position_text = f"{position_name} (Numpad {numpad_key})"
        position_pos = (x + 10, y + height - 20)
        cv2.putText(display, position_text, position_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (255, 255, 255), 1)

    def _create_fullscreen_placeholder(self, camera_id: str, width: int, height: int, status: str = "NO SIGNAL"):
        """Create fullscreen placeholder for unavailable camera"""
        display = np.zeros((height, width, 3), dtype=np.uint8)
        
       
        
        # Add border
        cv2.rectangle(display, (10, 10), (width - 10, height - 10), (100, 100, 100), 5)
        
        # Add text
        font_scale = 3.0
        thickness = 5
        color = (0, 0, 255)  # Red for no signal, yellow for errors
        
        # Camera ID
        camera_text = f"Camera {camera_id}"
        camera_size = cv2.getTextSize(camera_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
        camera_pos = ((width - camera_size[0]) // 2, (height - camera_size[1]) // 2 - 60)
        cv2.putText(display, camera_text, camera_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale, (255, 255, 255), thickness)
        
        # Status
        status_size = cv2.getTextSize(status, cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, thickness)[0]
        status_pos = ((width - status_size[0]) // 2, (height - status_size[1]) // 2 + 60)
        cv2.putText(display, status, status_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale * 0.8, color, thickness)
        
        # Navigation hint
        hint = "Use LEFT/RIGHT arrows to switch cameras, SPACE for view mode, R to rotate, T to reset rotations, L to switch stream quality"
        cv2.putText(display, hint, (30, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return display

    def _create_fullscreen_display(self, frames):
        """Create fullscreen display for single camera"""
        if not self.camera_id_list:
            # No cameras available
            blank = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(blank, "No cameras available", (400, 350), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
            return blank
        
        # Get current camera
        current_camera_id = self.camera_id_list[self.fullscreen_camera_index]
        
        # Use a large size for fullscreen to maximize camera display
        fullscreen_width = Constants.FULLSCREEN_WIDTH
        fullscreen_height = Constants.FULLSCREEN_HEIGHT
        
        # Try to get actual window size, but ensure minimum fullscreen size
        try:
            actual_window_size = self._get_actual_window_size()
            fullscreen_width = max(fullscreen_width, actual_window_size[0])
            fullscreen_height = max(fullscreen_height, actual_window_size[1])
        except Exception as e:
            logger.debug(f"Could not get actual window size: {e}")
        
        if current_camera_id in frames and frames[current_camera_id] is not None:
            # Resize frame to fill window while maintaining aspect ratio
            frame = frames[current_camera_id]
            
            try:
                # Apply rotation first
                frame = self._rotate_frame(frame, current_camera_id)
                
                # Skip if frame is None after rotation
                if frame is None:
                    return self._create_fullscreen_placeholder(current_camera_id, fullscreen_width, fullscreen_height, "FRAME ERROR")
                
                # Calculate scaling to fit window - use maximum scaling possible
                frame_h, frame_w = frame.shape[:2]
                
                # Scale to fit window while maintaining aspect ratio
                scale_w = fullscreen_width / frame_w
                scale_h = fullscreen_height / frame_h
                scale = min(scale_w, scale_h)
                
                # Make sure we're scaling up, not down
                scale = max(scale, 1.0)
                
                new_w = int(frame_w * scale)
                new_h = int(frame_h * scale)
                
                # Resize frame
                resized_frame = cv2.resize(frame, (new_w, new_h))
                
                # Create black background and center the frame
                display = np.zeros((fullscreen_height, fullscreen_width, 3), dtype=np.uint8)
                y_offset = (fullscreen_height - new_h) // 2
                x_offset = (fullscreen_width - new_w) // 2
                display[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized_frame
                
                # Add camera info overlay
                font_scale = 1.5  # Larger font for fullscreen
                thickness = 3
                color = (0, 255, 0)
                
                # Camera title
                title = f"Camera {current_camera_id} ({self.fullscreen_camera_index + 1}/{len(self.camera_id_list)})"
                cv2.putText(display, title, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
                
                # Navigation hint
                hint = "Use LEFT/RIGHT arrows to switch cameras, SPACE for view mode, R to rotate, T to reset all rotations, L to switch stream quality"
                cv2.putText(display, hint, (30, fullscreen_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                return display
                
            except Exception as e:
                logger.debug(f"Error processing fullscreen frame for camera {current_camera_id}: {e}")
                return self._create_fullscreen_placeholder(current_camera_id, fullscreen_width, fullscreen_height, "PROCESSING ERROR")
        else:
            # Camera not available - show placeholder
            if current_camera_id in self.cameras:
                camera = self.cameras[current_camera_id]
                if camera.is_healthy():
                    status = "BUFFERING..."
                else:
                    status = "CONNECTING..."
            else:
                status = "OFFLINE"
            
            return self._create_fullscreen_placeholder(current_camera_id, fullscreen_width, fullscreen_height, status)
            
    def _create_grid_display(self, frames, healthy_cameras):
        """Create scalable grid display for any number of cameras with caching"""
        # Get sorted camera IDs for consistent positioning
        camera_ids = sorted(self.camera_configs.keys(), key=lambda x: int(x))
        total_positions = self.grid_rows * self.grid_cols
        
        # Get actual window size with caching
        actual_window_size = self._get_actual_window_size()
        target_width, target_height = actual_window_size
        
        # Cache camera sizes to avoid recalculation
        cache_key = (target_width, target_height, self.grid_rows, self.grid_cols)
        if self.cached_camera_sizes is None or self.cached_camera_sizes[0] != cache_key:
            # Calculate optimal camera size to fill the window completely
            cam_width = target_width // self.grid_cols
            cam_height = target_height // self.grid_rows
            
            # Use the calculated size directly without forcing 16:9 aspect ratio
            # This allows cameras to fill their allocated space completely
            dynamic_cam_size = (cam_width, cam_height)
            self.cached_camera_sizes = (cache_key, dynamic_cam_size)
            logger.debug(f"Cached new camera sizes: {dynamic_cam_size}")
        else:
            dynamic_cam_size = self.cached_camera_sizes[1]
        
        # Create grid frames
        grid_frames = []
        
        for i in range(total_positions):
            if i < len(camera_ids):
                camera_id = camera_ids[i]
                
                if camera_id in frames and frames[camera_id] is not None:
                    # Apply rotation first, then resize frame to fill the entire allocated space
                    frame = self._rotate_frame(frames[camera_id], camera_id)
                    frame = cv2.resize(frame, dynamic_cam_size)
                    grid_frames.append(frame)
                else:
                    # Create placeholder
                    placeholder = np.zeros((dynamic_cam_size[1], dynamic_cam_size[0], 3), dtype=np.uint8)
                    
                    if camera_id in self.cameras:
                        camera = self.cameras[camera_id]
                        if camera.is_healthy():
                            status = "BUFFERING..."
                            color = (0, 255, 255)  # Yellow for buffering
                        else:
                            status = "CONNECTING..."
                            color = (0, 255, 0)  # Green for connecting
                    else:
                        status = "OFFLINE"
                        color = (0, 0, 255)  # Red for offline
                    
                    # Scale text based on camera size
                    font_scale = max(0.5, min(1.2, dynamic_cam_size[0] / 640))
                    thickness = max(1, int(font_scale * 2))
                    
                    # Center text positioning
                    text_x = dynamic_cam_size[0] // 4
                    text_y = dynamic_cam_size[1] // 2 - 20
                    status_y = dynamic_cam_size[1] // 2 + 20;
                    
                    cv2.putText(placeholder, f"Camera {camera_id}", (text_x, text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
                    cv2.putText(placeholder, status, (text_x + 20, status_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, color, thickness)
                    grid_frames.append(placeholder)
            else:
                # Empty position - create black frame
                empty_frame = np.zeros((dynamic_cam_size[1], dynamic_cam_size[0], 3), dtype=np.uint8)
                grid_frames.append(empty_frame)
        
        # Arrange frames in grid
        rows = []
        for row in range(self.grid_rows):
            row_frames = []
            for col in range(self.grid_cols):
                idx = row * self.grid_cols + col
                if idx < len(grid_frames):
                    row_frames.append(grid_frames[idx])
                else:
                    # Shouldn't happen, but safety fallback
                    empty_frame = np.zeros((dynamic_cam_size[1], dynamic_cam_size[0], 3), dtype=np.uint8)
                    row_frames.append(empty_frame)
            
            # Horizontally stack frames in this row
            if row_frames:
                row_image = np.hstack(row_frames)
                rows.append(row_image)
        
        # Vertically stack all rows to create the final grid
        if rows:
            final_display = np.vstack(rows)
            
            # Ensure the final display matches the target window size exactly
            if final_display.shape[:2] != (target_height, target_width):
                final_display = cv2.resize(final_display, (target_width, target_height))
            
            # Add grid mode indicator
            font_scale = 0.8
            thickness = 1
            color = (255, 255, 255)
            hint = "Press SPACE for fullscreen mode, L to switch stream quality"
            cv2.putText(final_display, hint, (10, target_height - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
            
            return final_display
        else:
            # Fallback - shouldn't happen
            return np.zeros((target_height, target_width, 3), dtype=np.uint8)
        
    def _show_hardware_acceleration_status(self):
        """Show hardware acceleration status"""
        try:
            logger.info("=== Hardware Acceleration Status ===")
            
            if self.hw_accel_config.is_enabled():
                logger.info(f"Hardware acceleration: ENABLED ({self.hw_accel_config.hw_accel_type})")
                if self.hw_accel_config.hw_accel_device:
                    logger.info(f"Device: {self.hw_accel_config.hw_accel_device}")
                if self.hw_accel_config.hw_accel_decoder:
                    logger.info(f"Decoder: {self.hw_accel_config.hw_accel_decoder}")
                if self.hw_accel_config.hw_accel_encoder:
                    logger.info(f"Encoder: {self.hw_accel_config.hw_accel_encoder}")
                    
                # Show detected capabilities
                if self.hw_accel_config.detected_capabilities:
                    caps = ", ".join(self.hw_accel_config.detected_capabilities.keys())
                    logger.info(f"Detected capabilities: {caps}")
            else:
                logger.info("Hardware acceleration: DISABLED")
                logger.info("Using software decoding for all cameras")
                
            logger.info("=====================================")
            
        except Exception as e:
            logger.error(f"Error showing hardware acceleration status: {e}")
            
    def _show_health(self) -> None:
        """Show camera health status with enhanced information - non-blocking"""
        try:
            logger.info("=== Health Status ===")
            
            # Show hardware acceleration status first
            hw_status = "ENABLED" if self.hw_accel_config.is_enabled() else "DISABLED"
            hw_type = f" ({self.hw_accel_config.hw_accel_type})" if self.hw_accel_config.is_enabled() else ""
            logger.info(f"Hardware acceleration: {hw_status}{hw_type}")
            
            for camera_id, camera in self.cameras.items():
                try:
                    stats_dict = camera.get_stats_dict()
                    error_stats = camera.get_error_stats()
                    
                    status = "HEALTHY" if stats_dict.get('is_healthy', False) else "UNHEALTHY"
                    quality = stats_dict.get('stream_quality', 'N/A')
                    state = stats_dict.get('connection_state', 'N/A')
                    
                    # Show hardware acceleration status per camera
                    hw_info = ""
                    if camera.hw_accel_config.is_enabled():
                        hw_info = f" [HW: {camera.hw_accel_config.hw_accel_type}]"
                    else:
                        hw_info = " [SW]"
                    
                    logger.info(f"Camera {camera_id}: {status} ({quality}) - State: {state}{hw_info}")
                    
                    consecutive_failures = error_stats.get('consecutive_failures', 0)
                    eof_errors = error_stats.get('eof_errors', 0)
                    if consecutive_failures > 0:
                        logger.info(f"  Failures: {consecutive_failures}, EOF: {eof_errors}")
                        
                    # Show last frame time for debugging
                    last_frame_time = stats_dict.get('last_frame_time', 0)
                    if last_frame_time > 0:
                        time_since_last_frame = time.time() - last_frame_time
                        logger.info(f"  Last frame: {time_since_last_frame:.1f}s ago")
                    else:
                        logger.info(f"  Last frame: No frames received yet")
                        
                except Exception as e:
                    logger.error(f"Error getting health info for camera {camera_id}: {e}")
                    logger.info(f"Camera {camera_id}: Health check failed")
                    
            logger.info("=====================")
        except Exception as e:
            logger.error(f"Error in _show_health: {e}")
            logger.info("=== Health Check Error ===")
            logger.info("Unable to display camera health due to error")
            logger.info("=============================")
    
    def _get_actual_window_size(self):
        """Get the actual window size from OpenCV with caching"""
        current_time = time.time()
        
        # Cache window size to avoid excessive system calls
        if (self.cached_window_size is not None and 
            current_time - self.last_window_check < Constants.WINDOW_SIZE_CACHE_DURATION):
            return self.cached_window_size
        
        self.last_window_check = current_time
        
        try:
            # Try multiple methods to get window size
            # Method 1: Get window image rect
            rect = cv2.getWindowImageRect(self.window_name)
            if rect[2] > 0 and rect[3] > 0:
                logger.debug(f"Window size from getWindowImageRect: {rect[2]}x{rect[3]}")
                # If window is maximized or very large, use it
                if rect[2] > 1600 or rect[3] > 900:
                    self.cached_window_size = (rect[2], rect[3])
                    return self.cached_window_size
        except Exception as e:
            logger.debug(f"getWindowImageRect failed: {e}")
        
        # Try to detect if window might be maximized by checking common screen resolutions
        try:
            # Get screen resolution using xrandr (Linux)
            result = subprocess.run(['xrandr'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if '*' in line and '+' in line:  # Current resolution marker
                        parts = line.split()
                        for part in parts:
                            if 'x' in part and part.replace('x', '').replace('.', '').isdigit():
                                width, height = map(int, part.split('x')[0:2])
                                if width >= 1920 and height >= 1080:
                                    logger.debug(f"Detected screen resolution: {width}x{height}")
                                    self.cached_window_size = (width, height)
                                    return self.cached_window_size
        except:
            pass
        
        # For fullscreen mode, use a larger default size
        if self.view_mode == ViewMode.FULLSCREEN:
            # Use a much larger size for fullscreen
            fullscreen_size = (Constants.FULLSCREEN_WIDTH, Constants.FULLSCREEN_HEIGHT)
            logger.debug(f"Using fullscreen default size: {fullscreen_size}")
            self.cached_window_size = fullscreen_size
            return self.cached_window_size
        
        # For grid mode, also use generous sizing when window seems large
        # Use at least Full HD for better scaling
        default_size = (Constants.MAX_WINDOW_WIDTH, Constants.MAX_WINDOW_HEIGHT)
        logger.debug(f"Using default large size: {default_size}")
        self.cached_window_size = default_size
        return self.cached_window_size
    
    def _update_camera_sizes_for_window(self):
        """Update camera sizes based on current window size"""
        # For grid mode, try to get actual window size
        try:
            actual_size = self._get_actual_window_size()
            # Use a larger size for better quality
            grid_width = max(Constants.MAX_WINDOW_WIDTH, actual_size[0])
            grid_height = max(Constants.MAX_WINDOW_HEIGHT, actual_size[1])
            
            # Recalculate camera sizes for the actual window
            new_cam_size = self._calculate_camera_size(
                self.grid_rows, self.grid_cols, grid_width, grid_height
            )
            logger.debug(f"Dynamic camera size: {new_cam_size}, window size: {grid_width}x{grid_height}")
            return new_cam_size, (grid_width, grid_height)
        except:
            # Fallback to larger default sizes
            default_width = 1920
            default_height = 1080
            new_cam_size = self._calculate_camera_size(
                self.grid_rows, self.grid_cols, default_width, default_height
            )
            return new_cam_size, (default_width, default_height)

    def _rotate_frame(self, frame, camera_id):
        """Rotate frame based on camera rotation state"""
        if frame is None:
            return None
            
        if camera_id not in self.camera_rotations:
            return frame
            
        rotation = self.camera_rotations[camera_id]
        
        if rotation == 0:
            return frame
        elif rotation == 1:  # 90° clockwise
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotation == 2:  # 180°
            return cv2.rotate(frame, cv2.ROTATE_180)
        elif rotation == 3:  # 270° clockwise (90° counter-clockwise)
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            return frame
    
    def _create_error_display(self, error_message: str):
        """Create error display when main display fails"""
        display = np.zeros((720, 1280, 3), dtype=np.uint8)
        
        # Add error message
        font_scale = 1.5
        thickness = 2
        color = (0, 0, 255)  # Red
        
        # Center the error message
        text_size = cv2.getTextSize(error_message, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
        text_x = (1280 - text_size[0]) // 2
        text_y = (720 - text_size[1]) // 2
        
        cv2.putText(display, error_message, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale, color, thickness)
        
        # Add instructions
        instructions = [
            "Application Error - Display system failed",
            "Try pressing 'q' to quit and restart",
            "Check logs for detailed error information"
        ]
        
        for i, instruction in enumerate(instructions):
            y_pos = text_y + 60 + i * 30
            cv2.putText(display, instruction, (50, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.8, (255, 255, 255), 1)
        
        return display
    
    def _rotate_current_camera(self):
        """Rotate the current camera in fullscreen mode"""
        if self.view_mode == ViewMode.FULLSCREEN and self.camera_id_list:
            current_camera_id = self.camera_id_list[self.fullscreen_camera_index]
            old_rotation = self.camera_rotations[current_camera_id]
            self.camera_rotations[current_camera_id] = (self.camera_rotations[current_camera_id] + 1) % Constants.ROTATION_STEPS
            rotation_degrees = self.camera_rotations[current_camera_id] * 90
            logger.info(f"Rotated Camera {current_camera_id} from {old_rotation * 90}° to {rotation_degrees}°")
            
            # Save settings immediately
            self._save_camera_settings()
        else:
            if self.view_mode == ViewMode.GRID:
                logger.info("Camera rotation only works in fullscreen mode. Press SPACE to switch to fullscreen first.")
            else:
                logger.info("No cameras available for rotation.")
    
    def _reset_all_rotations(self):
        """Reset all camera rotations to 0 degrees"""
        reset_count = 0
        for camera_id in self.camera_rotations:
            if self.camera_rotations[camera_id] != 0:
                self.camera_rotations[camera_id] = 0
                reset_count += 1
        
        if reset_count > 0:
            logger.info(f"Reset rotations for {reset_count} cameras to 0°")
            # Save settings immediately
            self._save_camera_settings()
        else:
            logger.info("All cameras already at 0° rotation")
        
    def _switch_stream_quality(self):
        """Switch all cameras between high-res and low-res streams"""
        if not self.cameras:
            logger.warning("No cameras available to switch stream quality")
            return
        
        # Check if any camera has low-res stream available
        has_lowres = any(camera.rtsp_url_lowres is not None for camera in self.cameras.values())
        if not has_lowres:
            logger.warning("No cameras have low-res streams configured")
            return
        
        # Determine current quality by checking first camera
        first_camera = next(iter(self.cameras.values()))
        current_quality = first_camera.get_current_stream_quality()
        
        # Switch to opposite quality
        use_lowres = current_quality == Constants.QUALITY_HIGH_RES
        new_quality = "low-res" if use_lowres else "high-res"
        
        logger.info(f"Switching all cameras to {new_quality} streams")
        
        # Switch all cameras
        for camera_id, camera in self.cameras.items():
            if camera.rtsp_url_lowres is not None:
                camera.switch_stream_quality(use_lowres)
            else:
                logger.warning(f"Camera {camera_id} has no low-res stream configured, skipping")
        
        logger.info(f"All cameras switched to {new_quality} streams")
        
    def _set_initial_quad_positions(self):
        """Set initial quad positions to desired cameras: top_left=cam6, top_right=cam3, bottom_left=cam5, bottom_right=cam1"""
        desired_positions = {
            'top_left': '6',
            'top_right': '3', 
            'bottom_left': '5',
            'bottom_right': '1'
        }
        
        for position, camera_id in desired_positions.items():
            if camera_id in self.camera_id_list:
                try:
                    camera_index = self.camera_id_list.index(camera_id)
                    self.quad_positions[position] = camera_index
                    logger.debug(f"Set initial quad position {position} to Camera {camera_id}")
                except ValueError:
                    logger.warning(f"Camera {camera_id} not found for initial quad position {position}")

def main():
    """Main entry point for the robust FFmpeg viewer"""
    try:
        viewer = RobustViewer()
        return viewer.start()
    except KeyboardInterrupt:
        logger.info("Viewer interrupted by user")
        return 0
    except Exception as e:
        logger.error(f"Viewer failed to start: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
