#!/usr/bin/env python3
"""
Advanced Multi-threaded RTSP Camera Viewer
Optimized for high bitrate streams (4500kb/s, 1080p, 20fps)
"""

import cv2
import threading
import queue
import time
import numpy as np
from typing import Optional, Dict, Any, Tuple
import logging
import sys
import os
import configparser
import warnings

# Suppress OpenCV warnings about pixel formats
warnings.filterwarnings("ignore", category=UserWarning, module="cv2")
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"  # Suppress OpenCV logging warnings

# Try to import our timeout reader
try:
    from timeout_reader import TimeoutFrameReader
    HAS_TIMEOUT_READER = True
except ImportError:
    HAS_TIMEOUT_READER = False

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CameraConfig:
    """Configuration manager for camera settings"""
    
    def __init__(self, config_file: str = "config.ini"):
        # If running from src directory, look for config in parent directory
        if not os.path.exists(config_file) and os.path.basename(os.getcwd()) == 'src':
            config_file = os.path.join('..', config_file)
        self.config_file = config_file
        self.config = configparser.ConfigParser()
        self.load_config()
        
    def load_config(self):
        """Load configuration from file"""
        if not os.path.exists(self.config_file):
            logger.error(f"Config file {self.config_file} not found!")
            raise FileNotFoundError(f"Config file {self.config_file} not found!")
            
        self.config.read(self.config_file)
        logger.info(f"Loaded configuration from {self.config_file}")
        
    def get_camera_urls(self) -> Dict[str, str]:
        """Get main camera URLs from config"""
        urls = {}
        if 'cameras' in self.config:
            for key, value in self.config['cameras'].items():
                if key.startswith('camera') and key.endswith('_url') and not key.endswith('_lowres'):
                    # Extract camera number from key (e.g., camera1_url -> 1)
                    camera_num = key.replace('camera', '').replace('_url', '')
                    urls[camera_num] = value
                    
        if not urls:
            logger.error("No camera URLs found in config file!")
            raise ValueError("No camera URLs found in config file!")
            
        logger.info(f"Found {len(urls)} main camera URLs in config")
        return urls
        
    def get_camera_lowres_urls(self) -> Dict[str, str]:
        """Get low-resolution camera URLs from config"""
        urls = {}
        if 'cameras' in self.config:
            for key, value in self.config['cameras'].items():
                if key.startswith('camera') and key.endswith('_url_lowres'):
                    # Extract camera number from key (e.g., camera1_url_lowres -> 1)
                    camera_num = key.replace('camera', '').replace('_url_lowres', '')
                    urls[camera_num] = value
                    
        logger.info(f"Found {len(urls)} low-res camera URLs in config")
        return urls
        
    def get_display_size(self) -> Tuple[int, int]:
        """Get display size from config"""
        try:
            width = self.config.getint('display', 'single_camera_width', fallback=640)
            height = self.config.getint('display', 'single_camera_height', fallback=360)
            return (width, height)
        except Exception as e:
            logger.warning(f"Error reading display size from config: {e}, using defaults")
            return (640, 360)
            
    def get_max_display_fps(self) -> int:
        """Get maximum display FPS from config"""
        try:
            return self.config.getint('display', 'max_display_fps', fallback=30)
        except Exception as e:
            logger.warning(f"Error reading max display FPS from config: {e}, using default")
            return 30
            
    def get_frame_buffer_size(self) -> int:
        """Get frame buffer size from config"""
        try:
            return self.config.getint('performance', 'frame_buffer_size', fallback=1)
        except Exception as e:
            logger.warning(f"Error reading frame buffer size from config: {e}, using default")
            return 1
            
    def get_restart_delay(self) -> float:
        """Get restart delay from config"""
        try:
            return self.config.getfloat('performance', 'restart_delay', fallback=5.0)
        except Exception as e:
            logger.warning(f"Error reading restart delay from config: {e}, using default")
            return 5.0
            
    def get_max_retries(self) -> int:
        """Get max retries from config"""
        try:
            return self.config.getint('performance', 'max_retries', fallback=10)
        except Exception as e:
            logger.warning(f"Error reading max retries from config: {e}, using default")
            return 10
            
    def get_health_check_timeout(self) -> float:
        """Get health check timeout from config"""
        try:
            return self.config.getfloat('performance', 'health_check_timeout', fallback=10.0)
        except Exception as e:
            logger.warning(f"Error reading health check timeout from config: {e}, using default")
            return 10.0
            
    def get_connection_timeout_ms(self) -> int:
        """Get connection timeout in milliseconds from config"""
        try:
            return self.config.getint('performance', 'connection_timeout_ms', fallback=5000)
        except Exception as e:
            logger.warning(f"Error reading connection timeout from config: {e}, using default")
            return 5000
            
    def get_read_timeout_ms(self) -> int:
        """Get read timeout in milliseconds from config"""
        try:
            return self.config.getint('performance', 'read_timeout_ms', fallback=5000)
        except Exception as e:
            logger.warning(f"Error reading read timeout from config: {e}, using default")
            return 5000
            
    def get_frame_read_timeout(self) -> float:
        """Get frame read timeout in seconds from config"""
        try:
            return self.config.getfloat('performance', 'frame_read_timeout', fallback=5.0)
        except Exception as e:
            logger.warning(f"Error reading frame read timeout from config: {e}, using default")
            return 5.0
            
    def get_ultra_low_latency_mode(self) -> bool:
        """Get ultra low latency mode setting from config"""
        try:
            return self.config.getboolean('performance', 'ultra_low_latency_mode', fallback=False)
        except Exception as e:
            logger.warning(f"Error reading ultra low latency mode from config: {e}, using default")
            return False
            
    def get_disable_overlays(self) -> bool:
        """Get disable overlays setting for maximum performance"""
        try:
            return self.config.getboolean('performance', 'disable_overlays', fallback=False)
        except Exception as e:
            logger.warning(f"Error reading disable overlays from config: {e}, using default")
            return False
            
    def get_zero_copy_mode(self) -> bool:
        """Get zero-copy mode setting for experimental ultra-low latency"""
        try:
            return self.config.getboolean('performance', 'zero_copy_mode', fallback=False)
        except Exception as e:
            logger.warning(f"Error reading zero copy mode from config: {e}, using default")
            return False
            
    def get_skip_display_sync(self) -> bool:
        """Get skip display sync setting for maximum frame rate"""
        try:
            return self.config.getboolean('performance', 'skip_display_sync', fallback=False)
        except Exception as e:
            logger.warning(f"Error reading skip display sync from config: {e}, using default")
            return False
            
    def get_minimal_processing_mode(self) -> bool:
        """Get minimal processing mode for ultra-low latency"""
        try:
            return self.config.getboolean('performance', 'minimal_processing_mode', fallback=False)
        except Exception as e:
            logger.warning(f"Error reading minimal processing mode from config: {e}, using default")
            return False
            
    def get_memory_pool_size(self) -> int:
        """Get memory pool size for frame buffers"""
        try:
            return self.config.getint('performance', 'memory_pool_size', fallback=0)
        except Exception as e:
            logger.warning(f"Error reading memory pool size from config: {e}, using default")
            return 0
            
    def get_hardware_decode(self) -> bool:
        """Get hardware decode setting"""
        try:
            return self.config.getboolean('performance', 'hardware_decode', fallback=True)
        except Exception as e:
            logger.warning(f"Error reading hardware decode from config: {e}, using default")
            return True
            
    def get_skip_frame_validation(self) -> bool:
        """Get skip frame validation setting for experimental speed"""
        try:
            return self.config.getboolean('performance', 'skip_frame_validation', fallback=False)
        except Exception as e:
            logger.warning(f"Error reading skip frame validation from config: {e}, using default")
            return False
            
    def get_suppress_format_warnings(self) -> bool:
        """Get suppress format warnings setting"""
        try:
            return self.config.getboolean('performance', 'suppress_format_warnings', fallback=True)
        except Exception as e:
            logger.warning(f"Error reading suppress format warnings from config: {e}, using default")
            return True
            
    def get_force_rgb_conversion(self) -> bool:
        """Get force RGB conversion setting to handle format issues"""
        try:
            return self.config.getboolean('performance', 'force_rgb_conversion', fallback=True)
        except Exception as e:
            logger.warning(f"Error reading force RGB conversion from config: {e}, using default")
            return True

class AdvancedRTSPCamera:
    """Advanced RTSP camera with hardware acceleration and ultra-low latency optimizations"""
    
    def __init__(self, camera_id: str, rtsp_url: str, target_size: Tuple[int, int] = (640, 360), 
                 max_queue_size: int = 1, max_retries: int = 10, restart_delay: float = 5.0,
                 health_check_timeout: float = 10.0, connection_timeout_ms: int = 5000,
                 read_timeout_ms: int = 5000, frame_read_timeout: float = 5.0,
                 ultra_low_latency: bool = False, disable_overlays: bool = False,
                 zero_copy_mode: bool = False, minimal_processing: bool = False,
                 hardware_decode: bool = True, skip_frame_validation: bool = False,
                 force_rgb_conversion: bool = True, rtsp_url_lowres: str = None):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.rtsp_url_lowres = rtsp_url_lowres  # Low-resolution stream URL
        self.current_url = rtsp_url  # Currently active URL
        self.is_lowres_mode = False  # Track current resolution mode
        self.target_size = target_size
        self.max_queue_size = max_queue_size
        self.max_retries = max_retries
        self.restart_delay = restart_delay
        self.health_check_timeout = health_check_timeout
        self.connection_timeout_ms = connection_timeout_ms
        self.read_timeout_ms = read_timeout_ms
        self.frame_read_timeout = frame_read_timeout
        self.ultra_low_latency = ultra_low_latency
        self.disable_overlays = disable_overlays
        self.zero_copy_mode = zero_copy_mode
        self.minimal_processing = minimal_processing
        self.hardware_decode = hardware_decode
        self.skip_frame_validation = skip_frame_validation
        self.force_rgb_conversion = force_rgb_conversion
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        self.running = False
        self.thread = None
        self.cap = None
        
        # Performance metrics
        self.fps_counter = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.dropped_frames = 0
        self.total_frames = 0
        
        # Memory management for zero-copy mode
        if self.zero_copy_mode:
            self.frame_buffer_pool = []
            self.pool_size = 10  # Pre-allocated frame buffers
            self._init_frame_pool()
        
        # Connection health
        self.last_frame_time = time.time()
        self.connection_healthy = False
        
    def _init_frame_pool(self):
        """Initialize pre-allocated frame buffer pool for zero-copy mode"""
        if not self.zero_copy_mode:
            return
        
        logger.info(f"Camera {self.camera_id} initializing zero-copy frame pool")
        for _ in range(self.pool_size):
            frame_buffer = np.empty((self.target_size[1], self.target_size[0], 3), dtype=np.uint8)
            self.frame_buffer_pool.append(frame_buffer)
    
    def _get_frame_buffer(self) -> Optional[np.ndarray]:
        """Get a pre-allocated frame buffer from the pool"""
        if not self.zero_copy_mode or not self.frame_buffer_pool:
            return None
        return self.frame_buffer_pool.pop(0) if self.frame_buffer_pool else None
    
    def _return_frame_buffer(self, frame_buffer: np.ndarray):
        """Return a frame buffer to the pool"""
        if self.zero_copy_mode and len(self.frame_buffer_pool) < self.pool_size:
            self.frame_buffer_pool.append(frame_buffer)
    
    def _process_frame_ultra_fast(self, frame: np.ndarray) -> np.ndarray:
        """Ultra-fast frame processing with minimal overhead"""
        if frame is None:
            return None
            
        # Validate and fix frame format if needed
        frame = self._validate_frame_format(frame)
        if frame is None:
            return None
            
        # Zero-copy optimization - use pre-allocated buffer if available
        if self.zero_copy_mode:
            target_buffer = self._get_frame_buffer()
            if target_buffer is not None:
                # Resize directly into the pre-allocated buffer
                if frame.shape[:2] != (self.target_size[1], self.target_size[0]):
                    # Use fastest interpolation for ultra-low latency
                    interpolation = cv2.INTER_NEAREST if self.ultra_low_latency else cv2.INTER_LINEAR
                    cv2.resize(frame, self.target_size, dst=target_buffer, interpolation=interpolation)
                else:
                    # Direct copy to avoid resize
                    np.copyto(target_buffer, frame)
                processed_frame = target_buffer
            else:
                # Fallback to regular resize if no buffer available
                interpolation = cv2.INTER_NEAREST if self.ultra_low_latency else cv2.INTER_LINEAR
                processed_frame = cv2.resize(frame, self.target_size, interpolation=interpolation) if frame.shape[:2] != (self.target_size[1], self.target_size[0]) else frame
        else:
            # Regular processing with optimal interpolation
            interpolation = cv2.INTER_NEAREST if self.ultra_low_latency else cv2.INTER_LINEAR
            processed_frame = cv2.resize(frame, self.target_size, interpolation=interpolation) if frame.shape[:2] != (self.target_size[1], self.target_size[0]) else frame
        
        # Add overlay only if not disabled and not in minimal processing mode
        if not self.disable_overlays and not self.minimal_processing:
            processed_frame = self._add_overlay(processed_frame)
        
        return processed_frame
    
    def _validate_frame_format(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """Validate and fix frame format issues (like yuvj420p) - more lenient approach"""
        if frame is None:
            return None
            
        try:
            # Check if frame has the expected shape and data type
            if len(frame.shape) == 3 and frame.shape[2] == 3 and frame.dtype == np.uint8:
                # Frame looks perfect as 3-channel color
                return frame
            elif len(frame.shape) == 3 and frame.shape[2] == 3:
                # Frame has 3 channels but wrong dtype - convert to uint8
                if frame.dtype != np.uint8:
                    if frame.max() <= 1.0:  # Normalized values
                        frame = (frame * 255).astype(np.uint8)
                    else:
                        frame = frame.astype(np.uint8)
                return frame
            elif len(frame.shape) == 2:
                # Single channel (grayscale) - convert to 3-channel
                return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 1:
                # Single channel but with extra dimension - convert to 3-channel
                frame_2d = frame.squeeze()
                return cv2.cvtColor(frame_2d, cv2.COLOR_GRAY2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 4:
                # 4-channel (RGBA) - convert to 3-channel BGR
                return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            else:
                # For any other format, try to use it as-is if it has reasonable dimensions
                if len(frame.shape) >= 2 and frame.shape[0] > 0 and frame.shape[1] > 0:
                    # Try to reshape to 3-channel if possible
                    if len(frame.shape) == 3:
                        return frame  # Accept as-is and let OpenCV handle it
                    else:
                        # Convert grayscale to BGR
                        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                else:
                    # Only log warning occasionally to avoid spam
                    if not self.skip_frame_validation and hasattr(self, '_last_format_warning'):
                        if time.time() - self._last_format_warning > 5.0:  # Log every 5 seconds max
                            logger.warning(f"Camera {self.camera_id} unusual frame format: {frame.shape}")
                            self._last_format_warning = time.time()
                    elif not hasattr(self, '_last_format_warning'):
                        self._last_format_warning = time.time()
                        if not self.skip_frame_validation:
                            logger.warning(f"Camera {self.camera_id} unusual frame format: {frame.shape}")
                    return None
                
        except Exception as e:
            # Only log validation errors occasionally to avoid spam
            if not self.skip_frame_validation and hasattr(self, '_last_validation_error'):
                if time.time() - self._last_validation_error > 10.0:  # Log every 10 seconds max
                    logger.warning(f"Camera {self.camera_id} frame validation error: {e}")
                    self._last_validation_error = time.time()
            elif not hasattr(self, '_last_validation_error'):
                self._last_validation_error = time.time()
                if not self.skip_frame_validation:
                    logger.warning(f"Camera {self.camera_id} frame validation error: {e}")
            # Return the original frame if validation fails - let OpenCV try to handle it
            return frame
    
    def start(self):
        """Start the camera capture thread"""
        if self.thread is not None and self.thread.is_alive():
            logger.warning(f"Camera {self.camera_id} thread already running")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info(f"Started camera thread for {self.camera_id}")
        
    def stop(self):
        """Stop the camera capture thread"""
        self.running = False
        self.connection_healthy = False
        
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            if self.thread.is_alive():
                logger.warning(f"Camera {self.camera_id} thread did not stop gracefully")
                
        if self.cap is not None:
            try:
                self.cap.release()
            except:
                pass  # Ignore errors during cleanup
            self.cap = None
            
        logger.info(f"Stopped camera {self.camera_id} - Dropped {self.dropped_frames}/{self.total_frames} frames")
        
    def _setup_capture(self) -> bool:
        """Setup camera capture with optimal settings - improved stability"""
        try:
            # Release existing capture with better error handling
            if self.cap is not None:
                try:
                    if self.cap.isOpened():
                        self.cap.release()
                    time.sleep(0.1)  # Give time for proper cleanup
                except Exception as e:
                    logger.debug(f"Camera {self.camera_id} cleanup warning: {e}")
                finally:
                    self.cap = None
                
            # Create new capture with backend preference and format handling
            backends_with_options = [
                (cv2.CAP_FFMPEG, ['-avoid_negative_ts', 'make_zero', '-fflags', '+genpts+discardcorrupt', '-flags', 'low_delay']),
                (cv2.CAP_GSTREAMER, []),
                (cv2.CAP_V4L2, [])
            ]
            
            for backend, options in backends_with_options:
                try:
                    if backend == cv2.CAP_FFMPEG and options:
                        # Set FFmpeg options to handle format issues better
                        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = ';'.join(options)
                    
                    self.cap = cv2.VideoCapture(self.current_url, backend)
                    if self.cap.isOpened():
                        logger.info(f"Camera {self.camera_id} using backend: {backend}")
                        break
                    else:
                        # Clean up failed attempt
                        if self.cap is not None:
                            self.cap.release()
                            self.cap = None
                except Exception as e:
                    logger.debug(f"Camera {self.camera_id} backend {backend} failed: {e}")
                    if self.cap is not None:
                        try:
                            self.cap.release()
                        except:
                            pass
                        self.cap = None
                    continue
            else:
                # Fallback to default
                try:
                    self.cap = cv2.VideoCapture(self.current_url)
                    if not self.cap.isOpened():
                        if self.cap is not None:
                            self.cap.release()
                            self.cap = None
                        return False
                except Exception as e:
                    logger.error(f"Camera {self.camera_id} fallback capture failed: {e}")
                    return False
                
            if self.cap is None or not self.cap.isOpened():
                return False
                
            # Ultra-low latency settings for RTSP streams
            settings = {
                cv2.CAP_PROP_BUFFERSIZE: 1,           # Absolute minimum buffer
                cv2.CAP_PROP_FPS: 30,                 # Higher FPS for smoother playback
                cv2.CAP_PROP_FRAME_WIDTH: 1920,       # Source resolution
                cv2.CAP_PROP_FRAME_HEIGHT: 1080,
                cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'H264'),
                # Timeout settings to prevent hanging
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC: self.connection_timeout_ms,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC: self.read_timeout_ms,
                # Color space and format settings to handle yuvj420p properly
                cv2.CAP_PROP_CONVERT_RGB: 1 if getattr(self, 'force_rgb_conversion', True) else 0,
                cv2.CAP_PROP_FORMAT: cv2.CV_8UC3,     # Force 8-bit 3-channel format
            }
            
            # Add ultra-low latency specific settings
            if self.ultra_low_latency:
                ultra_settings = {
                    cv2.CAP_PROP_BUFFERSIZE: 0,       # Zero buffer for minimal latency
                    cv2.CAP_PROP_FPS: 60,             # Max FPS for ultra-low latency
                    cv2.CAP_PROP_POS_MSEC: 0,         # Start from beginning
                    # Keep RGB conversion enabled even in ultra-low latency to avoid format issues
                    cv2.CAP_PROP_CONVERT_RGB: 1 if getattr(self, 'force_rgb_conversion', True) else 0,
                }
                settings.update(ultra_settings)
                
            # Hardware decoding settings
            if self.hardware_decode:
                hw_settings = {
                    # Enable hardware acceleration if available
                    cv2.CAP_PROP_CODEC_PIXEL_FORMAT: -1,  # Let OpenCV choose optimal format
                    # Additional hardware decode options
                    cv2.CAP_PROP_HW_ACCELERATION: cv2.VIDEO_ACCELERATION_ANY,
                }
                settings.update(hw_settings)
            
            for prop, value in settings.items():
                try:
                    self.cap.set(prop, value)
                except:
                    # Some backends don't support all properties
                    pass
                
            # Test read to verify connection - use threading approach only
            test_success = False
            
            if HAS_TIMEOUT_READER:
                try:
                    timeout_reader = TimeoutFrameReader(self.cap, 10.0)
                    ret, frame = timeout_reader.read_with_timeout()
                    test_success = ret and frame is not None
                    if test_success:
                        logger.info(f"Camera {self.camera_id} connected successfully")
                    else:
                        logger.warning(f"Camera {self.camera_id} failed initial frame test")
                except Exception as e:
                    logger.warning(f"Camera {self.camera_id} threading timeout failed: {e}")
                    test_success = False
            else:
                # Direct read without timeout as last resort
                try:
                    ret, frame = self.cap.read()
                    test_success = ret and frame is not None
                    if test_success:
                        logger.info(f"Camera {self.camera_id} connected successfully (no timeout)")
                    else:
                        logger.warning(f"Camera {self.camera_id} failed direct test read")
                except Exception as e:
                    logger.warning(f"Camera {self.camera_id} direct test read failed: {e}")
                    test_success = False
            
            if test_success:
                self.connection_healthy = True
                return True
            else:
                return False
                
        except Exception as e:
            logger.error(f"Camera {self.camera_id} setup failed: {e}")
            return False
            
    def _capture_loop(self):
        """Main capture loop with automatic reconnection and timeout handling"""
        consecutive_failures = 0
        last_successful_read = time.time()
        read_timeout = self.frame_read_timeout * 2  # Use double the frame timeout for overall read timeout
        
        while self.running:
            if not self.connection_healthy or consecutive_failures > 0:
                logger.info(f"Camera {self.camera_id} attempting connection...")
                if not self._setup_capture():
                    consecutive_failures += 1
                    if consecutive_failures >= self.max_retries:
                        logger.error(f"Camera {self.camera_id} failed after {self.max_retries} attempts")
                        break
                    time.sleep(self.restart_delay)
                    continue
                else:
                    consecutive_failures = 0
                    last_successful_read = time.time()
                    
            try:
                # Check if too much time has passed since last successful read
                current_time = time.time()
                if current_time - last_successful_read > read_timeout:
                    logger.warning(f"Camera {self.camera_id} read timeout, forcing reconnection")
                    self.connection_healthy = False
                    consecutive_failures += 1
                    continue
                
                # Use timeout for frame reading - threading approach only
                frame_read_success = False
                ret, frame = False, None
                
                # Safety check - ensure capture is still valid
                if self.cap is None or not self.cap.isOpened():
                    self.connection_healthy = False
                    consecutive_failures += 1
                    continue
                
                if HAS_TIMEOUT_READER:
                    try:
                        timeout_reader = TimeoutFrameReader(self.cap, self.frame_read_timeout)
                        ret, frame = timeout_reader.read_with_timeout()
                        frame_read_success = ret and frame is not None
                        if not frame_read_success:
                            # Don't log every timeout, only occasional ones
                            if consecutive_failures % 5 == 0:
                                logger.warning(f"Camera {self.camera_id} frame read timeout")
                    except Exception as e:
                        logger.warning(f"Camera {self.camera_id} timeout reader failed: {e}")
                        frame_read_success = False
                else:
                    # Direct read without timeout as fallback
                    try:
                        # Additional safety check before reading
                        if self.cap is not None and self.cap.isOpened():
                            ret, frame = self.cap.read()
                            frame_read_success = ret and frame is not None
                        else:
                            frame_read_success = False
                        if not frame_read_success and consecutive_failures % 5 == 0:
                            logger.warning(f"Camera {self.camera_id} direct read failed")
                    except Exception as e:
                        if consecutive_failures % 5 == 0:
                            logger.warning(f"Camera {self.camera_id} direct read exception: {e}")
                        frame_read_success = False
                
                if not frame_read_success:
                    self.connection_healthy = False
                    consecutive_failures += 1
                    continue
                
                # Skip frame validation for experimental speed boost
                if not self.skip_frame_validation and (not ret or frame is None):
                    # Only fail if both ret is False AND frame is None
                    if not ret and frame is None:
                        logger.warning(f"Camera {self.camera_id} failed to read frame")
                        self.connection_healthy = False
                        consecutive_failures += 1
                        continue
                    # If we have a frame even with ret=False, try to use it
                elif self.skip_frame_validation and frame is None:
                    # In skip validation mode, only check for None frame
                    continue
                
                # Additional safety check - if frame exists but ret is False, still try to use it
                if frame is None:
                    # Only count as failure if we truly have no frame
                    continue
                    
                # Successful frame read
                self.total_frames += 1
                self.last_frame_time = time.time()
                last_successful_read = self.last_frame_time
                
                # Skip FPS calculation in minimal processing mode
                if not self.minimal_processing:
                    self._update_fps()
                
                # Reset consecutive failures on successful read
                if consecutive_failures > 0:
                    logger.info(f"Camera {self.camera_id} recovered after {consecutive_failures} failures")
                    consecutive_failures = 0
                
                # Ultra-fast frame processing
                processed_frame = self._process_frame_ultra_fast(frame)
                
                # Ultra-aggressive frame dropping - always drop old frames immediately
                while not self.frame_queue.empty():
                    try:
                        old_frame = self.frame_queue.get_nowait()
                        # Return old frame to pool if using zero-copy mode
                        if self.zero_copy_mode and old_frame is not None:
                            self._return_frame_buffer(old_frame)
                        # Explicitly delete the reference to help with memory management
                        del old_frame
                        self.dropped_frames += 1
                    except queue.Empty:
                        break
                        
                # Add new frame with non-blocking put
                try:
                    self.frame_queue.put_nowait(processed_frame)
                except queue.Full:
                    # This shouldn't happen since we just emptied the queue, but just in case
                    self.dropped_frames += 1
                    
            except Exception as e:
                logger.warning(f"Camera {self.camera_id} capture error: {e}")
                self.connection_healthy = False
                consecutive_failures += 1
                
        self.connection_healthy = False
        
    def _update_fps(self):
        """Update FPS calculation"""
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.last_fps_time = current_time
            
    def _add_overlay(self, frame):
        """Add informational overlay to frame"""
        if frame is None:
            return frame
            
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        
        # Background for text readability
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (200, 100), (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)
        
        # Calculate frame age
        frame_age = time.time() - self.last_frame_time
        
        # Text information
        stream_info = self.get_current_stream_info()
        texts = [
            f"Cam {self.camera_id} ({stream_info})",
            f"FPS: {self.fps}",
            f"Drop: {self.dropped_frames}",
            f"Age: {frame_age:.1f}s",
            f"Health: {'OK' if self.connection_healthy else 'FAIL'}"
        ]
        
        colors = [
            (0, 255, 0) if stream_info == "MAIN" else (255, 255, 0),  # Camera ID - green for main, yellow for low-res
            (255, 255, 255),  # FPS - white
            (0, 255, 255),  # Dropped frames - cyan
            (255, 255, 0) if frame_age < 1.0 else (255, 0, 0),  # Frame age - yellow if fresh, red if old
            (0, 255, 0) if self.connection_healthy else (0, 0, 255)  # Health - green/red
        ]
        
        for i, (text, color) in enumerate(zip(texts, colors)):
            y_pos = 20 + i * 15
            cv2.putText(frame, text, (10, y_pos), font, font_scale, color, thickness)
            
        return frame
        
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the most recent frame - improved to reduce flickering"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            # Only show placeholder if camera has been unhealthy for a significant time
            if not self.is_healthy():
                # Check how long the camera has been unhealthy
                time_since_last_frame = time.time() - self.last_frame_time
                
                # Only show "not connected" if we haven't had a frame for a while
                if time_since_last_frame > 10.0:  # 10 seconds grace period
                    # Return black frame with status only if camera has been down for a while
                    black_frame = np.zeros((self.target_size[1], self.target_size[0], 3), dtype=np.uint8)
                    status = "NO SIGNAL" if time_since_last_frame > 30 else "RECONNECTING..."
                    color = (0, 0, 255) if time_since_last_frame > 30 else (0, 255, 255)
                    
                    cv2.putText(black_frame, f"Camera {self.camera_id}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.putText(black_frame, status, (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    return black_frame
                else:
                    # Camera just became unhealthy, return None to use last good frame
                    return None
            else:
                # Camera is healthy but just no frame in queue right now
                return None
            
    def is_healthy(self) -> bool:
        """Check if camera connection is healthy - more lenient approach"""
        # Camera is healthy if:
        # 1. Connection is marked as healthy AND
        # 2. We've received a frame within a reasonable timeout period (use 2x timeout for safety)
        extended_timeout = self.health_check_timeout * 2
        return (self.connection_healthy and 
                time.time() - self.last_frame_time < extended_timeout)

    def switch_stream_quality(self) -> bool:
        """Switch between main and low-resolution streams - thread-safe version"""
        if not self.rtsp_url_lowres:
            logger.warning(f"Camera {self.camera_id} has no low-res URL configured")
            return False
        
        # Add thread safety - ensure we're not in the middle of processing
        old_running = self.running
        self.running = False
        
        # Wait a moment for current operations to complete
        time.sleep(0.1)
        
        try:
            # Clear frame queue to prevent using old frames
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    break
            
            # Toggle between streams
            if self.is_lowres_mode:
                # Switch to main stream
                self.current_url = self.rtsp_url
                self.is_lowres_mode = False
                logger.info(f"Camera {self.camera_id} switching to MAIN stream")
            else:
                # Switch to low-res stream
                self.current_url = self.rtsp_url_lowres
                self.is_lowres_mode = True
                logger.info(f"Camera {self.camera_id} switching to LOW-RES stream")
            
            # Force reconnection with new URL
            self.connection_healthy = False
            
            # Close current capture safely
            if self.cap is not None:
                try:
                    # Release capture in a safer way
                    if self.cap.isOpened():
                        self.cap.release()
                    time.sleep(0.05)  # Brief pause to ensure cleanup
                except Exception as e:
                    logger.debug(f"Camera {self.camera_id} capture release error (non-fatal): {e}")
                finally:
                    self.cap = None
            
            # Resume operation
            self.running = old_running
            
            return True
            
        except Exception as e:
            logger.error(f"Camera {self.camera_id} stream switch error: {e}")
            # Restore running state even if there was an error
            self.running = old_running
            return False
        
    def get_current_stream_info(self) -> str:
        """Get info about current stream quality"""
        return "LOW-RES" if self.is_lowres_mode else "MAIN"

class HighPerformanceViewer:
    """High-performance multi-camera viewer optimized for 4500kb/s streams"""
    
    def __init__(self, config: CameraConfig):
        self.config = config
        self.camera_configs = config.get_camera_urls()
        self.cameras = {}
        self.running = False
        
        # Get display settings from config
        self.single_cam_size = config.get_display_size()
        self.grid_size = (self.single_cam_size[0] * 2, self.single_cam_size[1] * 2)  # 2x2 grid
        self.max_display_fps = config.get_max_display_fps()
        
        # Performance monitoring
        self.display_fps = 0
        self.display_fps_counter = 0
        self.last_display_fps_time = time.time()
        self.target_frame_time = 1.0 / self.max_display_fps if self.max_display_fps > 0 else 0
        
        # Low-latency optimization settings
        self.enable_vsync = False  # Disable VSync for lower latency
        self.skip_frame_limit = False  # Option to disable FPS limiting for max performance
        
        # Keep last good frame for each camera to avoid blinking
        self.last_good_frames = {}
        
        # Full screen mode for single camera display
        self.fullscreen_mode = False
        self.current_camera_index = 0
        self.camera_ids = sorted(self.camera_configs.keys())
        
        # Window management for better scaling
        self.window_name = "Multi-Camera RTSP Viewer (Press 'q' to quit)"
        
    def start(self):
        """Start the viewer application"""
        logger.info("Starting high-performance camera viewer...")
        logger.info(f"Target resolution per camera: {self.single_cam_size}")
        logger.info(f"Total display resolution: {self.grid_size}")
        logger.info(f"Max display FPS: {self.max_display_fps}")
        
        # Initialize cameras with config settings and ultra-low latency optimizations
        ultra_low_latency = self.config.get_ultra_low_latency_mode()
        disable_overlays = self.config.get_disable_overlays()
        zero_copy_mode = self.config.get_zero_copy_mode()
        skip_display_sync = self.config.get_skip_display_sync()
        minimal_processing = self.config.get_minimal_processing_mode()
        hardware_decode = self.config.get_hardware_decode()
        skip_frame_validation = self.config.get_skip_frame_validation()
        force_rgb_conversion = self.config.get_force_rgb_conversion()
        
        logger.info(f"Ultra low latency mode: {'ENABLED' if ultra_low_latency else 'DISABLED'}")
        logger.info(f"Overlays: {'DISABLED' if disable_overlays else 'ENABLED'}")
        logger.info(f"Zero-copy mode: {'ENABLED' if zero_copy_mode else 'DISABLED'}")
        logger.info(f"Display sync: {'DISABLED' if skip_display_sync else 'ENABLED'}")
        logger.info(f"Minimal processing: {'ENABLED' if minimal_processing else 'DISABLED'}")
        logger.info(f"Hardware decode: {'ENABLED' if hardware_decode else 'DISABLED'}")
        logger.info(f"Frame validation: {'DISABLED' if skip_frame_validation else 'ENABLED'}")
        logger.info(f"Force RGB conversion: {'ENABLED' if force_rgb_conversion else 'DISABLED'}")
        
        # Set display sync mode
        self.skip_display_sync = skip_display_sync
        if skip_display_sync:
            self.target_frame_time = 0  # No FPS limiting
        
        # Get low-res camera URLs
        camera_lowres_configs = self.config.get_camera_lowres_urls()
        
        for camera_id, rtsp_url in self.camera_configs.items():
            # Get corresponding low-res URL if available
            rtsp_url_lowres = camera_lowres_configs.get(camera_id, None)
            
            camera = AdvancedRTSPCamera(
                camera_id=camera_id, 
                rtsp_url=rtsp_url, 
                target_size=self.single_cam_size,
                max_queue_size=self.config.get_frame_buffer_size(),
                max_retries=self.config.get_max_retries(),
                restart_delay=self.config.get_restart_delay(),
                health_check_timeout=self.config.get_health_check_timeout(),
                connection_timeout_ms=self.config.get_connection_timeout_ms(),
                read_timeout_ms=self.config.get_read_timeout_ms(),
                frame_read_timeout=self.config.get_frame_read_timeout(),
                ultra_low_latency=ultra_low_latency,
                disable_overlays=disable_overlays,
                zero_copy_mode=zero_copy_mode,
                minimal_processing=minimal_processing,
                hardware_decode=hardware_decode,
                skip_frame_validation=skip_frame_validation,
                force_rgb_conversion=force_rgb_conversion,
                rtsp_url_lowres=rtsp_url_lowres
            )
            self.cameras[camera_id] = camera
            camera.start()
            time.sleep(0.05 if ultra_low_latency else 0.1)  # Ultra-fast startup in low-latency mode
            
        # Start display loop
        self.running = True
        self._display_loop()
        
    def stop(self):
        """Stop all cameras and cleanup"""
        logger.info("Stopping camera viewer...")
        self.running = False
        
        for camera in self.cameras.values():
            camera.stop()
            
        cv2.destroyAllWindows()
        
    def _display_loop(self):
        """Optimized display loop with FPS limiting and adaptive scaling"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # Make window resizable
        
        # Set initial window size with 16:9 aspect ratio
        initial_width = 1280
        initial_height = 720
        cv2.resizeWindow(self.window_name, initial_width, initial_height)
        
        # Display instructions
        self._show_startup_info()
        
        last_frame_time = time.time()
        
        while self.running:
            try:
                frame_start_time = time.time()
                
                # Collect frames from all cameras with ultra-fast processing
                frames = {}
                healthy_cameras = 0
                
                for camera_id, camera in self.cameras.items():
                    frame = camera.get_latest_frame()
                    
                    # Ultra-fast health check (skip complex checks in minimal mode)
                    if hasattr(self, 'skip_display_sync') and self.skip_display_sync:
                        # Skip health check for maximum speed
                        healthy_cameras += 1 if frame is not None else 0
                    else:
                        if camera.is_healthy():
                            healthy_cameras += 1
                    
                    if frame is not None:
                        # Got a new frame, update last good frame
                        frames[camera_id] = frame
                        self.last_good_frames[camera_id] = frame
                        # Also store timestamp for last good frame
                        if not hasattr(self, 'last_good_frame_times'):
                            self.last_good_frame_times = {}
                        self.last_good_frame_times[camera_id] = time.time()
                    elif camera_id in self.last_good_frames:
                        # Check if last good frame is not too old
                        if hasattr(self, 'last_good_frame_times') and camera_id in self.last_good_frame_times:
                            frame_age = time.time() - self.last_good_frame_times[camera_id]
                            if frame_age < 15.0:  # Keep using last good frame for up to 15 seconds
                                frames[camera_id] = self.last_good_frames[camera_id]
                            # If frame is older than 15 seconds, let it fall through to not include it
                        else:
                            # No timestamp info, use last good frame anyway
                            frames[camera_id] = self.last_good_frames[camera_id]
                    # Skip placeholder creation for maximum speed unless no frames at all
                
                # Create display based on mode with minimal processing
                if self.fullscreen_mode:
                    display_frame = self._create_fullscreen_display(frames)
                else:
                    display_frame = self._create_adaptive_grid_layout(frames)
                
                # Ensure we always have something to display
                if display_frame is None:
                    # Create emergency fallback frame
                    display_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                    cv2.putText(display_frame, "Initializing Cameras...", (400, 360), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                
                # Skip global overlay in ultra-fast mode
                if not getattr(self, 'skip_display_sync', False):
                    display_frame = self._add_global_overlay(display_frame, healthy_cameras)
                
                # Display frame
                cv2.imshow(self.window_name, display_frame)
                
                # Update display FPS (skip in minimal processing mode)
                if not getattr(self, 'skip_display_sync', False):
                    self._update_display_fps()
                
                # Handle input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                elif key == ord('f') or key == ord('F'):  # 'f' to toggle fullscreen mode
                    self._toggle_fullscreen_mode()
                elif key == 81 or key == 2 or key == 84:  # Left arrow key (various systems)
                    if self.fullscreen_mode:
                        self._previous_camera()
                    else:
                        # In grid mode, switch to fullscreen with previous camera
                        self._previous_camera()
                        self.fullscreen_mode = True
                        logger.info("Switched to fullscreen mode with previous camera")
                elif key == 83 or key == 3 or key == 85:  # Right arrow key (various systems)
                    if self.fullscreen_mode:
                        self._next_camera()
                    else:
                        # In grid mode, switch to fullscreen with next camera
                        self._next_camera()
                        self.fullscreen_mode = True
                        logger.info("Switched to fullscreen mode with next camera")
                elif key == ord('r'):  # 'r' to restart failed cameras
                    self._restart_failed_cameras()
                elif key == ord('s'):  # 's' to show statistics
                    self._print_statistics()
                elif key == ord('c'):  # 'c' to reload config
                    self._reload_config()
                elif key == ord('p'):  # 'p' to toggle performance mode
                    self.skip_frame_limit = not self.skip_frame_limit
                    logger.info(f"Performance mode (no FPS limit): {'ON' if self.skip_frame_limit else 'OFF'}")
                elif key == ord('z'):  # 'z' to toggle ultra-fast mode
                    self.skip_display_sync = not getattr(self, 'skip_display_sync', False)
                    logger.info(f"Ultra-fast mode (skip sync): {'ON' if self.skip_display_sync else 'OFF'}")
                elif key == ord('l') or key == ord('L'):  # 'l' to toggle stream quality
                    self._toggle_stream_quality()
                    
                # Ultra-fast FPS limiting (can be completely disabled)
                if not getattr(self, 'skip_display_sync', False) and self.target_frame_time > 0 and not self.skip_frame_limit:
                    frame_processing_time = time.time() - frame_start_time
                    sleep_time = self.target_frame_time - frame_processing_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    
            except Exception as e:
                logger.error(f"Display loop error: {e}")
                time.sleep(0.1)
                
        self.stop()
        
    def _create_placeholder_frame(self, camera_id: str, is_healthy: bool) -> np.ndarray:
        """Create a placeholder frame when no frame is available"""
        placeholder = np.zeros((self.single_cam_size[1], self.single_cam_size[0], 3), dtype=np.uint8)
        
        if is_healthy:
            # Camera is healthy but temporarily no frame
            status = "BUFFERING..."
            color = (0, 255, 255)  # Yellow
        else:
            # Camera is not healthy
            status = "CONNECTING..."
            color = (0, 0, 255)  # Red
            
        cv2.putText(placeholder, f"Camera {camera_id}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(placeholder, status, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return placeholder
        
    def _create_grid_layout(self, frames: Dict[str, np.ndarray]) -> np.ndarray:
        """Create 2x2 grid layout from camera frames that adaptively fills the window"""
        camera_ids = sorted(frames.keys())
        
        # Get the current window properties - we'll create a large grid that OpenCV will scale
        # The key is to create a grid that has a good aspect ratio for typical window sizes
        
        # Calculate grid dimensions that work well for common window aspect ratios
        # Most monitors are 16:9, so we'll target that ratio for the overall grid
        target_aspect_ratio = 16.0 / 9.0  # Common widescreen ratio
        
        # Start with a base size and scale up significantly for better quality
        base_grid_height = 1080  # Full HD height for quality
        base_grid_width = int(base_grid_height * target_aspect_ratio)  # 1920 for 16:9
        
        # Individual camera size for the 2x2 grid
        grid_cam_width = base_grid_width // 2
        grid_cam_height = base_grid_height // 2
        
        # Ensure we have frames for all positions, scaled to grid size
        grid_frames = []
        for i in range(4):
            camera_id = str(i + 1)  # Camera IDs are "1", "2", "3", "4"
            if camera_id in frames:
                # Resize frame to grid camera size
                frame = cv2.resize(frames[camera_id], (grid_cam_width, grid_cam_height), 
                                 interpolation=cv2.INTER_LINEAR)
                grid_frames.append(frame)
            else:
                # Create placeholder frame for missing camera at grid size
                placeholder = np.zeros((grid_cam_height, grid_cam_width, 3), dtype=np.uint8)
                
                # Scale text size based on frame size
                font_scale = max(1.0, grid_cam_width / 600)
                thickness = max(2, int(grid_cam_width / 300))
                
                # Center the text better
                text_y_offset = grid_cam_height // 3
                
                cv2.putText(placeholder, f"Camera {camera_id}", 
                           (30, text_y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, (128, 128, 128), thickness)
                cv2.putText(placeholder, "Not Connected", 
                           (30, text_y_offset + int(50 * font_scale)), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (128, 128, 128), thickness)
                grid_frames.append(placeholder)
                
        # Create 2x2 grid
        top_row = np.hstack([grid_frames[0], grid_frames[1]])
        bottom_row = np.hstack([grid_frames[2], grid_frames[3]])
        combined = np.vstack([top_row, bottom_row])
        
        return combined
    
    def _create_adaptive_grid_layout(self, frames: Dict[str, np.ndarray]) -> np.ndarray:
        """Create an adaptive 2x2 grid layout that better fills different window sizes"""
        camera_ids = sorted(frames.keys())
        
        # Create multiple size variants and choose the best one
        # This approach creates content that will scale better to different window sizes
        
        # Try to create a grid that matches common aspect ratios
        # Most people use 16:9 monitors, so optimize for that
        
        # Create a high-resolution grid for quality
        grid_width = 1920  # Full HD width
        grid_height = 1080  # Full HD height
        
        # Individual camera size
        cam_width = grid_width // 2
        cam_height = grid_height // 2
        
        # Process frames for all 4 camera positions
        grid_frames = []
        for i in range(4):
            camera_id = str(i + 1)  # Camera IDs are "1", "2", "3", "4"
            if camera_id in frames:
                # Resize frame to fill the camera slot completely
                frame = cv2.resize(frames[camera_id], (cam_width, cam_height), 
                                 interpolation=cv2.INTER_LINEAR)
                grid_frames.append(frame)
            else:
                # Create placeholder frame that fills the slot
                placeholder = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                
                # Add some visual elements to make it clear this is a camera slot
                # Draw a border
                cv2.rectangle(placeholder, (10, 10), (cam_width-10, cam_height-10), (50, 50, 50), 2)
                
                # Scale text appropriately
                font_scale = max(1.5, cam_width / 400)
                thickness = max(3, int(cam_width / 200))
                
                # Center text better
                text_size = cv2.getTextSize(f"Camera {camera_id}", cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                text_x = (cam_width - text_size[0]) // 2
                text_y = (cam_height - text_size[1]) // 2
                
                cv2.putText(placeholder, f"Camera {camera_id}", 
                           (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, (128, 128, 128), thickness)
                
                status_text = "Not Connected"
                status_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.6, thickness)[0]
                status_x = (cam_width - status_size[0]) // 2
                status_y = text_y + int(60 * font_scale)
                
                cv2.putText(placeholder, status_text, 
                           (status_x, status_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.6, (100, 100, 100), thickness)
                
                grid_frames.append(placeholder)
        
        # Create the 2x2 grid
        top_row = np.hstack([grid_frames[0], grid_frames[1]])
        bottom_row = np.hstack([grid_frames[2], grid_frames[3]])
        final_grid = np.vstack([top_row, bottom_row])
        
        return final_grid
        
    def _add_global_overlay(self, frame: np.ndarray, healthy_cameras: int) -> np.ndarray:
        """Add global status overlay"""
        h, w = frame.shape[:2]
        
        # Skip overlay in fullscreen mode (has its own overlay)
        if self.fullscreen_mode:
            return frame
            
        # Status bar at bottom for grid mode - scale based on frame size
        overlay_height = max(30, int(h * 0.04))  # 4% of height, minimum 30px
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h-overlay_height), (w, h), (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.8, overlay, 0.2, 0)
        
        # Status text with more detailed info
        total_cameras = len(self.cameras)
        status_text = f"Grid View | Cameras: {healthy_cameras}/{total_cameras} healthy | Display FPS: {self.display_fps}/{self.max_display_fps} | F:Fullscreen ← →:Navigate L:Quality Q:Quit R:Restart S:Stats"
        
        # Scale text size based on window width and height
        font_scale = max(0.4, min(w / 2000, h / 1000))
        thickness = max(1, int(font_scale * 2))
        margin = max(10, int(w * 0.008))
        
        cv2.putText(frame, status_text, (margin, h - int(overlay_height * 0.3)), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
        
        return frame
        
    def _update_display_fps(self):
        """Update display FPS counter"""
        self.display_fps_counter += 1
        current_time = time.time()
        if current_time - self.last_display_fps_time >= 1.0:
            self.display_fps = self.display_fps_counter
            self.display_fps_counter = 0
            self.last_display_fps_time = current_time
            
    def _restart_failed_cameras(self):
        """Restart cameras that have failed"""
        logger.info("Checking for failed cameras...")
        failed_cameras = []
        
        for camera_id, camera in self.cameras.items():
            if not camera.is_healthy():
                failed_cameras.append(camera_id)
                
        if failed_cameras:
            logger.info(f"Restarting failed cameras: {failed_cameras}")
            for camera_id in failed_cameras:
                camera = self.cameras[camera_id]
                logger.info(f"Restarting camera {camera_id}")
                camera.stop()
                time.sleep(1)
                camera.start()
        else:
            logger.info("All cameras are healthy - no restart needed")
                
    def _toggle_stream_quality(self):
        """Toggle stream quality for all cameras or current camera in fullscreen mode"""
        if self.fullscreen_mode and self.camera_ids:
            # In fullscreen mode, toggle only the current camera
            current_camera_id = self.camera_ids[self.current_camera_index]
            if current_camera_id in self.cameras:
                camera = self.cameras[current_camera_id]
                if camera.switch_stream_quality():
                    stream_info = camera.get_current_stream_info()
                    logger.info(f"Camera {current_camera_id} switched to {stream_info} stream")
                else:
                    logger.warning(f"Camera {current_camera_id} failed to switch streams")
        else:
            # In grid mode, toggle all cameras with staggered timing to prevent resource conflicts
            switched_cameras = []
            failed_cameras = []
            
            for i, (camera_id, camera) in enumerate(self.cameras.items()):
                # Add small delay between camera switches to prevent simultaneous operations
                if i > 0:
                    time.sleep(0.2)  # 200ms delay between switches
                    
                if camera.switch_stream_quality():
                    stream_info = camera.get_current_stream_info()
                    switched_cameras.append(f"{camera_id}({stream_info})")
                else:
                    failed_cameras.append(camera_id)
            
            if switched_cameras:
                logger.info(f"Switched stream quality for cameras: {', '.join(switched_cameras)}")
            if failed_cameras:
                logger.warning(f"Failed to switch streams for cameras: {', '.join(failed_cameras)}")
                
    def _print_statistics(self):
        """Print detailed statistics"""
        logger.info("=== Camera Statistics ===")
        for camera_id, camera in self.cameras.items():
            drop_rate = (camera.dropped_frames / max(camera.total_frames, 1)) * 100
            logger.info(f"Camera {camera_id}: FPS={camera.fps}, "
                       f"Dropped={camera.dropped_frames}/{camera.total_frames} ({drop_rate:.1f}%), "
                       f"Healthy={camera.is_healthy()}")
        logger.info(f"Display FPS: {self.display_fps}/{self.max_display_fps}")
        logger.info(f"Frame buffer size: {self.config.get_frame_buffer_size()}")
        logger.info(f"Max retries: {self.config.get_max_retries()}")
        logger.info(f"Restart delay: {self.config.get_restart_delay()}s")
        logger.info(f"Health check timeout: {self.config.get_health_check_timeout()}s")
        logger.info(f"Connection timeout: {self.config.get_connection_timeout_ms()}ms")
        logger.info(f"Read timeout: {self.config.get_read_timeout_ms()}ms")
        logger.info(f"Frame read timeout: {self.config.get_frame_read_timeout()}s")
        
    def _reload_config(self):
        """Reload configuration from file"""
        try:
            logger.info("Reloading configuration...")
            self.config.load_config()
            
            # Update display settings
            new_single_cam_size = self.config.get_display_size()
            new_max_display_fps = self.config.get_max_display_fps()
            
            if new_single_cam_size != self.single_cam_size:
                logger.info(f"Display size changed from {self.single_cam_size} to {new_single_cam_size}")
                self.single_cam_size = new_single_cam_size
                self.grid_size = (self.single_cam_size[0] * 2, self.single_cam_size[1] * 2)
                
            if new_max_display_fps != self.max_display_fps:
                logger.info(f"Max display FPS changed from {self.max_display_fps} to {new_max_display_fps}")
                self.max_display_fps = new_max_display_fps
                self.target_frame_time = 1.0 / self.max_display_fps if self.max_display_fps > 0 else 0
                
            # Check for new camera URLs
            new_camera_configs = self.config.get_camera_urls()
            if new_camera_configs != self.camera_configs:
                logger.info("Camera URLs changed - restart required for camera changes")
                
            logger.info("Configuration reloaded successfully")
            
        except Exception as e:
            logger.error(f"Failed to reload configuration: {e}")
        
    def _show_startup_info(self):
        """Show startup information"""
        logger.info("=== Multi-Camera RTSP Viewer ===")
        logger.info("Configuration loaded from config.ini")
        logger.info("Controls:")
        logger.info("  'q' or ESC - Quit application")
        logger.info("  'f' - Toggle fullscreen mode (single camera)")
        logger.info("  ← → Arrow keys - Navigate cameras in fullscreen mode")
        logger.info("  'l' or 'L' - Toggle stream quality (main/low-res)")
        logger.info("  'p' - Toggle performance mode (no FPS limit)")
        logger.info("  'z' - Toggle ultra-fast mode (skip display sync)")
        logger.info("  'r' - Restart failed cameras")
        logger.info("  's' - Show statistics")
        logger.info("  'c' - Reload configuration")
        logger.info("Optimized for ultra-low latency RTSP streams")
        logger.info("Aggressive frame dropping and zero-copy optimizations enabled")
        logger.info("Window is resizable and maximizable")
        
        # Show performance optimizations status
        ultra_low_latency = self.config.get_ultra_low_latency_mode()
        disable_overlays = self.config.get_disable_overlays()
        zero_copy_mode = self.config.get_zero_copy_mode()
        skip_display_sync = self.config.get_skip_display_sync()
        minimal_processing = self.config.get_minimal_processing_mode()
        hardware_decode = self.config.get_hardware_decode()
        skip_frame_validation = self.config.get_skip_frame_validation()
        force_rgb_conversion = self.config.get_force_rgb_conversion()
        
        logger.info(f"Ultra low latency mode: {'ENABLED' if ultra_low_latency else 'DISABLED'}")
        logger.info(f"Overlays: {'DISABLED' if disable_overlays else 'ENABLED'}")
        logger.info(f"Zero-copy mode: {'ENABLED' if zero_copy_mode else 'DISABLED'}")
        logger.info(f"Display sync: {'DISABLED' if skip_display_sync else 'ENABLED'}")
        logger.info(f"Minimal processing: {'ENABLED' if minimal_processing else 'DISABLED'}")
        logger.info(f"Hardware decode: {'ENABLED' if hardware_decode else 'DISABLED'}")
        logger.info(f"Frame validation: {'DISABLED' if skip_frame_validation else 'ENABLED'}")
        logger.info(f"Force RGB conversion: {'ENABLED' if force_rgb_conversion else 'DISABLED'}")
        
        logger.info(f"Using cameras: {list(self.camera_configs.keys())}")
        logger.info(f"Display settings: {self.single_cam_size[0]}x{self.single_cam_size[1]} per camera, max {self.max_display_fps} FPS")
    
    def _toggle_fullscreen_mode(self):
        """Toggle between grid view and fullscreen single camera view"""
        self.fullscreen_mode = not self.fullscreen_mode
        if self.fullscreen_mode:
            current_camera_id = self.camera_ids[self.current_camera_index] if self.camera_ids else "1"
            logger.info(f"Switched to fullscreen mode - Camera {current_camera_id}")
        else:
            logger.info("Switched to grid view")
    
    def _next_camera(self):
        """Switch to next camera"""
        if self.camera_ids:
            self.current_camera_index = (self.current_camera_index + 1) % len(self.camera_ids)
            current_camera_id = self.camera_ids[self.current_camera_index]
            if self.fullscreen_mode:
                logger.info(f"Switched to Camera {current_camera_id}")
    
    def _previous_camera(self):
        """Switch to previous camera"""
        if self.camera_ids:
            self.current_camera_index = (self.current_camera_index - 1) % len(self.camera_ids)
            current_camera_id = self.camera_ids[self.current_camera_index]
            if self.fullscreen_mode:
                logger.info(f"Switched to Camera {current_camera_id}")
    
    def _create_fullscreen_display(self, frames: Dict[str, np.ndarray]) -> np.ndarray:
        """Create fullscreen display for single camera"""
        if not self.camera_ids:
            # No cameras available, create placeholder
            placeholder = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(placeholder, "No Cameras Available", (100, 360), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 3)
            return placeholder
        
        current_camera_id = self.camera_ids[self.current_camera_index]
        
        if current_camera_id in frames:
            frame = frames[current_camera_id]
            
            # Scale frame to a larger size for fullscreen, maintaining aspect ratio
            # Use a more generous fullscreen size that works well when maximized
            target_height = 1080  # Full HD height
            aspect_ratio = frame.shape[1] / frame.shape[0]  # width/height
            target_width = int(target_height * aspect_ratio)
            
            # Ensure reasonable bounds
            if target_width > 1920:  # If too wide, limit by width instead
                target_width = 1920
                target_height = int(target_width / aspect_ratio)
            
            # Scale frame to fullscreen size
            fullscreen_frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
            
            # Add fullscreen overlay
            fullscreen_frame = self._add_fullscreen_overlay(fullscreen_frame, current_camera_id)
            
            return fullscreen_frame
        else:
            # Camera not available, create placeholder
            placeholder = np.zeros((1080, 1920, 3), dtype=np.uint8)
            cv2.putText(placeholder, f"Camera {current_camera_id}", (100, 500), 
                       cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 255, 255), 4)
            cv2.putText(placeholder, "Not Available", (100, 600), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 3)
            return placeholder
    
    def _add_fullscreen_overlay(self, frame: np.ndarray, camera_id: str) -> np.ndarray:
        """Add overlay for fullscreen mode"""
        h, w = frame.shape[:2]
        
        # Scale overlay elements based on frame size
        overlay_height = max(80, int(h * 0.08))  # 8% of height, minimum 80px
        bottom_height = max(50, int(h * 0.05))   # 5% of height, minimum 50px
        
        # Semi-transparent background for overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, overlay_height), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, h-bottom_height), (w, h), (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.8, overlay, 0.2, 0)
        
        # Scale text based on frame size
        title_scale = max(1.0, w / 1200)  # Scale based on width
        info_scale = max(0.7, w / 1600)
        controls_scale = max(0.6, w / 2000)
        
        title_thickness = max(2, int(w / 600))
        info_thickness = max(2, int(w / 800))
        controls_thickness = max(2, int(w / 1000))
        
        # Positioning based on frame size
        margin = max(20, int(w * 0.015))
        
        # Top overlay - Camera info
        camera = self.cameras.get(camera_id)
        if camera:
            # Camera info with stream quality
            stream_info = camera.get_current_stream_info()
            cv2.putText(frame, f"Camera {camera_id} - Fullscreen Mode ({stream_info})", 
                       (margin, int(overlay_height * 0.4)), 
                       cv2.FONT_HERSHEY_SIMPLEX, title_scale, (0, 255, 0), title_thickness)
            cv2.putText(frame, f"FPS: {camera.fps} | Health: {'OK' if camera.is_healthy() else 'FAIL'}", 
                       (margin, int(overlay_height * 0.75)), 
                       cv2.FONT_HERSHEY_SIMPLEX, info_scale, (255, 255, 255), info_thickness)
        
        # Bottom overlay - Controls
        controls_text = f"Camera {self.current_camera_index + 1}/{len(self.camera_ids)} | ← → : Switch Camera | L: Quality | F: Grid View | Q: Quit"
        cv2.putText(frame, controls_text, (margin, h - int(bottom_height * 0.3)), 
                   cv2.FONT_HERSHEY_SIMPLEX, controls_scale, (0, 255, 255), controls_thickness)
        
        return frame

def main():
    """Main function"""
    try:
        # Load configuration from config.ini
        config = CameraConfig("config.ini")
        
        # Check OpenCV version and capabilities
        logger.info(f"OpenCV Version: {cv2.__version__}")
        logger.info(f"Available backends: {[cv2.videoio_registry.getBackendName(b) for b in cv2.videoio_registry.getBackends()]}")
        
        # Display loaded configuration
        logger.info("=== Configuration Summary ===")
        camera_urls = config.get_camera_urls()
        camera_lowres_urls = config.get_camera_lowres_urls()
        for camera_id, url in camera_urls.items():
            main_info = f"Camera {camera_id} (MAIN): {url}"
            lowres_url = camera_lowres_urls.get(camera_id, "Not configured")
            lowres_info = f"Camera {camera_id} (LOW-RES): {lowres_url}"
            logger.info(main_info)
            logger.info(lowres_info)
        
        display_size = config.get_display_size()
        logger.info(f"Display size per camera: {display_size[0]}x{display_size[1]}")
        logger.info(f"Max display FPS: {config.get_max_display_fps()}")
        logger.info(f"Frame buffer size: {config.get_frame_buffer_size()}")
        logger.info(f"Max retries: {config.get_max_retries()}")
        logger.info(f"Restart delay: {config.get_restart_delay()}s")
        logger.info(f"Health check timeout: {config.get_health_check_timeout()}s")
        logger.info(f"Connection timeout: {config.get_connection_timeout_ms()}ms")
        logger.info(f"Read timeout: {config.get_read_timeout_ms()}ms")
        logger.info(f"Frame read timeout: {config.get_frame_read_timeout()}s")
        
        # Create and start viewer
        viewer = HighPerformanceViewer(config)
        viewer.start()
        
    except FileNotFoundError:
        logger.error("Config file 'config.ini' not found! Please create it with camera URLs.")
        logger.error("Example config.ini:")
        logger.error("[cameras]")
        logger.error("camera1_url = rtsp://admin:admin@192.168.0.4:8554/profile0")
        logger.error("camera2_url = rtsp://admin:admin@192.168.0.5:8554/profile0")
        logger.error("[display]")
        logger.error("single_camera_width = 640")
        logger.error("single_camera_height = 360")
        return
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        return
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Application error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'viewer' in locals():
            viewer.stop()

if __name__ == "__main__":
    main()
