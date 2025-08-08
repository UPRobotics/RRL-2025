#!/usr/bin/env python3
"""
Robust QR Code Reader with RTSP Stream Support
Displays bounding boxes around detected QR codes and shows data on console and overlay
"""

import cv2
import numpy as np
import threading
import queue
import time
import logging
import subprocess
import sys
import os
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from pyzbar import pyzbar
from PIL import Image, ImageDraw, ImageFont
import json
import traceback

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('qr_reader.log')
    ]
)
logger = logging.getLogger(__name__)

@dataclass
class QRCodeData:
    """Data structure for detected QR codes"""
    data: str
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    polygon: List[Tuple[int, int]]
    timestamp: float
    confidence: float = 1.0

class QRCodeReader:
    """Robust QR Code Reader with RTSP stream and video file support"""
    
    def __init__(self, source: str, window_name: str = "QR Code Reader"):
        self.source = source
        self.window_name = window_name
        self.running = False
        self.frame_queue = queue.Queue(maxsize=5)
        self.qr_data_queue = queue.Queue()
        
        # Determine if source is a file or stream
        self.is_file = os.path.isfile(source)
        self.is_rtsp = source.startswith('rtsp://')
        
        # Threading components
        self.capture_thread = None
        self.display_thread = None
        self.qr_detection_thread = None
        
        # Stream components
        self.cap = None
        self.ffmpeg_process = None
        
        # Detection settings
        self.show_fps = True
        self.show_timestamp = True
        self.bbox_color = (0, 255, 0)  # Green
        self.text_color = (255, 255, 255)  # White
        self.text_bg_color = (0, 0, 0)  # Black background
        
        # Performance tracking
        self.frame_count = 0
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0
        
        # QR code history (to avoid spam)
        self.qr_history = {}
        self.qr_history_timeout = 2.0  # seconds
        
        # QR code display persistence
        self.active_qr_codes = {}  # Store currently visible QR codes
        self.qr_display_timeout = 3.0  # How long to keep showing QR codes
        
    def start(self):
        """Start the QR code reader"""
        logger.info(f"Starting QR Code Reader for source: {self.source}")
        self.running = True
        
        # Start threads
        self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.qr_detection_thread = threading.Thread(target=self._detect_qr_codes, daemon=True)
        self.display_thread = threading.Thread(target=self._display_frames, daemon=True)
        
        self.capture_thread.start()
        self.qr_detection_thread.start()
        self.display_thread.start()
        
        logger.info("QR Code Reader started successfully")
        
    def stop(self):
        """Stop the QR code reader"""
        logger.info("Stopping QR Code Reader...")
        self.running = False
        
        # Close video capture
        if self.cap:
            self.cap.release()
            
        # Terminate FFmpeg process
        if self.ffmpeg_process:
            try:
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.ffmpeg_process.kill()
                
        # Close OpenCV windows
        cv2.destroyAllWindows()
        
        logger.info("QR Code Reader stopped")
        
    def _init_capture(self) -> bool:
        """Initialize video capture with fallback methods"""
        logger.info("Initializing video capture...")
        
        # Handle video file input
        if self.is_file:
            try:
                self.cap = cv2.VideoCapture(self.source)
                if self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        logger.info(f"Successfully initialized video file capture: {self.source}")
                        return True
                        
                self.cap.release()
                self.cap = None
                logger.error(f"Failed to open video file: {self.source}")
                return False
                
            except Exception as e:
                logger.error(f"Video file capture failed: {e}")
                return False
        
        # Handle RTSP stream input
        elif self.is_rtsp:
            # Method 1: Direct OpenCV RTSP
            try:
                self.cap = cv2.VideoCapture(self.source)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                if self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        logger.info("Successfully initialized direct RTSP capture")
                        return True
                        
                self.cap.release()
                self.cap = None
                
            except Exception as e:
                logger.warning(f"Direct RTSP capture failed: {e}")
                
            # Method 2: FFmpeg pipe
            try:
                return self._init_ffmpeg_capture()
            except Exception as e:
                logger.error(f"FFmpeg capture failed: {e}")
                return False
        
        # Handle other sources (webcam, etc.)
        else:
            try:
                # Try to convert to int for webcam index
                source_int = int(self.source)
                self.cap = cv2.VideoCapture(source_int)
            except ValueError:
                # Not an integer, try as string
                self.cap = cv2.VideoCapture(self.source)
                
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    logger.info(f"Successfully initialized capture for source: {self.source}")
                    return True
                    
            self.cap.release()
            self.cap = None
            logger.error(f"Failed to open source: {self.source}")
            return False
            
    def _init_ffmpeg_capture(self) -> bool:
        """Initialize FFmpeg-based capture"""
        logger.info("Initializing FFmpeg capture...")
        
        ffmpeg_cmd = [
            'ffmpeg',
            '-rtsp_transport', 'tcp',
            '-i', self.source,
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-an',  # no audio
            '-loglevel', 'error',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
            '-strict', 'experimental',
            '-vf', 'fps=30',
            '-'
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            
            # Test read
            raw_frame = self.ffmpeg_process.stdout.read(640 * 480 * 3)
            if len(raw_frame) > 0:
                logger.info("Successfully initialized FFmpeg capture")
                return True
                
        except Exception as e:
            logger.error(f"FFmpeg initialization failed: {e}")
            
        return False
        
    def _capture_frames(self):
        """Capture frames from source"""
        logger.info("Starting frame capture thread...")
        
        while self.running:
            try:
                if not self._init_capture():
                    logger.error("Failed to initialize capture, retrying in 5 seconds...")
                    time.sleep(5)
                    continue
                
                # Get video properties if it's a file
                fps = 30  # default fps
                if self.is_file and self.cap:
                    fps = self.cap.get(cv2.CAP_PROP_FPS)
                    if fps <= 0:
                        fps = 30
                    frame_delay = 1.0 / fps
                    logger.info(f"Video file FPS: {fps}, frame delay: {frame_delay:.3f}s")
                else:
                    frame_delay = 1.0 / 30  # default for streams
                    
                while self.running:
                    frame = self._read_frame()
                    if frame is not None:
                        # Add frame to queue (non-blocking)
                        try:
                            self.frame_queue.put(frame, timeout=0.1)
                        except queue.Full:
                            # Remove old frame and add new one
                            try:
                                self.frame_queue.get_nowait()
                                self.frame_queue.put(frame, timeout=0.1)
                            except queue.Empty:
                                pass
                        
                        # For video files, control playback speed
                        if self.is_file:
                            time.sleep(frame_delay)
                            
                    else:
                        if self.is_file:
                            # End of video file
                            logger.info("End of video file reached")
                            break
                        else:
                            logger.warning("Failed to read frame, reinitializing...")
                            break
                        
            except Exception as e:
                logger.error(f"Error in capture thread: {e}")
                time.sleep(1)
                
        logger.info("Frame capture thread stopped")
        
    def _read_frame(self) -> Optional[np.ndarray]:
        """Read a frame from the current capture method"""
        try:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    return frame
                    
            elif self.ffmpeg_process:
                # Calculate frame size (assuming 640x480 for now)
                frame_size = 640 * 480 * 3
                raw_frame = self.ffmpeg_process.stdout.read(frame_size)
                
                if len(raw_frame) == frame_size:
                    frame = np.frombuffer(raw_frame, dtype=np.uint8)
                    frame = frame.reshape((480, 640, 3))
                    return frame
                    
        except Exception as e:
            logger.error(f"Error reading frame: {e}")
            
        return None
        
    def _detect_qr_codes(self):
        """Detect QR codes in frames"""
        logger.info("Starting QR code detection thread...")
        
        while self.running:
            try:
                # Get frame from queue
                try:
                    frame = self.frame_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                    
                # Detect QR codes
                qr_codes = self._find_qr_codes(frame)
                
                if qr_codes:
                    for qr_code in qr_codes:
                        self._process_qr_code(qr_code)
                        
                # Clean up old QR history
                self._cleanup_qr_history()
                
            except Exception as e:
                logger.error(f"Error in QR detection thread: {e}")
                time.sleep(0.1)
                
        logger.info("QR code detection thread stopped")
        
    def _find_qr_codes(self, frame: np.ndarray) -> List[QRCodeData]:
        """Find QR codes in the frame"""
        qr_codes = []
        
        try:
            # Convert to grayscale for better detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Use pyzbar to detect QR codes
            decoded_objects = pyzbar.decode(gray)
            
            for obj in decoded_objects:
                # Extract data
                data = obj.data.decode('utf-8')
                
                # Get bounding box
                bbox = obj.rect
                x, y, w, h = bbox.left, bbox.top, bbox.width, bbox.height
                
                # Get polygon points
                polygon = [(point.x, point.y) for point in obj.polygon]
                
                qr_code = QRCodeData(
                    data=data,
                    bbox=(x, y, w, h),
                    polygon=polygon,
                    timestamp=time.time()
                )
                
                qr_codes.append(qr_code)
                
        except Exception as e:
            logger.error(f"Error detecting QR codes: {e}")
            
        return qr_codes
        
    def _process_qr_code(self, qr_code: QRCodeData):
        """Process detected QR code"""
        current_time = time.time()
        
        # Check if we've seen this QR code recently (for console spam prevention)
        should_print = True
        if qr_code.data in self.qr_history:
            last_seen = self.qr_history[qr_code.data]
            if current_time - last_seen < self.qr_history_timeout:
                should_print = False
                
        # Update history
        self.qr_history[qr_code.data] = current_time
        
        # Print to console (only if not spam)
        if should_print:
            logger.info(f"QR Code detected: {qr_code.data}")
            print(f"[{time.strftime('%H:%M:%S')}] QR Code: {qr_code.data}")
        
        # Always add to display queue (for continuous visual display)
        try:
            self.qr_data_queue.put(qr_code, timeout=0.1)
        except queue.Full:
            # Remove old QR code and add new one
            try:
                self.qr_data_queue.get_nowait()
                self.qr_data_queue.put(qr_code, timeout=0.1)
            except queue.Empty:
                pass
                
    def _cleanup_qr_history(self):
        """Clean up old QR code history"""
        current_time = time.time()
        expired_keys = []
        
        for data, timestamp in self.qr_history.items():
            if current_time - timestamp > self.qr_history_timeout * 2:
                expired_keys.append(data)
                
        for key in expired_keys:
            del self.qr_history[key]
            
    def _display_frames(self):
        """Display frames with QR code overlays"""
        logger.info("Starting display thread...")
        
        while self.running:
            try:
                # Get frame from queue
                try:
                    frame = self.frame_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                    
                # Make a copy for drawing
                display_frame = frame.copy()
                
                # Draw QR code overlays
                self._draw_qr_overlays(display_frame)
                
                # Draw info overlay
                self._draw_info_overlay(display_frame)
                
                # Show frame
                cv2.imshow(self.window_name, display_frame)
                
                # Update FPS counter
                self._update_fps_counter()
                
                # Check for quit key
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' or ESC
                    logger.info("Quit key pressed")
                    self.running = False
                    break
                    
            except Exception as e:
                logger.error(f"Error in display thread: {e}")
                time.sleep(0.1)
                
        logger.info("Display thread stopped")
        
    def _draw_qr_overlays(self, frame: np.ndarray):
        """Draw QR code bounding boxes and data"""
        current_time = time.time()
        
        # Get all new QR codes from the queue
        while True:
            try:
                qr_code = self.qr_data_queue.get_nowait()
                # Add to active QR codes with current timestamp
                self.active_qr_codes[qr_code.data] = qr_code
            except queue.Empty:
                break
        
        # Clean up expired QR codes
        expired_codes = []
        for data, qr_code in self.active_qr_codes.items():
            if current_time - qr_code.timestamp > self.qr_display_timeout:
                expired_codes.append(data)
        
        for data in expired_codes:
            del self.active_qr_codes[data]
        
        # Draw each active QR code
        for qr_code in self.active_qr_codes.values():
            self._draw_qr_code(frame, qr_code)
            
    def _draw_qr_code(self, frame: np.ndarray, qr_code: QRCodeData):
        """Draw a single QR code with bounding box and text"""
        x, y, w, h = qr_code.bbox
        
        # Calculate age-based opacity for fade effect
        current_time = time.time()
        age = current_time - qr_code.timestamp
        alpha = max(0.3, 1.0 - (age / self.qr_display_timeout))
        
        # Adjust colors based on alpha
        bbox_color = tuple(int(c * alpha) for c in self.bbox_color)
        text_color = tuple(int(c * alpha) for c in self.text_color)
        
        # Draw bounding box with thicker lines
        cv2.rectangle(frame, (x, y), (x + w, y + h), bbox_color, 3)
        
        # Draw corner markers for better visibility
        corner_size = 10
        cv2.rectangle(frame, (x, y), (x + corner_size, y + corner_size), bbox_color, -1)
        cv2.rectangle(frame, (x + w - corner_size, y), (x + w, y + corner_size), bbox_color, -1)
        cv2.rectangle(frame, (x, y + h - corner_size), (x + corner_size, y + h), bbox_color, -1)
        cv2.rectangle(frame, (x + w - corner_size, y + h - corner_size), (x + w, y + h), bbox_color, -1)
        
        # Draw polygon outline
        if len(qr_code.polygon) > 2:
            points = np.array(qr_code.polygon, dtype=np.int32)
            cv2.polylines(frame, [points], True, bbox_color, 2)
            
        # Prepare text
        text = qr_code.data
        if len(text) > 30:
            text = text[:27] + "..."
            
        # Get text size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Calculate text position
        text_x = x
        text_y = y - 15
        
        # Ensure text stays within frame bounds
        if text_y - text_height < 0:
            text_y = y + h + text_height + 15
            
        # Draw text background with some transparency
        bg_color = tuple(int(c * 0.8) for c in self.text_bg_color)
        cv2.rectangle(frame, 
                     (text_x - 5, text_y - text_height - 5),
                     (text_x + text_width + 5, text_y + baseline + 5),
                     bg_color, -1)
        
        # Draw text
        cv2.putText(frame, text, (text_x, text_y), font, font_scale, text_color, thickness)
        
        # Draw timestamp
        timestamp_text = time.strftime('%H:%M:%S', time.localtime(qr_code.timestamp))
        timestamp_y = text_y + text_height + 20
        
        cv2.putText(frame, timestamp_text, (text_x, timestamp_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
        
    def _draw_info_overlay(self, frame: np.ndarray):
        """Draw information overlay on frame"""
        height, width = frame.shape[:2]
        
        # Draw FPS
        if self.show_fps:
            fps_text = f"FPS: {self.current_fps:.1f}"
            cv2.putText(frame, fps_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        # Draw timestamp
        if self.show_timestamp:
            timestamp_text = time.strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(frame, timestamp_text, (10, height - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        # Draw instructions
        instructions = "Press 'q' or ESC to quit"
        cv2.putText(frame, instructions, (10, height - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
    def _update_fps_counter(self):
        """Update FPS counter"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_counter / (current_time - self.last_fps_time)
            self.fps_counter = 0
            self.last_fps_time = current_time
            
    def run(self):
        """Main run loop"""
        try:
            self.start()
            
            # Wait for threads to start
            time.sleep(1)
            
            # Keep main thread alive
            while self.running:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            traceback.print_exc()
        finally:
            self.stop()

def main():
    """Main function"""
    # Default source (can be RTSP URL, video file, or webcam index)
    default_source = "rtsp://192.168.0.203:554/stream1"
    
    # Allow source override from command line
    if len(sys.argv) > 1:
        source = sys.argv[1]
    else:
        source = default_source
        
    logger.info(f"Starting QR Code Reader with source: {source}")
    
    # Create and run QR code reader
    qr_reader = QRCodeReader(source)
    qr_reader.run()

if __name__ == "__main__":
    main()