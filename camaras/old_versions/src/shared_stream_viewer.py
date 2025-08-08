#!/usr/bin/env python3
"""
Shared Stream Multi-Camera Viewer
Uses a single RTSP connection to simulate multiple cameras
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
from typing import Dict, List, Optional

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SharedStreamSource:
    """Single RTSP stream source that feeds multiple virtual cameras"""
    
    def __init__(self, rtsp_url: str, target_size: tuple = (640, 360)):
        self.rtsp_url = rtsp_url
        self.target_size = target_size
        
        # Threading
        self.running = False
        self.thread = None
        self.process = None
        
        # Subscribers (virtual cameras)
        self.subscribers: Dict[str, queue.Queue] = {}
        self.subscribers_lock = threading.Lock()
        
        # Statistics
        self.fps_counter = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.total_frames = 0
        self.last_frame_time = time.time()
        
        # Frame buffer size calculation
        self.frame_buffer_size = self.target_size[0] * self.target_size[1] * 3  # RGB
        
        logger.info(f"SharedStreamSource initialized for {rtsp_url}")
        
    def subscribe(self, camera_id: str) -> queue.Queue:
        """Subscribe a virtual camera to receive frames"""
        with self.subscribers_lock:
            frame_queue = queue.Queue(maxsize=1)  # Ultra-low latency
            self.subscribers[camera_id] = frame_queue
            logger.info(f"Camera {camera_id} subscribed to shared stream")
            return frame_queue
            
    def unsubscribe(self, camera_id: str):
        """Unsubscribe a virtual camera"""
        with self.subscribers_lock:
            if camera_id in self.subscribers:
                del self.subscribers[camera_id]
                logger.info(f"Camera {camera_id} unsubscribed from shared stream")
                
    def start(self):
        """Start the shared stream"""
        if self.thread and self.thread.is_alive():
            logger.warning("SharedStreamSource already running")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()
        logger.info("SharedStreamSource started")
        
    def stop(self):
        """Stop the shared stream"""
        self.running = False
        
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            except Exception as e:
                logger.warning(f"SharedStreamSource process cleanup error: {e}")
                
        if self.thread:
            self.thread.join(timeout=3)
            
        logger.info(f"SharedStreamSource stopped - {self.total_frames} frames processed")
        
    def _stream_loop(self):
        """Main streaming loop"""
        consecutive_failures = 0
        max_failures = 10
        
        while self.running:
            try:
                if consecutive_failures > 0:
                    delay = min(consecutive_failures * 2, 10)
                    logger.info(f"SharedStreamSource reconnecting in {delay}s")
                    time.sleep(delay)
                
                # Create FFmpeg command
                process = (
                    ffmpeg
                    .input(
                        self.rtsp_url,
                        rtsp_transport='tcp',
                        fflags='nobuffer',
                        flags='low_delay',
                        probesize=65536,
                        analyzeduration=1000000,
                        max_delay=500000,
                        reconnect=1,
                        reconnect_streamed=1,
                        reconnect_delay_max=5
                    )
                    .video
                    .filter('scale', self.target_size[0], self.target_size[1])
                    .output(
                        'pipe:',
                        format='rawvideo',
                        pix_fmt='rgb24',
                        vsync=0,
                        loglevel='error'
                    )
                    .run_async(pipe_stdout=True, pipe_stderr=subprocess.PIPE, quiet=False)
                )
                
                self.process = process
                logger.info("SharedStreamSource FFmpeg process started successfully")
                consecutive_failures = 0
                
                # Read and distribute frames
                while self.running and self.process.poll() is None:
                    try:
                        # Read raw frame data
                        raw_frame = self.process.stdout.read(self.frame_buffer_size)
                        
                        if len(raw_frame) == 0:
                            logger.warning("SharedStreamSource EOF received")
                            break
                            
                        if len(raw_frame) != self.frame_buffer_size:
                            continue  # Skip incomplete frames
                            
                        # Convert to numpy array
                        frame = np.frombuffer(raw_frame, dtype=np.uint8)
                        frame = frame.reshape((self.target_size[1], self.target_size[0], 3))
                        
                        # Convert RGB to BGR for OpenCV
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        
                        # Update statistics
                        self.total_frames += 1
                        self.last_frame_time = time.time()
                        self._update_fps()
                        
                        # Distribute frame to all subscribers
                        self._distribute_frame(frame)
                        
                    except Exception as e:
                        logger.warning(f"SharedStreamSource frame processing error: {e}")
                        break
                        
                # Process ended
                if self.running:
                    consecutive_failures += 1
                    if consecutive_failures > max_failures:
                        logger.error("SharedStreamSource failed too many times, stopping")
                        break
                    logger.warning(f"SharedStreamSource ended, will restart (failure {consecutive_failures})")
                    
            except Exception as e:
                consecutive_failures += 1
                logger.error(f"SharedStreamSource error: {e}")
                if consecutive_failures > max_failures:
                    break
                    
        logger.info("SharedStreamSource stream loop ended")
        
    def _update_fps(self):
        """Update FPS counter"""
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.last_fps_time = current_time
            
    def _distribute_frame(self, frame):
        """Distribute frame to all subscribers"""
        with self.subscribers_lock:
            for camera_id, frame_queue in self.subscribers.items():
                # Create a copy of the frame for each camera
                frame_copy = frame.copy()
                
                # Drop old frames to maintain low latency
                while not frame_queue.empty():
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        break
                        
                # Add new frame
                try:
                    frame_queue.put_nowait(frame_copy)
                except queue.Full:
                    pass  # Skip if queue is full
                    
    def is_healthy(self):
        """Check if the shared stream is healthy"""
        return (self.running and 
                self.process and 
                self.process.poll() is None and
                time.time() - self.last_frame_time < 10)

class VirtualCamera:
    """Virtual camera that receives frames from shared stream"""
    
    def __init__(self, camera_id: str, shared_source: SharedStreamSource):
        self.camera_id = camera_id
        self.shared_source = shared_source
        self.frame_queue = None
        
        # Statistics
        self.frames_received = 0
        self.frames_displayed = 0
        
    def start(self):
        """Start receiving frames from shared source"""
        self.frame_queue = self.shared_source.subscribe(self.camera_id)
        logger.info(f"VirtualCamera {self.camera_id} started")
        
    def stop(self):
        """Stop receiving frames"""
        if self.frame_queue:
            self.shared_source.unsubscribe(self.camera_id)
            self.frame_queue = None
        logger.info(f"VirtualCamera {self.camera_id} stopped - {self.frames_received} received, {self.frames_displayed} displayed")
        
    def get_frame(self):
        """Get latest frame with camera-specific overlay"""
        if not self.frame_queue:
            return None
            
        try:
            frame = self.frame_queue.get_nowait()
            self.frames_received += 1
            
            # Add camera-specific overlay
            self._add_overlay(frame)
            self.frames_displayed += 1
            
            return frame
        except queue.Empty:
            return None
            
    def _add_overlay(self, frame):
        """Add camera-specific overlay"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        color = (0, 255, 0)
        thickness = 1
        
        # Create semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (200, 80), (0, 0, 0), -1)
        frame[:] = cv2.addWeighted(frame, 0.8, overlay, 0.2, 0)
        
        # Add text
        texts = [
            f"Camera {self.camera_id}",
            f"Source FPS: {self.shared_source.fps}",
            f"Received: {self.frames_received}",
            f"Displayed: {self.frames_displayed}"
        ]
        
        for i, text in enumerate(texts):
            y = 20 + i * 15
            cv2.putText(frame, text, (10, y), font, font_scale, color, thickness)
            
    def is_healthy(self):
        """Check if camera is healthy"""
        return self.shared_source.is_healthy()

class SharedStreamViewer:
    """Viewer using shared stream for multiple virtual cameras"""
    
    def __init__(self, config_file="config.ini"):
        self.config = configparser.ConfigParser()
        self.config.read(config_file)
        
        # Load camera configurations
        self.camera_configs = self._load_camera_configs()
        
        # Create shared stream source
        if self.camera_configs:
            # Use the first camera's URL as the shared source
            first_url = list(self.camera_configs.values())[0]
            self.shared_source = SharedStreamSource(first_url, (640, 360))
        else:
            raise ValueError("No camera configurations found")
            
        # Create virtual cameras
        self.cameras = {}
        for camera_id in self.camera_configs.keys():
            self.cameras[camera_id] = VirtualCamera(camera_id, self.shared_source)
            
        # Display settings
        self.running = False
        self.window_name = "Shared Stream Multi-Camera Viewer"
        
        logger.info(f"SharedStreamViewer initialized with {len(self.cameras)} virtual cameras")
        
    def _load_camera_configs(self):
        """Load camera configurations from config file"""
        configs = {}
        if 'cameras' in self.config:
            for key, value in self.config['cameras'].items():
                if key.startswith('camera') and key.endswith('_url') and not key.endswith('_lowres'):
                    camera_num = key.replace('camera', '').replace('_url', '')
                    configs[camera_num] = value
        return configs
        
    def start(self):
        """Start the viewer"""
        logger.info("Starting shared stream viewer...")
        
        # Check FFmpeg availability
        try:
            subprocess.run(['ffmpeg', '-version'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            logger.error("FFmpeg not found! Please install FFmpeg")
            return False
            
        # Start shared source
        self.shared_source.start()
        time.sleep(2)  # Give shared source time to start
        
        # Start virtual cameras
        for camera in self.cameras.values():
            camera.start()
            
        # Start display loop
        self.running = True
        self._display_loop()
        
        return True
        
    def stop(self):
        """Stop the viewer"""
        logger.info("Stopping shared stream viewer...")
        self.running = False
        
        # Stop virtual cameras
        for camera in self.cameras.values():
            camera.stop()
            
        # Stop shared source
        self.shared_source.stop()
        
        cv2.destroyAllWindows()
        
    def _display_loop(self):
        """Main display loop"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
        logger.info("=== Controls ===")
        logger.info("ESC or 'q' - Quit")
        logger.info("'s' - Show statistics")
        logger.info("================")
        
        while self.running:
            try:
                # Collect frames from all virtual cameras
                frames = {}
                for camera_id, camera in self.cameras.items():
                    frame = camera.get_frame()
                    if frame is not None:
                        frames[camera_id] = frame
                        
                # Create display
                display_frame = self._create_display(frames)
                
                if display_frame is not None:
                    cv2.imshow(self.window_name, display_frame)
                    
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # ESC
                    break
                elif key == ord('s'):
                    self._show_statistics()
                    
            except Exception as e:
                logger.error(f"Display loop error: {e}")
                break
                
        self.stop()
        
    def _create_display(self, frames):
        """Create display based on number of cameras"""
        if not frames:
            # No frames available
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Starting cameras...", (200, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return blank
            
        camera_ids = sorted(frames.keys())
        num_cameras = len(camera_ids)
        
        if num_cameras == 1:
            # Single camera - full screen
            return frames[camera_ids[0]]
            
        elif num_cameras <= 4:
            # 2x2 grid
            return self._create_grid_2x2(frames, camera_ids)
            
        else:
            # 2x4 grid for 5-8 cameras
            return self._create_grid_2x4(frames, camera_ids)
            
    def _create_grid_2x2(self, frames, camera_ids):
        """Create 2x2 grid display"""
        # Resize frames to fit grid
        resized_frames = []
        for camera_id in camera_ids[:4]:  # Max 4 cameras
            frame = frames.get(camera_id)
            if frame is not None:
                resized = cv2.resize(frame, (320, 180))
                resized_frames.append(resized)
            else:
                # Black placeholder
                black = np.zeros((180, 320, 3), dtype=np.uint8)
                cv2.putText(black, f"Cam {camera_id}", (100, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                resized_frames.append(black)
                
        # Pad to 4 frames
        while len(resized_frames) < 4:
            black = np.zeros((180, 320, 3), dtype=np.uint8)
            resized_frames.append(black)
            
        # Create grid
        top_row = np.hstack([resized_frames[0], resized_frames[1]])
        bottom_row = np.hstack([resized_frames[2], resized_frames[3]])
        return np.vstack([top_row, bottom_row])
        
    def _create_grid_2x4(self, frames, camera_ids):
        """Create 2x4 grid display for 5-8 cameras"""
        # Resize frames to fit grid
        resized_frames = []
        for camera_id in camera_ids[:8]:  # Max 8 cameras
            frame = frames.get(camera_id)
            if frame is not None:
                resized = cv2.resize(frame, (160, 90))
                resized_frames.append(resized)
            else:
                # Black placeholder
                black = np.zeros((90, 160, 3), dtype=np.uint8)
                cv2.putText(black, f"Cam {camera_id}", (50, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                resized_frames.append(black)
                
        # Pad to 8 frames
        while len(resized_frames) < 8:
            black = np.zeros((90, 160, 3), dtype=np.uint8)
            resized_frames.append(black)
            
        # Create grid (2 rows, 4 columns)
        top_row = np.hstack(resized_frames[:4])
        bottom_row = np.hstack(resized_frames[4:8])
        return np.vstack([top_row, bottom_row])
        
    def _show_statistics(self):
        """Show statistics"""
        logger.info("=== Statistics ===")
        logger.info(f"Shared Source FPS: {self.shared_source.fps}")
        logger.info(f"Total Source Frames: {self.shared_source.total_frames}")
        logger.info(f"Shared Source Healthy: {self.shared_source.is_healthy()}")
        logger.info("Virtual Cameras:")
        for camera_id, camera in self.cameras.items():
            logger.info(f"  Camera {camera_id}: {camera.frames_received} received, {camera.frames_displayed} displayed")
        logger.info("==================")

def main():
    """Main function"""
    try:
        viewer = SharedStreamViewer()
        return 0 if viewer.start() else 1
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 0
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
