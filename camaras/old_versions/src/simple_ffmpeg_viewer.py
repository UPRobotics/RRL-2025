#!/usr/bin/env python3
"""
Simple FFmpeg Ultra-Low Latency Camera Viewer
Optimized for single camera with scalable architecture
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

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SimpleFfmpegCamera:
    """Ultra-low latency camera using FFmpeg with minimal overhead"""
    
    def __init__(self, camera_id: str, rtsp_url: str, target_size: tuple = (640, 360)):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.target_size = target_size
        
        # Threading
        self.running = False
        self.thread = None
        self.process = None
        
        # Ultra-low latency frame queue (size 1 for minimal latency)
        self.frame_queue = queue.Queue(maxsize=1)
        
        # Statistics
        self.fps_counter = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.total_frames = 0
        self.dropped_frames = 0
        self.last_frame_time = time.time()
        
        # Frame buffer size calculation
        self.frame_buffer_size = self.target_size[0] * self.target_size[1] * 3  # RGB
        
        logger.info(f"Camera {self.camera_id} initialized for ultra-low latency")
        
    def start(self):
        """Start the camera stream"""
        if self.thread and self.thread.is_alive():
            logger.warning(f"Camera {self.camera_id} already running")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()
        logger.info(f"Camera {self.camera_id} started")
        
    def stop(self):
        """Stop the camera stream"""
        self.running = False
        
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            except Exception as e:
                logger.warning(f"Camera {self.camera_id} process cleanup error: {e}")
                
        if self.thread:
            self.thread.join(timeout=3)
            
        logger.info(f"Camera {self.camera_id} stopped - {self.total_frames} frames, {self.dropped_frames} dropped")
        
    def _stream_loop(self):
        """Main streaming loop with ultra-low latency FFmpeg"""
        consecutive_failures = 0
        max_failures = 5
        
        while self.running:
            try:
                # Add delay for connection stability
                if consecutive_failures > 0:
                    delay = min(consecutive_failures * 2, 10)  # Max 10 second delay
                    logger.info(f"Camera {self.camera_id} reconnecting in {delay}s (attempt {consecutive_failures + 1})")
                    time.sleep(delay)
                
                # Create FFmpeg command with more stable parameters
                process = (
                    ffmpeg
                    .input(
                        self.rtsp_url,
                        rtsp_transport='tcp',
                        fflags='nobuffer',
                        flags='low_delay',
                        probesize=65536,  # Increased for stability
                        analyzeduration=1000000,  # Increased for stability
                        max_delay=500000,  # Allow some delay for stability
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
                        loglevel='error'  # Reduce log noise
                    )
                    .run_async(pipe_stdout=True, pipe_stderr=subprocess.PIPE, quiet=False)
                )
                
                self.process = process
                logger.info(f"Camera {self.camera_id} FFmpeg process started successfully")
                consecutive_failures = 0
                
                # Read frames with better error handling
                frame_count = 0
                while self.running and self.process.poll() is None:
                    try:
                        # Read raw frame data with timeout
                        raw_frame = self.process.stdout.read(self.frame_buffer_size)
                        
                        if len(raw_frame) == 0:
                            logger.warning(f"Camera {self.camera_id} EOF received")
                            break
                            
                        if len(raw_frame) != self.frame_buffer_size:
                            logger.warning(f"Camera {self.camera_id} incomplete frame: {len(raw_frame)}/{self.frame_buffer_size}")
                            continue  # Skip incomplete frames instead of breaking
                            
                        # Convert to numpy array
                        frame = np.frombuffer(raw_frame, dtype=np.uint8)
                        frame = frame.reshape((self.target_size[1], self.target_size[0], 3))
                        
                        # Convert RGB to BGR for OpenCV
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        
                        # Add minimal overlay
                        self._add_overlay(frame)
                        
                        # Update statistics
                        self.total_frames += 1
                        self.last_frame_time = time.time()
                        self._update_fps()
                        frame_count += 1
                        
                        # Ultra-aggressive frame dropping - always keep latest frame
                        while not self.frame_queue.empty():
                            try:
                                self.frame_queue.get_nowait()
                                self.dropped_frames += 1
                            except queue.Empty:
                                break
                                
                        # Add new frame (non-blocking)
                        try:
                            self.frame_queue.put_nowait(frame)
                        except queue.Full:
                            self.dropped_frames += 1
                            
                    except Exception as e:
                        logger.warning(f"Camera {self.camera_id} frame processing error: {e}")
                        # Don't break immediately, try to continue
                        if frame_count == 0:
                            break  # If we haven't received any frames, break
                        continue
                        
                # Process ended
                if self.running:
                    consecutive_failures += 1
                    if consecutive_failures > max_failures:
                        logger.error(f"Camera {self.camera_id} failed too many times, stopping")
                        break
                    logger.warning(f"Camera {self.camera_id} stream ended, will restart (failure {consecutive_failures}/{max_failures})")
                    
            except Exception as e:
                consecutive_failures += 1
                logger.error(f"Camera {self.camera_id} setup error: {e}")
                if consecutive_failures > max_failures:
                    logger.error(f"Camera {self.camera_id} failed too many times, stopping")
                    break
                    
        logger.info(f"Camera {self.camera_id} stream loop ended")
        
    def _update_fps(self):
        """Update FPS counter"""
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.last_fps_time = current_time
            
    def _add_overlay(self, frame):
        """Add minimal overlay for debugging"""
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
            f"FPS: {self.fps}",
            f"Total: {self.total_frames}",
            f"Dropped: {self.dropped_frames}"
        ]
        
        for i, text in enumerate(texts):
            y = 20 + i * 15
            cv2.putText(frame, text, (10, y), font, font_scale, color, thickness)
            
    def get_frame(self):
        """Get latest frame"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
            
    def is_healthy(self):
        """Check if camera is healthy"""
        return (self.running and 
                self.process and 
                self.process.poll() is None and
                time.time() - self.last_frame_time < 10)

class SimpleViewer:
    """Simple viewer supporting 1-8 cameras"""
    
    def __init__(self, config_file="config.ini"):
        self.config = configparser.ConfigParser()
        self.config.read(config_file)
        
        # Load camera configurations
        self.cameras = {}
        self.camera_configs = self._load_camera_configs()
        
        # Display settings
        self.single_cam_size = (640, 360)
        self.running = False
        self.window_name = "Ultra-Low Latency Camera Viewer"
        
        logger.info(f"Viewer initialized with {len(self.camera_configs)} cameras")
        
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
        logger.info("Starting ultra-low latency viewer...")
        
        # Check FFmpeg availability
        try:
            subprocess.run(['ffmpeg', '-version'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            logger.error("FFmpeg not found! Please install FFmpeg")
            return False
            
        # Initialize cameras with staggered startup
        for i, (camera_id, rtsp_url) in enumerate(self.camera_configs.items()):
            camera = SimpleFfmpegCamera(camera_id, rtsp_url, self.single_cam_size)
            self.cameras[camera_id] = camera
            camera.start()
            # Stagger camera starts to reduce connection conflicts
            if i < len(self.camera_configs) - 1:  # Don't sleep after last camera
                time.sleep(2.0)  # Longer delay between camera starts
            
        # Start display loop
        self.running = True
        self._display_loop()
        
        return True
        
    def stop(self):
        """Stop the viewer"""
        logger.info("Stopping viewer...")
        self.running = False
        
        for camera in self.cameras.values():
            camera.stop()
            
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
                # Collect frames from all cameras
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
            cv2.putText(blank, "No camera feed", (200, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
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
        """Show camera statistics"""
        logger.info("=== Camera Statistics ===")
        for camera_id, camera in self.cameras.items():
            logger.info(f"Camera {camera_id}:")
            logger.info(f"  FPS: {camera.fps}")
            logger.info(f"  Total Frames: {camera.total_frames}")
            logger.info(f"  Dropped Frames: {camera.dropped_frames}")
            logger.info(f"  Healthy: {camera.is_healthy()}")
        logger.info("========================")

def main():
    """Main function"""
    try:
        viewer = SimpleViewer()
        return 0 if viewer.start() else 1
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 0
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
