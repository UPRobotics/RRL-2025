#!/usr/bin/env python3
"""
Multi-threaded RTSP Camera Viewer
Displays 4 RTSP camera streams in real-time with frame dropping and crash isolation.
"""

import cv2
import threading
import queue
import time
import numpy as np
from typing import Optional, Dict, Any
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RTSPCameraThread:
    """Thread-safe RTSP camera capture with frame dropping"""
    
    def __init__(self, camera_id: str, rtsp_url: str, max_queue_size: int = 2):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.max_queue_size = max_queue_size
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        self.running = False
        self.thread = None
        self.cap = None
        self.last_frame_time = 0
        self.fps_counter = 0
        self.fps = 0
        self.last_fps_time = time.time()
        
    def start(self):
        """Start the camera capture thread"""
        if self.thread is not None and self.thread.is_alive():
            logger.warning(f"Camera {self.camera_id} thread already running")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.thread.start()
        logger.info(f"Started camera thread for {self.camera_id}")
        
    def stop(self):
        """Stop the camera capture thread"""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
        logger.info(f"Stopped camera thread for {self.camera_id}")
        
    def _capture_frames(self):
        """Main capture loop running in separate thread"""
        retry_count = 0
        max_retries = 5
        
        while self.running and retry_count < max_retries:
            try:
                # Initialize camera with optimized settings for low latency
                self.cap = cv2.VideoCapture(self.rtsp_url)
                
                # Configure OpenCV for low latency
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer
                self.cap.set(cv2.CAP_PROP_FPS, 20)        # Set FPS
                
                if not self.cap.isOpened():
                    raise Exception(f"Failed to open camera {self.camera_id}")
                
                logger.info(f"Connected to camera {self.camera_id}")
                retry_count = 0  # Reset retry counter on successful connection
                
                while self.running:
                    ret, frame = self.cap.read()
                    
                    if not ret:
                        logger.warning(f"Failed to read frame from camera {self.camera_id}")
                        break
                        
                    # Update FPS counter
                    self._update_fps()
                    
                    # Drop frames if queue is full (keep only latest)
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()  # Remove old frame
                        except queue.Empty:
                            pass
                    
                    # Add timestamp and camera info to frame
                    timestamp = time.time()
                    frame_with_info = self._add_frame_info(frame, timestamp)
                    
                    try:
                        self.frame_queue.put_nowait(frame_with_info)
                    except queue.Full:
                        pass  # Skip if still full
                        
            except Exception as e:
                retry_count += 1
                logger.error(f"Camera {self.camera_id} error (attempt {retry_count}/{max_retries}): {e}")
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
                
                if retry_count < max_retries and self.running:
                    time.sleep(2)  # Wait before retry
                    
        if retry_count >= max_retries:
            logger.error(f"Camera {self.camera_id} failed after {max_retries} attempts")
            
    def _update_fps(self):
        """Update FPS calculation"""
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.last_fps_time = current_time
            
    def _add_frame_info(self, frame, timestamp):
        """Add camera info and timestamp to frame"""
        if frame is None:
            return None
            
        # Add text overlay with camera info
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)  # Green
        thickness = 2
        
        # Camera ID
        cv2.putText(frame, f"Camera {self.camera_id}", (10, 30), 
                   font, font_scale, color, thickness)
        
        # FPS
        cv2.putText(frame, f"FPS: {self.fps}", (10, 60), 
                   font, font_scale, color, thickness)
        
        # Timestamp
        time_str = time.strftime("%H:%M:%S", time.localtime(timestamp))
        cv2.putText(frame, time_str, (10, 90), 
                   font, font_scale, color, thickness)
        
        return frame
        
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the latest frame, dropping older ones"""
        latest_frame = None
        
        # Get all available frames, keeping only the latest
        while True:
            try:
                latest_frame = self.frame_queue.get_nowait()
            except queue.Empty:
                break
                
        return latest_frame

class MultiCameraViewer:
    """Main application class for displaying multiple RTSP cameras"""
    
    def __init__(self, camera_configs: Dict[str, str]):
        self.camera_configs = camera_configs
        self.cameras = {}
        self.running = False
        self.display_size = (640, 480)  # Size for each camera display
        
    def start(self):
        """Start all camera threads and display loop"""
        logger.info("Starting multi-camera viewer...")
        
        # Initialize and start all camera threads
        for camera_id, rtsp_url in self.camera_configs.items():
            camera = RTSPCameraThread(camera_id, rtsp_url)
            self.cameras[camera_id] = camera
            camera.start()
            
        # Wait a moment for cameras to initialize
        time.sleep(2)
        
        self.running = True
        self._display_loop()
        
    def stop(self):
        """Stop all camera threads"""
        logger.info("Stopping multi-camera viewer...")
        self.running = False
        
        for camera in self.cameras.values():
            camera.stop()
            
        cv2.destroyAllWindows()
        
    def _display_loop(self):
        """Main display loop"""
        while self.running:
            frames = {}
            
            # Collect latest frames from all cameras
            for camera_id, camera in self.cameras.items():
                frame = camera.get_latest_frame()
                if frame is not None:
                    # Resize frame for display
                    frame_resized = cv2.resize(frame, self.display_size)
                    frames[camera_id] = frame_resized
                else:
                    # Create black frame with "No Signal" text if camera is not working
                    black_frame = np.zeros((self.display_size[1], self.display_size[0], 3), dtype=np.uint8)
                    cv2.putText(black_frame, f"Camera {camera_id}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(black_frame, "No Signal", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    frames[camera_id] = black_frame
            
            # Create grid layout (2x2 for 4 cameras)
            if len(frames) >= 4:
                camera_ids = list(frames.keys())[:4]
                
                # Top row
                top_row = np.hstack([frames[camera_ids[0]], frames[camera_ids[1]]])
                # Bottom row
                bottom_row = np.hstack([frames[camera_ids[2]], frames[camera_ids[3]]])
                # Combined view
                combined_frame = np.vstack([top_row, bottom_row])
                
                cv2.imshow("Multi-Camera View", combined_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC to quit
                break
                
        self.stop()

def main():
    """Main function"""
    # Camera configurations - update these URLs with your actual camera IPs
    camera_configs = {
        "1": "rtsp://admin:admin@192.168.0.4:8554/profile0",
        "2": "rtsp://admin:admin@192.168.0.5:8554/profile0", 
        "3": "rtsp://admin:admin@192.168.0.6:8554/profile0",
        "4": "rtsp://admin:admin@192.168.0.7:8554/profile0"  # Added 4th camera
    }
    
    viewer = MultiCameraViewer(camera_configs)
    
    try:
        viewer.start()
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Application error: {e}")
    finally:
        viewer.stop()

if __name__ == "__main__":
    main()