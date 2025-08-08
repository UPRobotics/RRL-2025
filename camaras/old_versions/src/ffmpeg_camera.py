#!/usr/bin/env python3
"""
FFmpeg-Python Ultra-Low Latency Camera Viewer
Uses ffmpeg-python for minimal latency with grid view and fullscreen support
"""

import ffmpeg
import cv2
import threading
import queue
import time
import numpy as np
from typing import Optional, Dict, Any, Tuple
import logging
import sys
import os
import subprocess
import signal
from contextlib import contextmanager

logger = logging.getLogger(__name__)

class FFmpegCamera:
    """Ultra-low latency camera using ffmpeg-python"""
    
    def __init__(self, camera_id: str, rtsp_url: str, target_size: Tuple[int, int] = (640, 360),
                 max_queue_size: int = 1, rtsp_url_lowres: str = None):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.rtsp_url_lowres = rtsp_url_lowres
        self.current_url = rtsp_url
        self.is_lowres_mode = False
        self.target_size = target_size
        self.max_queue_size = max_queue_size
        
        # Threading and process management
        self.running = False
        self.thread = None
        self.process = None
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        
        # Performance metrics
        self.fps_counter = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.dropped_frames = 0
        self.total_frames = 0
        self.last_frame_time = time.time()
        self.connection_healthy = False
        
        # Frame buffer for reading raw data
        self.frame_buffer_size = self.target_size[0] * self.target_size[1] * 3  # RGB
        
    def start(self):
        """Start FFmpeg capture"""
        if self.thread is not None and self.thread.is_alive():
            logger.warning(f"Camera {self.camera_id} thread already running")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info(f"Started FFmpeg camera thread for {self.camera_id}")
        
    def stop(self):
        """Stop FFmpeg capture"""
        self.running = False
        self.connection_healthy = False
        
        if self.process is not None:
            try:
                # Graceful shutdown
                self.process.terminate()
                try:
                    self.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    # Force kill if needed
                    self.process.kill()
                    self.process.wait()
            except Exception as e:
                logger.warning(f"Camera {self.camera_id} process cleanup error: {e}")
            finally:
                self.process = None
                
        if self.thread is not None:
            self.thread.join(timeout=5.0)
            if self.thread.is_alive():
                logger.warning(f"Camera {self.camera_id} thread did not stop gracefully")
                
        logger.info(f"Stopped FFmpeg camera {self.camera_id} - Dropped {self.dropped_frames}/{self.total_frames} frames")
        
    def _create_ffmpeg_input(self):
        """Create FFmpeg input stream with ultra-low latency settings"""
        input_stream = ffmpeg.input(
            self.current_url,
            # Ultra-low latency RTSP settings
            rtsp_transport='tcp',
            probesize=1024,
            analyzeduration=1000000,
            fflags='nobuffer+fastseek+flush_packets',
            flags='low_delay',
            # Error handling for stability
            err_detect='ignore_err',
            # Frame dropping for real-time
            framedrop=True,
            max_delay=0,
            reorder_queue_size=0,
        )
        
        # Apply video filters for scaling and format conversion
        video = input_stream.video.filter(
            'scale', 
            self.target_size[0], 
            self.target_size[1],
            # Use fast scaling for minimal latency
            flags='fast_bilinear'
        ).filter(
            'fps', 
            fps=30,  # Target FPS
            round='up'
        )
        
        return video
        
    def _capture_loop(self):
        """Main capture loop using FFmpeg"""
        consecutive_failures = 0
        max_retries = 10
        restart_delay = 2.0
        
        while self.running:
            try:
                if consecutive_failures > 0:
                    logger.info(f"Camera {self.camera_id} attempting reconnection (attempt {consecutive_failures})")
                    if consecutive_failures >= max_retries:
                        logger.error(f"Camera {self.camera_id} failed after {max_retries} attempts")
                        break
                    time.sleep(restart_delay)
                
                # Create FFmpeg process
                video_input = self._create_ffmpeg_input()
                
                # Output to pipe with RGB24 format
                process = ffmpeg.run_async(
                    ffmpeg.output(
                        video_input,
                        'pipe:',
                        format='rawvideo',
                        pix_fmt='rgb24',
                        vsync=0,  # No video sync for minimal latency
                        # Additional low-latency options
                        preset='ultrafast',
                        tune='zerolatency'
                    ),
                    pipe_stdout=True,
                    pipe_stderr=True,
                    quiet=True  # Suppress FFmpeg output
                )
                
                self.process = process
                consecutive_failures = 0
                self.connection_healthy = True
                
                logger.info(f"Camera {self.camera_id} FFmpeg process started successfully")
                
                # Read frames from FFmpeg output
                while self.running and self.process.poll() is None:
                    try:
                        # Read raw frame data
                        raw_frame = self.process.stdout.read(self.frame_buffer_size)
                        
                        if len(raw_frame) != self.frame_buffer_size:
                            # Incomplete frame or EOF
                            break
                            
                        # Convert raw bytes to numpy array
                        frame = np.frombuffer(raw_frame, dtype=np.uint8)
                        frame = frame.reshape((self.target_size[1], self.target_size[0], 3))
                        
                        # Convert RGB to BGR for OpenCV compatibility
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        
                        # Add overlay with camera info
                        frame = self._add_overlay(frame)
                        
                        # Update metrics
                        self.total_frames += 1
                        self.last_frame_time = time.time()
                        self._update_fps()
                        
                        # Ultra-aggressive frame dropping - always use latest frame
                        while not self.frame_queue.empty():
                            try:
                                self.frame_queue.get_nowait()
                                self.dropped_frames += 1
                            except queue.Empty:
                                break
                                
                        # Add new frame
                        try:
                            self.frame_queue.put_nowait(frame)
                        except queue.Full:
                            self.dropped_frames += 1
                            
                    except Exception as e:
                        logger.warning(f"Camera {self.camera_id} frame processing error: {e}")
                        break
                        
                # Process ended
                if self.running:
                    consecutive_failures += 1
                    self.connection_healthy = False
                    logger.warning(f"Camera {self.camera_id} FFmpeg process ended, will retry")
                    
            except Exception as e:
                consecutive_failures += 1
                self.connection_healthy = False
                logger.error(f"Camera {self.camera_id} FFmpeg setup error: {e}")
                
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
            f"Cam {self.camera_id} ({stream_info}) [FFmpeg]",
            f"FPS: {self.fps}",
            f"Drop: {self.dropped_frames}",
            f"Age: {frame_age:.1f}s",
            f"Health: {'OK' if self.connection_healthy else 'FAIL'}"
        ]
        
        colors = [
            (0, 255, 0) if stream_info == "MAIN" else (255, 255, 0),  # Camera ID
            (255, 255, 255),  # FPS
            (0, 255, 255),  # Dropped frames
            (255, 255, 0) if frame_age < 1.0 else (255, 0, 0),  # Frame age
            (0, 255, 0) if self.connection_healthy else (0, 0, 255)  # Health
        ]
        
        for i, (text, color) in enumerate(zip(texts, colors)):
            y_pos = 20 + i * 15
            cv2.putText(frame, text, (10, y_pos), font, font_scale, color, thickness)
            
        return frame
        
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the most recent frame"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            if not self.is_healthy():
                time_since_last_frame = time.time() - self.last_frame_time
                
                if time_since_last_frame > 10.0:  # 10 seconds grace period
                    # Create status frame
                    black_frame = np.zeros((self.target_size[1], self.target_size[0], 3), dtype=np.uint8)
                    status = "NO SIGNAL" if time_since_last_frame > 30 else "RECONNECTING..."
                    color = (0, 0, 255) if time_since_last_frame > 30 else (0, 255, 255)
                    
                    cv2.putText(black_frame, f"FFmpeg Camera {self.camera_id}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.putText(black_frame, status, (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    return black_frame
                    
            return None
            
    def is_healthy(self) -> bool:
        """Check if camera connection is healthy"""
        return (self.connection_healthy and 
                time.time() - self.last_frame_time < 20.0 and
                self.process is not None and 
                self.process.poll() is None)
        
    def switch_stream_quality(self) -> bool:
        """Switch between main and low-resolution streams"""
        if not self.rtsp_url_lowres:
            logger.warning(f"Camera {self.camera_id} has no low-res URL configured")
            return False
            
        # Stop current stream
        old_running = self.running
        self.running = False
        
        # Wait for current operations to complete
        time.sleep(0.2)
        
        try:
            # Clear frame queue
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    break
            
            # Toggle URL
            if self.is_lowres_mode:
                self.current_url = self.rtsp_url
                self.is_lowres_mode = False
                logger.info(f"Camera {self.camera_id} switching to MAIN stream")
            else:
                self.current_url = self.rtsp_url_lowres
                self.is_lowres_mode = True
                logger.info(f"Camera {self.camera_id} switching to LOW-RES stream")
            
            # Kill current process
            if self.process is not None:
                try:
                    self.process.terminate()
                    self.process.wait(timeout=2.0)
                except:
                    try:
                        self.process.kill()
                        self.process.wait()
                    except:
                        pass
                self.process = None
            
            # Restart
            self.running = old_running
            return True
            
        except Exception as e:
            logger.error(f"Camera {self.camera_id} stream switch error: {e}")
            self.running = old_running
            return False
        
    def get_current_stream_info(self) -> str:
        """Get current stream quality info"""
        return "LOW-RES" if self.is_lowres_mode else "MAIN"

class FFmpegViewer:
    """Multi-camera viewer using FFmpeg for ultra-low latency"""
    
    def __init__(self, config):
        self.config = config
        self.camera_configs = config.get_camera_urls()
        self.camera_lowres_configs = config.get_camera_lowres_urls()
        self.cameras = {}
        self.running = False
        
        # Display settings
        self.single_cam_size = config.get_display_size()
        self.grid_size = (self.single_cam_size[0] * 2, self.single_cam_size[1] * 2)
        self.max_display_fps = config.get_max_display_fps()
        
        # Performance monitoring
        self.display_fps = 0
        self.display_fps_counter = 0
        self.last_display_fps_time = time.time()
        self.target_frame_time = 1.0 / self.max_display_fps if self.max_display_fps > 0 else 0
        
        # Keep last good frame for each camera
        self.last_good_frames = {}
        self.last_good_frame_times = {}
        
        # Fullscreen mode
        self.fullscreen_mode = False
        self.current_camera_index = 0
        self.camera_ids = sorted(self.camera_configs.keys())
        
        # Window management
        self.window_name = "FFmpeg Multi-Camera Viewer (Press 'q' to quit)"
        
    def start(self):
        """Start the FFmpeg viewer"""
        logger.info("Starting FFmpeg ultra-low latency viewer...")
        logger.info(f"Target resolution per camera: {self.single_cam_size}")
        logger.info(f"Total display resolution: {self.grid_size}")
        logger.info(f"Max display FPS: {self.max_display_fps}")
        
        # Check if FFmpeg is available
        try:
            subprocess.run(['ffmpeg', '-version'], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL, 
                         check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            logger.error("FFmpeg not found! Please install FFmpeg")
            logger.error("Ubuntu/Debian: sudo apt install ffmpeg")
            logger.error("CentOS/RHEL: sudo yum install ffmpeg")
            return False
            
        # Initialize cameras
        for camera_id, rtsp_url in self.camera_configs.items():
            rtsp_url_lowres = self.camera_lowres_configs.get(camera_id, None)
            
            camera = FFmpegCamera(
                camera_id=camera_id,
                rtsp_url=rtsp_url,
                rtsp_url_lowres=rtsp_url_lowres,
                target_size=self.single_cam_size,
                max_queue_size=1  # Minimal buffer for low latency
            )
            
            self.cameras[camera_id] = camera
            camera.start()
            time.sleep(0.2)  # Stagger startup
            
        self.running = True
        self._display_loop()
        return True
        
    def stop(self):
        """Stop all cameras and cleanup"""
        logger.info("Stopping FFmpeg viewer...")
        self.running = False
        
        for camera in self.cameras.values():
            camera.stop()
            
        cv2.destroyAllWindows()
        
    def _display_loop(self):
        """Main display loop with grid and fullscreen support"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # Set initial window size
        initial_width = 1280
        initial_height = 720
        cv2.resizeWindow(self.window_name, initial_width, initial_height)
        
        self._show_startup_info()
        
        while self.running:
            try:
                frame_start_time = time.time()
                
                # Collect frames from all cameras
                frames = {}
                healthy_cameras = 0
                
                for camera_id, camera in self.cameras.items():
                    frame = camera.get_latest_frame()
                    
                    if camera.is_healthy():
                        healthy_cameras += 1
                    
                    if frame is not None:
                        frames[camera_id] = frame
                        self.last_good_frames[camera_id] = frame
                        self.last_good_frame_times[camera_id] = time.time()
                    elif camera_id in self.last_good_frames:
                        # Use last good frame if not too old
                        if camera_id in self.last_good_frame_times:
                            age = time.time() - self.last_good_frame_times[camera_id]
                            if age < 5.0:  # Use last good frame for up to 5 seconds
                                frames[camera_id] = self.last_good_frames[camera_id]
                
                # Create display
                if self.fullscreen_mode:
                    display_frame = self._create_fullscreen_display(frames)
                else:
                    display_frame = self._create_grid_display(frames)
                
                # Fallback frame
                if display_frame is None:
                    display_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                    cv2.putText(display_frame, "Initializing FFmpeg Cameras...", (400, 360), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                
                # Add global overlay
                display_frame = self._add_global_overlay(display_frame, healthy_cameras)
                
                # Display frame
                cv2.imshow(self.window_name, display_frame)
                
                # Update display FPS
                self._update_display_fps()
                
                # Handle input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                elif key == ord('f') or key == ord('F'):  # Toggle fullscreen
                    self._toggle_fullscreen_mode()
                elif key == 81 or key == 2 or key == 84:  # Left arrow
                    if self.fullscreen_mode:
                        self._previous_camera()
                elif key == 83 or key == 3 or key == 85:  # Right arrow  
                    if self.fullscreen_mode:
                        self._next_camera()
                elif key == ord('l') or key == ord('L'):  # Toggle stream quality
                    self._toggle_stream_quality()
                elif key == ord('r'):  # Restart failed cameras
                    self._restart_failed_cameras()
                elif key == ord('s'):  # Show statistics
                    self._print_statistics()
                    
                # FPS limiting
                if self.target_frame_time > 0:
                    frame_processing_time = time.time() - frame_start_time
                    sleep_time = self.target_frame_time - frame_processing_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                        
            except Exception as e:
                logger.error(f"Display loop error: {e}")
                time.sleep(0.1)
                
        self.stop()
        
    def _create_grid_display(self, frames: Dict[str, np.ndarray]) -> np.ndarray:
        """Create 2x2 grid layout"""
        # Create high-resolution grid
        grid_width = 1920
        grid_height = 1080
        cam_width = grid_width // 2
        cam_height = grid_height // 2
        
        # Process frames for all 4 positions
        grid_frames = []
        for i in range(4):
            camera_id = str(i + 1)
            if camera_id in frames:
                # Resize frame to grid size
                frame = cv2.resize(frames[camera_id], (cam_width, cam_height), 
                                 interpolation=cv2.INTER_LINEAR)
                grid_frames.append(frame)
            else:
                # Create placeholder
                placeholder = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                font_scale = 1.2
                thickness = 2
                
                text_camera = f"FFmpeg Camera {camera_id}"
                text_status = "NO SIGNAL"
                color = (0, 0, 255)
                
                # Center text
                text_size_camera = cv2.getTextSize(text_camera, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                text_size_status = cv2.getTextSize(text_status, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                
                x_camera = (cam_width - text_size_camera[0]) // 2
                y_camera = (cam_height - text_size_camera[1]) // 2 - 20
                x_status = (cam_width - text_size_status[0]) // 2
                y_status = (cam_height - text_size_status[1]) // 2 + 20
                
                cv2.putText(placeholder, text_camera, (x_camera, y_camera), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
                cv2.putText(placeholder, text_status, (x_status, y_status), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
                
                grid_frames.append(placeholder)
        
        # Create 2x2 grid
        top_row = np.hstack([grid_frames[0], grid_frames[1]])
        bottom_row = np.hstack([grid_frames[2], grid_frames[3]])
        combined = np.vstack([top_row, bottom_row])
        
        return combined
        
    def _create_fullscreen_display(self, frames: Dict[str, np.ndarray]) -> np.ndarray:
        """Create fullscreen display for current camera"""
        if self.current_camera_index >= len(self.camera_ids):
            self.current_camera_index = 0
            
        current_camera_id = self.camera_ids[self.current_camera_index]
        
        if current_camera_id in frames:
            # Scale up the current camera frame
            frame = frames[current_camera_id]
            # Scale to a good fullscreen size while maintaining aspect ratio
            target_height = 1080
            aspect_ratio = frame.shape[1] / frame.shape[0]
            target_width = int(target_height * aspect_ratio)
            
            fullscreen_frame = cv2.resize(frame, (target_width, target_height), 
                                        interpolation=cv2.INTER_LINEAR)
            
            # Add fullscreen overlay
            overlay = fullscreen_frame.copy()
            h, w = fullscreen_frame.shape[:2]
            cv2.rectangle(overlay, (10, h-100), (500, h-10), (0, 0, 0), -1)
            fullscreen_frame = cv2.addWeighted(fullscreen_frame, 0.8, overlay, 0.2, 0)
            
            # Add text
            camera = self.cameras[current_camera_id]
            stream_info = camera.get_current_stream_info()
            texts = [
                f"Fullscreen: Camera {current_camera_id} ({stream_info})",
                f"FPS: {camera.fps} | Health: {'OK' if camera.is_healthy() else 'FAIL'}",
                "F: Grid view | ← →: Switch cameras | L: Quality | Q: Quit"
            ]
            
            for i, text in enumerate(texts):
                y_pos = h - 80 + i * 25
                cv2.putText(fullscreen_frame, text, (20, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                           
            return fullscreen_frame
        else:
            # No frame available for current camera
            placeholder = np.zeros((1080, 1920, 3), dtype=np.uint8)
            cv2.putText(placeholder, f"FFmpeg Camera {current_camera_id} - NO SIGNAL", 
                       (500, 540), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            return placeholder
            
    def _add_global_overlay(self, frame: np.ndarray, healthy_cameras: int) -> np.ndarray:
        """Add global status overlay"""
        if self.fullscreen_mode:
            return frame  # Fullscreen has its own overlay
            
        h, w = frame.shape[:2]
        
        # Status bar at bottom
        overlay_height = max(30, int(h * 0.04))
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h-overlay_height), (w, h), (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.8, overlay, 0.2, 0)
        
        # Status text
        total_cameras = len(self.cameras)
        status_text = f"FFmpeg Grid View | Cameras: {healthy_cameras}/{total_cameras} | Display FPS: {self.display_fps} | F:Fullscreen L:Quality Q:Quit"
        
        font_scale = max(0.4, min(w / 2000, h / 1000))
        thickness = max(1, int(font_scale * 2))
        margin = max(10, int(w * 0.008))
        
        cv2.putText(frame, status_text, (margin, h - margin), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
                   
        return frame
        
    def _update_display_fps(self):
        """Update display FPS calculation"""
        self.display_fps_counter += 1
        current_time = time.time()
        if current_time - self.last_display_fps_time >= 1.0:
            self.display_fps = self.display_fps_counter
            self.display_fps_counter = 0
            self.last_display_fps_time = current_time
            
    def _toggle_fullscreen_mode(self):
        """Toggle between grid and fullscreen mode"""
        self.fullscreen_mode = not self.fullscreen_mode
        mode = "fullscreen" if self.fullscreen_mode else "grid"
        logger.info(f"Switched to {mode} mode")
        
    def _next_camera(self):
        """Switch to next camera in fullscreen mode"""
        if self.fullscreen_mode:
            self.current_camera_index = (self.current_camera_index + 1) % len(self.camera_ids)
            current_id = self.camera_ids[self.current_camera_index]
            logger.info(f"Switched to camera {current_id}")
            
    def _previous_camera(self):
        """Switch to previous camera in fullscreen mode"""
        if self.fullscreen_mode:
            self.current_camera_index = (self.current_camera_index - 1) % len(self.camera_ids)
            current_id = self.camera_ids[self.current_camera_index]
            logger.info(f"Switched to camera {current_id}")
            
    def _toggle_stream_quality(self):
        """Toggle stream quality for all cameras"""
        logger.info("Toggling stream quality for all cameras...")
        for camera in self.cameras.values():
            camera.switch_stream_quality()
            
    def _restart_failed_cameras(self):
        """Restart failed cameras"""
        logger.info("Restarting failed cameras...")
        for camera_id, camera in self.cameras.items():
            if not camera.is_healthy():
                logger.info(f"Restarting camera {camera_id}")
                camera.stop()
                time.sleep(0.5)
                camera.start()
                
    def _print_statistics(self):
        """Print camera statistics"""
        print("\n" + "="*60)
        print("FFmpeg Camera Statistics")
        print("="*60)
        
        for camera_id, camera in self.cameras.items():
            status = "HEALTHY" if camera.is_healthy() else "UNHEALTHY"
            stream_info = camera.get_current_stream_info()
            
            print(f"Camera {camera_id}:")
            print(f"  Status: {status}")
            print(f"  Stream: {stream_info}")
            print(f"  FPS: {camera.fps}")
            print(f"  Total Frames: {camera.total_frames}")
            print(f"  Dropped Frames: {camera.dropped_frames}")
            print(f"  Drop Rate: {camera.dropped_frames/max(camera.total_frames,1)*100:.1f}%")
            print()
            
        print(f"Display FPS: {self.display_fps}")
        print(f"Mode: {'Fullscreen' if self.fullscreen_mode else 'Grid'}")
        print("="*60)
        
    def _show_startup_info(self):
        """Show startup information"""
        print("\n" + "="*60)
        print("FFmpeg Ultra-Low Latency Camera Viewer")
        print("="*60)
        print("Controls:")
        print("  'q' or ESC - Quit")
        print("  'f' - Toggle fullscreen/grid mode")
        print("  ← → Arrow keys - Navigate cameras (fullscreen mode)")
        print("  'l' - Toggle stream quality (main/low-res)")
        print("  'r' - Restart failed cameras")
        print("  's' - Show statistics")
        print("="*60)
        print(f"Cameras: {len(self.cameras)} initialized")
        print("Using FFmpeg for ultra-low latency streaming")
        print("="*60)

def main():
    """Main function for FFmpeg viewer"""
    try:
        # Import config from main module
        import sys
        import os
        sys.path.insert(0, os.path.dirname(__file__))
        from camaras_advanced import CameraConfig
        
        # Load configuration
        config = CameraConfig("../config.ini" if os.path.basename(os.getcwd()) == 'src' else "config.ini")
        
        # Create and start viewer
        viewer = FFmpegViewer(config)
        success = viewer.start()
        
        return 0 if success else 1
        
    except FileNotFoundError:
        logger.error("Config file 'config.ini' not found!")
        return 1
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
