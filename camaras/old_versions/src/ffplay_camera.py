#!/usr/bin/env python3
"""
FFplay-based Ultra-Low Latency Camera Viewer
Uses FFplay directly for minimal latency instead of OpenCV
"""

import subprocess
import threading
import time
import logging
import os
import signal
from typing import Dict, List, Optional, Tuple
import configparser

logger = logging.getLogger(__name__)

class FFplayCamera:
    """Ultra-low latency camera using FFplay directly"""
    
    def __init__(self, camera_id: str, rtsp_url: str, position: Tuple[int, int], 
                 size: Tuple[int, int] = (960, 540), rtsp_url_lowres: str = None):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.rtsp_url_lowres = rtsp_url_lowres
        self.current_url = rtsp_url
        self.is_lowres_mode = False
        self.position = position  # (x, y) window position
        self.size = size  # (width, height) window size
        self.process = None
        self.running = False
        self.thread = None
        
        # Performance monitoring
        self.start_time = None
        self.restart_count = 0
        
    def start(self):
        """Start FFplay process"""
        if self.process is not None:
            self.stop()
            
        self.running = True
        self.thread = threading.Thread(target=self._run_ffplay, daemon=True)
        self.thread.start()
        logger.info(f"Started FFplay for camera {self.camera_id}")
        
    def stop(self):
        """Stop FFplay process"""
        self.running = False
        
        if self.process is not None:
            try:
                # Graceful shutdown first
                self.process.terminate()
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    # Force kill if graceful shutdown fails
                    self.process.kill()
                    self.process.wait()
            except Exception as e:
                logger.warning(f"Camera {self.camera_id} process cleanup error: {e}")
            finally:
                self.process = None
                
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            
        logger.info(f"Stopped FFplay for camera {self.camera_id}")
        
    def _run_ffplay(self):
        """Run FFplay process with monitoring and auto-restart"""
        while self.running:
            try:
                self.start_time = time.time()
                
                # Build FFplay command with ultra-low latency settings
                cmd = self._build_ffplay_command()
                
                logger.info(f"Camera {self.camera_id} starting FFplay: {' '.join(cmd[:3])}...")
                
                # Start FFplay process
                self.process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid  # Create new process group for clean shutdown
                )
                
                # Monitor process
                while self.running and self.process.poll() is None:
                    time.sleep(0.1)
                    
                # Process ended
                if self.running:
                    self.restart_count += 1
                    logger.warning(f"Camera {self.camera_id} FFplay ended unexpectedly, restart #{self.restart_count}")
                    time.sleep(2.0)  # Brief delay before restart
                    
            except Exception as e:
                logger.error(f"Camera {self.camera_id} FFplay error: {e}")
                if self.running:
                    time.sleep(5.0)
                    
    def _build_ffplay_command(self) -> List[str]:
        """Build optimized FFplay command"""
        cmd = [
            'ffplay',
            '-rtsp_transport', 'tcp',
            '-i', self.current_url,
            '-an',  # No audio
            '-probesize', '1024',
            '-analyzeduration', '1000000',
            '-fflags', 'nobuffer+fastseek',
            '-flags', 'low_delay',
            '-err_detect', 'ignore_err',
            '-framedrop',
            '-max_delay', '0',
            '-reorder_queue_size', '0',
            '-vf', f'setpts=0.5*PTS,scale={self.size[0]}:{self.size[1]}',
            '-window_title', f'Camera {self.camera_id} ({self.get_stream_info()})',
            '-x', str(self.position[0]),
            '-y', str(self.position[1])
        ]
        
        return cmd
        
    def switch_stream_quality(self) -> bool:
        """Switch between main and low-resolution streams"""
        if not self.rtsp_url_lowres:
            logger.warning(f"Camera {self.camera_id} has no low-res URL configured")
            return False
            
        # Toggle URL
        if self.is_lowres_mode:
            self.current_url = self.rtsp_url
            self.is_lowres_mode = False
            logger.info(f"Camera {self.camera_id} switching to MAIN stream")
        else:
            self.current_url = self.rtsp_url_lowres
            self.is_lowres_mode = True
            logger.info(f"Camera {self.camera_id} switching to LOW-RES stream")
            
        # Restart with new URL
        if self.running:
            self._restart_stream()
            
        return True
        
    def _restart_stream(self):
        """Restart stream with current URL"""
        if self.process is not None:
            try:
                self.process.terminate()
                self.process.wait(timeout=1.0)
            except:
                pass
            self.process = None
            
    def get_stream_info(self) -> str:
        """Get current stream quality info"""
        return "LOW-RES" if self.is_lowres_mode else "MAIN"
        
    def is_healthy(self) -> bool:
        """Check if camera process is running"""
        return (self.process is not None and 
                self.process.poll() is None and 
                self.running)

class FFplayViewer:
    """Multi-camera viewer using FFplay for ultra-low latency"""
    
    def __init__(self, config):
        self.config = config
        self.cameras = {}
        self.running = False
        
        # Get camera configurations
        self.camera_configs = config.get_camera_urls()
        self.camera_lowres_configs = config.get_camera_lowres_urls()
        
        # Display settings
        self.single_cam_size = config.get_display_size()
        
        # Calculate grid positions (2x2 layout)
        self.grid_positions = self._calculate_grid_positions()
        
    def _calculate_grid_positions(self) -> Dict[str, Tuple[int, int]]:
        """Calculate window positions for 2x2 grid"""
        positions = {}
        camera_ids = sorted(self.camera_configs.keys())
        
        # Grid layout (adjust these values based on your screen resolution)
        base_x = 100
        base_y = 100
        spacing_x = self.single_cam_size[0] + 20
        spacing_y = self.single_cam_size[1] + 50  # Extra space for window title
        
        grid_map = {
            '1': (0, 0),  # Top-left
            '2': (1, 0),  # Top-right
            '3': (0, 1),  # Bottom-left
            '4': (1, 1)   # Bottom-right
        }
        
        for camera_id in camera_ids:
            if camera_id in grid_map:
                grid_x, grid_y = grid_map[camera_id]
                pos_x = base_x + (grid_x * spacing_x)
                pos_y = base_y + (grid_y * spacing_y)
                positions[camera_id] = (pos_x, pos_y)
            else:
                # Fallback for additional cameras
                positions[camera_id] = (base_x, base_y)
                
        return positions
        
    def start(self):
        """Start the FFplay viewer"""
        logger.info("Starting FFplay ultra-low latency viewer...")
        
        # Check if ffplay is available
        try:
            subprocess.run(['ffplay', '-version'], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL, 
                         check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            logger.error("FFplay not found! Please install FFmpeg with FFplay")
            logger.error("Ubuntu/Debian: sudo apt install ffmpeg")
            logger.error("CentOS/RHEL: sudo yum install ffmpeg")
            return False
            
        # Initialize cameras
        for camera_id, rtsp_url in self.camera_configs.items():
            position = self.grid_positions.get(camera_id, (100, 100))
            rtsp_url_lowres = self.camera_lowres_configs.get(camera_id, None)
            
            camera = FFplayCamera(
                camera_id=camera_id,
                rtsp_url=rtsp_url,
                rtsp_url_lowres=rtsp_url_lowres,
                position=position,
                size=self.single_cam_size
            )
            
            self.cameras[camera_id] = camera
            camera.start()
            time.sleep(0.5)  # Stagger startup
            
        self.running = True
        
        # Show instructions
        self._show_instructions()
        
        # Start control loop
        self._control_loop()
        
        return True
        
    def stop(self):
        """Stop all cameras"""
        logger.info("Stopping FFplay viewer...")
        self.running = False
        
        for camera in self.cameras.values():
            camera.stop()
            
    def _show_instructions(self):
        """Show usage instructions"""
        print("\n" + "="*60)
        print("FFplay Ultra-Low Latency Camera Viewer")
        print("="*60)
        print("Controls:")
        print("  'q' + Enter - Quit")
        print("  'l' + Enter - Toggle stream quality (main/low-res)")
        print("  'r' + Enter - Restart all cameras")
        print("  's' + Enter - Show statistics")
        print("  'h' + Enter - Show this help")
        print("="*60)
        print(f"Cameras: {len(self.cameras)} started")
        print("Camera windows should appear on your screen")
        print("Each window can be moved and resized independently")
        print("="*60)
        
    def _control_loop(self):
        """Interactive control loop"""
        try:
            while self.running:
                try:
                    command = input("Command (h for help, q to quit): ").strip().lower()
                    
                    if command == 'q':
                        break
                    elif command == 'l':
                        self._toggle_all_stream_quality()
                    elif command == 'r':
                        self._restart_all_cameras()
                    elif command == 's':
                        self._show_statistics()
                    elif command == 'h':
                        self._show_instructions()
                    elif command == '':
                        continue  # Empty input, just continue
                    else:
                        print(f"Unknown command: '{command}'. Type 'h' for help.")
                        
                except EOFError:
                    # Ctrl+D pressed
                    break
                except KeyboardInterrupt:
                    # Ctrl+C pressed
                    break
                    
        except Exception as e:
            logger.error(f"Control loop error: {e}")
        finally:
            self.stop()
            
    def _toggle_all_stream_quality(self):
        """Toggle stream quality for all cameras"""
        print("Toggling stream quality for all cameras...")
        for camera in self.cameras.values():
            camera.switch_stream_quality()
            
    def _restart_all_cameras(self):
        """Restart all cameras"""
        print("Restarting all cameras...")
        for camera in self.cameras.values():
            camera.stop()
            time.sleep(0.5)
            camera.start()
            
    def _show_statistics(self):
        """Show camera statistics"""
        print("\n" + "="*50)
        print("Camera Statistics")
        print("="*50)
        
        for camera_id, camera in self.cameras.items():
            status = "RUNNING" if camera.is_healthy() else "STOPPED"
            stream_info = camera.get_stream_info()
            uptime = time.time() - camera.start_time if camera.start_time else 0
            
            print(f"Camera {camera_id}:")
            print(f"  Status: {status}")
            print(f"  Stream: {stream_info}")
            print(f"  Uptime: {uptime:.1f}s")
            print(f"  Restarts: {camera.restart_count}")
            print(f"  Position: {camera.position}")
            print()
            
def main():
    """Main function for FFplay viewer"""
    try:
        # Load configuration
        from camaras_advanced import CameraConfig
        config = CameraConfig("../config.ini")
        
        # Create and start viewer
        viewer = FFplayViewer(config)
        success = viewer.start()
        
        if not success:
            return 1
            
    except FileNotFoundError:
        logger.error("Config file 'config.ini' not found!")
        return 1
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1
        
    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())
