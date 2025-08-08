#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from depthai_ros_msgs.msg import SpatialDetectionArray
from std_msgs.msg import Header
import time
import os
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class OakDS2DepthNode(Node):
    def __init__(self):
        super().__init__('oak_d_s2_depth_node')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create publishers
        self.rgb_pub = self.create_publisher(Image, '/oak_d_s2/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oak_d_s2/depth/image_raw', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/oak_d_s2/rgb/camera_info', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/oak_d_s2/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/oak_d_s2/right/camera_info', 10)
        
        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()
        
        # Define camera nodes
        self.rgb = self.pipeline.create(dai.node.ColorCamera)
        self.left = self.pipeline.create(dai.node.MonoCamera)
        self.right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        
        # Configure RGB camera
        self.rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.rgb.setFps(30)
        self.rgb.setPreviewSize(640, 400)
        
        # Configure mono cameras
        self.left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.left.setFps(30)
        
        self.right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        self.right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.right.setFps(30)
        
        # Configure stereo depth
        self.stereo.initialConfig.setConfidenceThreshold(200)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(False)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        
        # Link mono cameras to stereo depth
        self.left.out.link(self.stereo.left)
        self.right.out.link(self.stereo.right)
        
        # Find available devices
        devices = dai.Device.getAllConnectedDevices()
        if not devices:
            raise RuntimeError("No DepthAI devices found. Please connect the Oak-DS2 camera.")
        
        # Select the first available device
        device_info = devices[0]
        self.get_logger().info(f"Using first available device. Will verify serial number after initialization.")
        
        # Attempt to reset USB port to clear any stuck state
        self.reset_usb_port()
        
        # Start the device with retry mechanism
        max_retries = 5
        self.device = None
        usb_speeds = [dai.UsbSpeed.SUPER, dai.UsbSpeed.HIGH]  # Try USB3 first, then USB2
        last_error = None
        for usb_speed in usb_speeds:
            for attempt in range(max_retries):
                try:
                    self.get_logger().info(f"Attempting to initialize device with USB speed: {usb_speed}, attempt {attempt + 1}/{max_retries}")
                    self.device = dai.Device(device_info, maxUsbSpeed=usb_speed)
                    self.device.startPipeline(self.pipeline)
                    # Verify serial number after initialization
                    serial_number = self.device.getMxId()
                    self.get_logger().info(f"Device initialized successfully with USB speed {usb_speed}. Serial number: {serial_number}")
                    if serial_number != "1944301091BC181300":
                        self.get_logger().warning(f"Expected serial number 1944301091BC181300, but got {serial_number}")
                    break
                except RuntimeError as e:
                    last_error = str(e)
                    self.get_logger().warning(f"Initialization attempt {attempt + 1}/{max_retries} failed with USB speed {usb_speed}: {last_error}")
                    if "X_LINK" in last_error and attempt < max_retries - 1:
                        self.get_logger().warning("XLink error, retrying in 5 seconds...")
                        time.sleep(5)
                    else:
                        if self.device:
                            self.device.close()
                        self.device = None
                        break
            if self.device:
                break
        if not self.device:
            self.get_logger().error(
                f"Failed to connect to device after trying all USB speeds. Last error: {last_error or 'Unknown error'}. "
                "Check 'lsusb' and 'dmesg | tail -n 50' for USB disconnections. "
                "Ensure USB3 cable and port are used. Try 'sudo fuser /dev/bus/usb/001/XXX' or 'sudo lsof /dev/bus/usb/001/XXX' to check processes."
            )
            raise RuntimeError(f"Failed to connect to device: {last_error or 'Unknown error'}")
        
        # Create output queues directly
        try:
            self.rgb_queue = self.rgb.preview.createOutputQueue(maxSize=4, blocking=False)
            self.depth_queue = self.stereo.depth.createOutputQueue(maxSize=4, blocking=False)
        except Exception as e:
            self.get_logger().error(f"Failed to create output queues: {str(e)}")
            self.device.close()
            raise
        
        # Get camera calibration
        try:
            self.calib_data = self.device.readCalibration()
            self.rgb_intrinsics = self.calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
            self.left_intrinsics = self.calib_data.getCameraIntrinsics(dai.CameraBoardSocket.LEFT)
            self.right_intrinsics = self.calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT)
        except Exception as e:
            self.get_logger().error(f"Failed to read calibration: {str(e)}")
            self.device.close()
            raise
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / 30, self.publish_frames)

    def reset_usb_port(self):
        """Attempt to reset the USB port to clear any stuck state."""
        try:
            import subprocess
            # Identify the USB port (e.g., 1-1 or 2-1) from lsusb or dmesg
            usb_port = "1-1"  # Adjust based on dmesg output
            self.get_logger().info(f"Resetting USB port {usb_port}")
            subprocess.run(["echo", usb_port, "|", "sudo", "tee", "/sys/bus/usb/devices/1-1/remove"], check=False)
            time.sleep(1)
            subprocess.run(["echo", "0", "|", "sudo", "tee", "/sys/bus/usb/devices/usb1/power/autosuspend"], check=False)
            subprocess.run(["echo", "auto", "|", "sudo", "tee", "/sys/bus/usb/devices/usb1/power/control"], check=False)
            self.get_logger().info("USB port reset completed")
        except Exception as e:
            self.get_logger().warning(f"Failed to reset USB port: {str(e)}")

    def get_camera_info(self, intrinsics, width, height, frame_id):
        camera_info = CameraInfo()
        camera_info.header.frame_id = frame_id
        camera_info.width = width
        camera_info.height = height
        camera_info.k = [intrinsics[0][0], 0.0, intrinsics[0][2],
                        0.0, intrinsics[1][1], intrinsics[1][2],
                        0.0, 0.0, 1.0]
        camera_info.d = self.calib_data.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
        camera_info.p = [intrinsics[0][0], 0.0, intrinsics[0][2], 0.0,
                        0.0, intrinsics[1][1], intrinsics[1][2], 0.0,
                        0.0, 0.0, 1.0, 0.0]
        return camera_info

    def publish_frames(self):
        # Get current timestamp
        stamp = self.get_clock().now().to_msg()
        
        # Publish RGB frame
        rgb_frame = self.rgb_queue.tryGet()
        if rgb_frame is not None:
            rgb_cv = rgb_frame.getCvFrame()
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_cv, encoding="bgr8")
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = "oak_d_s2_rgb"
            self.rgb_pub.publish(rgb_msg)
            
            # Publish RGB camera info
            rgb_info = self.get_camera_info(self.rgb_intrinsics, 640, 400, "oak_d_s2_rgb")
            rgb_info.header.stamp = stamp
            self.rgb_info_pub.publish(rgb_info)
        
        # Publish depth frame
        depth_frame = self.depth_queue.tryGet()
        if depth_frame is not None:
            depth_cv = depth_frame.getFrame()
            depth_msg = self.bridge.cv2_to_imgmsg(depth_cv, "16UC1")
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = "oak_d_s2_depth"
            self.depth_pub.publish(depth_msg)
            
            # Publish left and right camera info
            left_info = self.get_camera_info(self.left_intrinsics, 640, 400, "oak_d_s2_left")
            left_info.header.stamp = stamp
            self.left_info_pub.publish(left_info)
            
            right_info = self.get_camera_info(self.right_intrinsics, 640, 400, "oak_d_s2_right")
            right_info.header.stamp = stamp
            self.right_info_pub.publish(right_info)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OakDS2DepthNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        if node and node.device:
            node.get_logger().info("Closing device")
            node.device.close()
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()