"""
Multi-Camera RTSP Viewer Package
High-performance Python application for displaying multiple RTSP camera streams.
"""

__version__ = "1.0.0"
__author__ = "Camera Viewer Team"

from .camaras_advanced import HighPerformanceViewer, CameraConfig
from .timeout_reader import TimeoutFrameReader

__all__ = [
    "HighPerformanceViewer",
    "CameraConfig", 
    "TimeoutFrameReader"
]
