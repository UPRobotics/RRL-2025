#!/usr/bin/env python3
"""
Alternative timeout handler for systems that might have issues with signal-based timeouts
"""

import threading
import time
import queue
from typing import Optional, Any

class TimeoutFrameReader:
    """Non-blocking frame reader with timeout support"""
    
    def __init__(self, cap, timeout: float = 5.0):
        self.cap = cap
        self.timeout = timeout
        self.result_queue = queue.Queue(maxsize=1)
        self.worker_thread = None
        
    def read_with_timeout(self) -> tuple:
        """Read a frame with timeout"""
        # Clear any existing results
        while not self.result_queue.empty():
            try:
                self.result_queue.get_nowait()
            except queue.Empty:
                break
        
        # Start worker thread
        self.worker_thread = threading.Thread(target=self._read_worker, daemon=True)
        self.worker_thread.start()
        
        try:
            # Wait for result with timeout
            ret, frame = self.result_queue.get(timeout=self.timeout)
            return ret, frame
        except queue.Empty:
            # Timeout occurred
            return False, None
        except Exception as e:
            # Any other error
            return False, None
            
    def _read_worker(self):
        """Worker thread that reads frames"""
        try:
            if self.cap is None or not self.cap.isOpened():
                self.result_queue.put((False, None))
                return
                
            ret, frame = self.cap.read()
            
            # Add result to queue
            try:
                self.result_queue.put((ret, frame), timeout=1.0)
            except queue.Full:
                # Queue is full, try to clear it and add result
                try:
                    self.result_queue.get_nowait()
                    self.result_queue.put((ret, frame), timeout=1.0)
                except:
                    pass
        except Exception as e:
            try:
                self.result_queue.put((False, None), timeout=1.0)
            except:
                pass
