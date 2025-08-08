#!/usr/bin/env python3
"""
Real-time Audio Relay System
Captures audio from USB microphone and relays it to speakers in real-time
"""

import pyaudio
import threading
import time
import logging
import sys
import signal
import numpy as np
from typing import Optional, List, Tuple
import queue

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('mic_relay.log')
    ]
)
logger = logging.getLogger(__name__)

class AudioRelay:
    """Real-time audio relay from microphone to speakers"""
    
    def __init__(self, 
                 target_device_name: str = "USB Composite Device",
                 chunk_size: int = 1024,
                 sample_rate: int = 44100,
                 channels: int = 2,
                 format: int = pyaudio.paInt16):
        
        self.target_device_name = target_device_name
        self.chunk_size = chunk_size
        self.sample_rate = sample_rate
        self.channels = channels
        self.format = format
        self.running = False
        
        # PyAudio instance
        self.audio = pyaudio.PyAudio()
        
        # Audio streams
        self.input_stream = None
        self.output_stream = None
        
        # Device info
        self.input_device_index = None
        self.output_device_index = None
        
        # Audio buffer
        self.audio_buffer = queue.Queue(maxsize=10)
        
        # Threading
        self.input_thread = None
        self.output_thread = None
        
        # Performance monitoring
        self.frames_processed = 0
        self.start_time = None
        
        # Audio processing
        self.volume_multiplier = 1.0
        self.enable_monitoring = True
        
    def find_devices(self) -> Tuple[Optional[int], Optional[int]]:
        """Find input and output audio devices"""
        logger.info("Scanning for audio devices...")
        
        input_device = None
        output_device = None
        
        # List all audio devices
        device_count = self.audio.get_device_count()
        logger.info(f"Found {device_count} audio devices:")
        
        for i in range(device_count):
            try:
                info = self.audio.get_device_info_by_index(i)
                device_name = info['name']
                max_input_channels = info['maxInputChannels']
                max_output_channels = info['maxOutputChannels']
                
                logger.info(f"  Device {i}: {device_name}")
                logger.info(f"    Input channels: {max_input_channels}, Output channels: {max_output_channels}")
                
                # Look for USB microphone (input device)
                if (self.target_device_name.lower() in device_name.lower() and 
                    max_input_channels > 0 and input_device is None):
                    input_device = i
                    logger.info(f"    -> Selected as INPUT device")
                
                # Look for default output device
                if (max_output_channels > 0 and output_device is None and 
                    ('speaker' in device_name.lower() or 'audio' in device_name.lower() or 
                     i == self.audio.get_default_output_device_info()['index'])):
                    output_device = i
                    logger.info(f"    -> Selected as OUTPUT device")
                    
            except Exception as e:
                logger.warning(f"Error getting info for device {i}: {e}")
        
        # Fallback to default devices if specific ones not found
        if input_device is None:
            try:
                input_device = self.audio.get_default_input_device_info()['index']
                logger.info(f"Using default input device: {input_device}")
            except Exception as e:
                logger.error(f"No input device found: {e}")
                
        if output_device is None:
            try:
                output_device = self.audio.get_default_output_device_info()['index']
                logger.info(f"Using default output device: {output_device}")
            except Exception as e:
                logger.error(f"No output device found: {e}")
        
        return input_device, output_device
    
    def test_device_compatibility(self, device_index: int, is_input: bool = True) -> Tuple[int, int, int]:
        """Test device compatibility and find best settings"""
        logger.info(f"Testing {'input' if is_input else 'output'} device {device_index}...")
        
        # Test different configurations
        test_rates = [44100, 48000, 22050, 16000]
        test_channels = [2, 1] if is_input else [2, 1]
        test_formats = [pyaudio.paInt16, pyaudio.paInt32, pyaudio.paFloat32]
        
        for rate in test_rates:
            for channels in test_channels:
                for fmt in test_formats:
                    try:
                        if is_input:
                            # Test input stream
                            test_stream = self.audio.open(
                                format=fmt,
                                channels=channels,
                                rate=rate,
                                input=True,
                                input_device_index=device_index,
                                frames_per_buffer=self.chunk_size
                            )
                        else:
                            # Test output stream
                            test_stream = self.audio.open(
                                format=fmt,
                                channels=channels,
                                rate=rate,
                                output=True,
                                output_device_index=device_index,
                                frames_per_buffer=self.chunk_size
                            )
                        
                        test_stream.close()
                        logger.info(f"  Compatible: {rate}Hz, {channels}ch, format={fmt}")
                        return rate, channels, fmt
                        
                    except Exception as e:
                        continue
        
        # If no configuration works, return defaults
        logger.warning(f"No compatible configuration found for device {device_index}, using defaults")
        return 44100, 2, pyaudio.paInt16
    
    def initialize_streams(self) -> bool:
        """Initialize audio input and output streams"""
        logger.info("Initializing audio streams...")
        
        # Find devices
        self.input_device_index, self.output_device_index = self.find_devices()
        
        if self.input_device_index is None or self.output_device_index is None:
            logger.error("Could not find required audio devices")
            return False
        
        # Test device compatibility
        input_rate, input_channels, input_format = self.test_device_compatibility(
            self.input_device_index, is_input=True)
        output_rate, output_channels, output_format = self.test_device_compatibility(
            self.output_device_index, is_input=False)
        
        # Use the most compatible settings
        # Use the highest common sample rate that both devices support
        common_rates = [48000, 44100, 22050, 16000]
        self.sample_rate = 48000  # Default to 48kHz which the USB mic supports
        for rate in common_rates:
            if rate <= input_rate and rate <= output_rate:
                self.sample_rate = rate
                break
        
        # Use minimum channels to ensure compatibility
        self.channels = min(input_channels, output_channels)
        if self.channels < 1:
            self.channels = 1
            
        # Use compatible format
        self.format = input_format  # Use input format for consistency
        
        logger.info(f"Using settings: {self.sample_rate}Hz, {self.channels}ch, format={self.format}")
        
        try:
            # Create input stream with USB mic specific settings
            self.input_stream = self.audio.open(
                format=self.format,
                channels=1,  # USB mic is mono
                rate=48000,  # USB mic supports 48kHz
                input=True,
                input_device_index=self.input_device_index,
                frames_per_buffer=self.chunk_size,
                stream_callback=None
            )
            
            # Create output stream with system default settings
            self.output_stream = self.audio.open(
                format=self.format,
                channels=1,  # Match input to avoid conversion issues
                rate=48000,  # Use same rate as input
                output=True,
                output_device_index=self.output_device_index,
                frames_per_buffer=self.chunk_size,
                stream_callback=None
            )
            
            # Update settings to reflect actual configuration
            self.sample_rate = 48000
            self.channels = 1
            
            logger.info("Audio streams initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize audio streams: {e}")
            return False
    
    def audio_input_worker(self):
        """Worker thread for audio input"""
        logger.info("Starting audio input worker...")
        
        while self.running:
            try:
                # Read audio data from microphone
                data = self.input_stream.read(self.chunk_size, exception_on_overflow=False)
                
                # Apply volume control if needed
                if self.volume_multiplier != 1.0:
                    # Convert to numpy array for processing
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    audio_data = (audio_data * self.volume_multiplier).astype(np.int16)
                    data = audio_data.tobytes()
                
                # Add to buffer
                try:
                    self.audio_buffer.put(data, timeout=0.01)
                except queue.Full:
                    # Buffer is full, skip this frame to avoid latency buildup
                    try:
                        self.audio_buffer.get_nowait()  # Remove old frame
                        self.audio_buffer.put(data, timeout=0.01)
                    except queue.Empty:
                        pass
                
                self.frames_processed += 1
                
            except Exception as e:
                if self.running:
                    logger.error(f"Error in audio input worker: {e}")
                    time.sleep(0.001)
        
        logger.info("Audio input worker stopped")
    
    def audio_output_worker(self):
        """Worker thread for audio output"""
        logger.info("Starting audio output worker...")
        
        while self.running:
            try:
                # Get audio data from buffer
                try:
                    data = self.audio_buffer.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                # Write audio data to speakers
                self.output_stream.write(data)
                
            except Exception as e:
                if self.running:
                    logger.error(f"Error in audio output worker: {e}")
                    time.sleep(0.001)
        
        logger.info("Audio output worker stopped")
    
    def start_relay(self):
        """Start the audio relay"""
        logger.info("Starting audio relay...")
        
        # Initialize streams
        if not self.initialize_streams():
            logger.error("Failed to initialize audio streams")
            return False
        
        # Start streams
        self.input_stream.start_stream()
        self.output_stream.start_stream()
        
        # Set running flag
        self.running = True
        self.start_time = time.time()
        
        # Start worker threads
        self.input_thread = threading.Thread(target=self.audio_input_worker, daemon=True)
        self.output_thread = threading.Thread(target=self.audio_output_worker, daemon=True)
        
        self.input_thread.start()
        self.output_thread.start()
        
        logger.info("Audio relay started successfully")
        logger.info(f"Relaying audio from device {self.input_device_index} to device {self.output_device_index}")
        logger.info("Press Ctrl+C to stop")
        
        return True
    
    def stop_relay(self):
        """Stop the audio relay"""
        logger.info("Stopping audio relay...")
        
        self.running = False
        
        # Stop streams
        if self.input_stream:
            self.input_stream.stop_stream()
            self.input_stream.close()
            
        if self.output_stream:
            self.output_stream.stop_stream()
            self.output_stream.close()
        
        # Close PyAudio
        self.audio.terminate()
        
        # Print statistics
        if self.start_time:
            elapsed = time.time() - self.start_time
            logger.info(f"Relay ran for {elapsed:.2f} seconds")
            logger.info(f"Processed {self.frames_processed} audio frames")
            if elapsed > 0:
                fps = self.frames_processed / elapsed
                logger.info(f"Average frame rate: {fps:.2f} FPS")
        
        logger.info("Audio relay stopped")
    
    def monitor_performance(self):
        """Monitor and log performance metrics"""
        logger.info("Starting performance monitor...")
        
        while self.running:
            try:
                time.sleep(5)  # Check every 5 seconds
                
                if self.start_time:
                    elapsed = time.time() - self.start_time
                    if elapsed > 0:
                        fps = self.frames_processed / elapsed
                        buffer_size = self.audio_buffer.qsize()
                        #logger.info(f"Performance: {fps:.1f} FPS, Buffer: {buffer_size} frames")
                
            except Exception as e:
                logger.error(f"Error in performance monitor: {e}")
                
        logger.info("Performance monitor stopped")
    
    def run(self):
        """Main run loop"""
        try:
            # Start the relay
            if not self.start_relay():
                return
            
            # Start performance monitoring if enabled
            if self.enable_monitoring:
                monitor_thread = threading.Thread(target=self.monitor_performance, daemon=True)
                monitor_thread.start()
            
            # Keep main thread alive
            while self.running:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.stop_relay()

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info("Received shutdown signal")
    global audio_relay
    if audio_relay:
        audio_relay.stop_relay()
    sys.exit(0)

def main():
    """Main function"""
    global audio_relay
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info("Starting USB Microphone Audio Relay")
    logger.info("Target device: USB Composite Device (Jieli Technology)")
    
    # Create audio relay
    audio_relay = AudioRelay(
        target_device_name="USB Composite Device",
        chunk_size=1024,  # Small chunk size for low latency
        sample_rate=44100,
        channels=2
    )
    
    # Run the relay
    audio_relay.run()

if __name__ == "__main__":
    main()