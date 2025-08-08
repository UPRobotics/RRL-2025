#!/usr/bin/env python3
"""
Real-time Audio Relay System with Speech Recognition
Captures audio from USB microphone, relays it to speakers, and displays recognized speech in real-time
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
import cv2
import whisper
import tempfile
import os
import wave
import io
from datetime import datetime
import re

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

# Enable debug logging for audio processing
audio_logger = logging.getLogger(__name__ + '.audio')
audio_logger.setLevel(logging.DEBUG)

class SpeechRecognizer:
    """Real-time speech recognition using Whisper"""
    
    def __init__(self, model_name: str = "base.en"):
        self.model_name = model_name
        self.model = None
        self.audio_buffer = queue.Queue(maxsize=10)
        self.recognition_thread = None
        self.running = False
        self.recognized_text = ""
        self.last_recognition_time = 0
        self.recognition_history = []
        self.max_history = 10
        
        # Audio settings for recognition
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_duration = 2.0  # Process 2 seconds of audio at a time (reduced for better responsiveness)
        self.overlap_duration = 0.3  # Overlap between chunks
        
    def load_model(self) -> bool:
        """Load Whisper model"""
        try:
            logger.info(f"Loading Whisper model: {self.model_name}")
            self.model = whisper.load_model(self.model_name)
            logger.info("Whisper model loaded successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to load Whisper model: {e}")
            return False
    
    def preprocess_audio(self, audio_data: np.ndarray, input_sample_rate: int = 48000) -> np.ndarray:
        """Preprocess audio for Whisper"""
        # Ensure input is int16 first
        if audio_data.dtype != np.int16:
            audio_data = audio_data.astype(np.int16)
        
        # Convert to float32 and normalize (Whisper expects float32)
        audio_float = audio_data.astype(np.float32) / 32768.0
        
        # Resample to 16kHz if needed (Whisper requirement)
        if len(audio_float) > 0 and input_sample_rate != self.sample_rate:
            # Better resampling using decimation for common ratios
            if input_sample_rate == 48000 and self.sample_rate == 16000:
                # 48kHz to 16kHz is a 3:1 ratio, use decimation
                audio_float = audio_float[::3]  # Take every 3rd sample
            else:
                # Fallback to linear interpolation
                ratio = self.sample_rate / input_sample_rate
                target_length = int(len(audio_float) * ratio)
                if target_length > 0:
                    # Create arrays explicitly as float32
                    x_old = np.arange(len(audio_float)).astype(np.float32)
                    x_new = np.linspace(0, len(audio_float) - 1, target_length).astype(np.float32)
                    audio_float = np.interp(x_new, x_old, audio_float).astype(np.float32)
        
        # Apply light filtering to reduce noise
        if len(audio_float) > 10:
            # Simple moving average filter to reduce noise
            window_size = 3
            kernel = np.ones(window_size) / window_size
            audio_float = np.convolve(audio_float, kernel, mode='same')
        
        # Ensure output is float32 and contiguous
        return np.ascontiguousarray(audio_float, dtype=np.float32)
    
    def add_audio_for_recognition(self, audio_data: bytes):
        """Add audio data for speech recognition"""
        try:
            # Convert bytes to numpy array
            audio_np = np.frombuffer(audio_data, dtype=np.int16)
            
            # Only add if there's actual audio content
            if len(audio_np) > 0:
                # Add to recognition buffer with larger timeout
                try:
                    self.audio_buffer.put(audio_np, timeout=0.05)
                except queue.Full:
                    # Remove old audio and add new - keep the most recent
                    try:
                        self.audio_buffer.get_nowait()
                        self.audio_buffer.put(audio_np, timeout=0.05)
                    except queue.Empty:
                        # Buffer became empty, just add new data
                        self.audio_buffer.put(audio_np, timeout=0.05)
        except Exception as e:
            logger.error(f"Error adding audio for recognition: {e}")
    
    def recognize_speech_worker(self):
        """Worker thread for speech recognition"""
        logger.info("Starting speech recognition worker...")
        
        audio_chunks = []
        chunk_size = int(self.sample_rate * self.chunk_duration)
        
        while self.running:
            try:
                # Collect audio chunks
                try:
                    audio_chunk = self.audio_buffer.get(timeout=0.1)
                    audio_chunks.append(audio_chunk)
                except queue.Empty:
                    continue
                
                # Combine chunks into a single array
                if audio_chunks:
                    combined_audio = np.concatenate(audio_chunks)
                    
                    # Process if we have enough audio
                    if len(combined_audio) >= chunk_size:
                        # Take the required chunk size
                        audio_for_recognition = combined_audio[:chunk_size]
                        
                        # Log audio characteristics
                        audio_energy = np.mean(np.abs(audio_for_recognition))
                        max_amplitude = np.max(np.abs(audio_for_recognition))
                        logger.debug(f"Processing audio chunk: length={len(audio_for_recognition)}, energy={audio_energy:.6f}, max_amp={max_amplitude:.6f}")
                        
                        # Keep overlap for next iteration
                        overlap_size = int(self.sample_rate * self.overlap_duration)
                        if len(combined_audio) > overlap_size:
                            audio_chunks = [combined_audio[-overlap_size:]]
                        else:
                            audio_chunks = []
                        
                        # Preprocess audio with correct sample rate
                        processed_audio = self.preprocess_audio(audio_for_recognition, input_sample_rate=48000)
                        
                        # Recognize speech
                        self.recognize_audio(processed_audio)
                
            except Exception as e:
                if self.running:
                    logger.error(f"Error in speech recognition worker: {e}")
                    time.sleep(0.1)
        
        logger.info("Speech recognition worker stopped")
    
    def recognize_audio(self, audio_data: np.ndarray):
        """Recognize speech from audio data"""
        try:
            if self.model is None:
                return
            
            # Calculate audio energy for better detection
            audio_energy = np.mean(np.abs(audio_data))
            max_amplitude = np.max(np.abs(audio_data))
            
            # Only process if there's significant audio (lowered threshold)
            if max_amplitude < 0.005:  # Reduced threshold for better sensitivity
                return
            
            # Log audio characteristics for debugging
            logger.debug(f"Audio energy: {audio_energy:.6f}, Max amplitude: {max_amplitude:.6f}")
            
            # Ensure absolutely float32 before passing to Whisper
            audio_data = np.ascontiguousarray(audio_data, dtype=np.float32)
            
            # Perform recognition with better options
            result = self.model.transcribe(
                audio_data, 
                language="en",
                task="transcribe",
                temperature=0.0,  # More deterministic
                best_of=1,
                beam_size=1,
                patience=1.0,
                suppress_tokens="-1"  # Don't suppress any tokens
            )
            text = result["text"].strip()
            
            # Filter out empty or very short results (reduced minimum length)
            if len(text) > 1:  # Reduced from 2 to 1 to catch single words/numbers
                current_time = time.time()
                self.recognized_text = text
                self.last_recognition_time = current_time
                
                # Add to history
                self.recognition_history.append({
                    'text': text,
                    'timestamp': current_time,
                    'confidence': result.get('confidence', 0.0),
                    'energy': audio_energy
                })
                
                # Limit history size
                if len(self.recognition_history) > self.max_history:
                    self.recognition_history.pop(0)
                
                # Log recognition with audio characteristics
                logger.info(f"Recognized: '{text}' (energy: {audio_energy:.6f})")
                
        except Exception as e:
            logger.error(f"Error during speech recognition: {e}")
    
    def start(self):
        """Start speech recognition"""
        if not self.load_model():
            return False
        
        self.running = True
        self.recognition_thread = threading.Thread(target=self.recognize_speech_worker, daemon=True)
        self.recognition_thread.start()
        
        logger.info("Speech recognition started")
        return True
    
    def stop(self):
        """Stop speech recognition"""
        self.running = False
        if self.recognition_thread:
            self.recognition_thread.join(timeout=5)
        logger.info("Speech recognition stopped")
    
    def get_current_text(self) -> str:
        """Get current recognized text"""
        return self.recognized_text
    
    def get_recent_text(self, max_age: float = 5.0) -> List[str]:
        """Get recently recognized text within max_age seconds"""
        current_time = time.time()
        recent_texts = []
        
        for entry in self.recognition_history:
            if current_time - entry['timestamp'] <= max_age:
                recent_texts.append(entry['text'])
        
        return recent_texts

class SpeechDisplay:
    """Display recognized speech in a window"""
    
    def __init__(self, window_name: str = "Speech Recognition"):
        self.window_name = window_name
        self.window_width = 800
        self.window_height = 600
        self.running = False
        self.display_thread = None
        
        # Display settings
        self.background_color = (30, 30, 30)  # Dark gray
        self.text_color = (255, 255, 255)  # White
        self.highlight_color = (0, 255, 0)  # Green
        self.border_color = (100, 100, 100)  # Light gray
        
        # Text settings
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.8
        self.font_thickness = 2
        self.line_spacing = 40
        
    def create_display_image(self, speech_recognizer: SpeechRecognizer) -> np.ndarray:
        """Create display image with recognized text"""
        img = np.full((self.window_height, self.window_width, 3), self.background_color, dtype=np.uint8)
        
        # Draw title
        title = "Real-time Speech Recognition"
        title_size = cv2.getTextSize(title, self.font, 1.0, 2)[0]
        title_x = (self.window_width - title_size[0]) // 2
        cv2.putText(img, title, (title_x, 40), self.font, 1.0, self.highlight_color, 2)
        
        # Draw border
        cv2.rectangle(img, (10, 60), (self.window_width - 10, self.window_height - 10), 
                     self.border_color, 2)
        
        # Get current and recent text
        current_text = speech_recognizer.get_current_text()
        recent_texts = speech_recognizer.get_recent_text(max_age=10.0)
        
        y_offset = 100
        
        # Display current recognition
        if current_text:
            # Highlight current text
            cv2.putText(img, "Current:", (20, y_offset), self.font, 0.6, self.highlight_color, 2)
            y_offset += 30
            
            # Word wrap for long text
            words = current_text.split()
            lines = []
            current_line = ""
            
            for word in words:
                test_line = current_line + " " + word if current_line else word
                text_size = cv2.getTextSize(test_line, self.font, self.font_scale, self.font_thickness)[0]
                
                if text_size[0] < self.window_width - 40:
                    current_line = test_line
                else:
                    if current_line:
                        lines.append(current_line)
                    current_line = word
            
            if current_line:
                lines.append(current_line)
            
            # Draw current text lines
            for line in lines:
                cv2.putText(img, line, (20, y_offset), self.font, self.font_scale, 
                           self.text_color, self.font_thickness)
                y_offset += self.line_spacing
        
        # Display recent history
        y_offset += 20
        if recent_texts:
            cv2.putText(img, "Recent History:", (20, y_offset), self.font, 0.6, self.highlight_color, 2)
            y_offset += 30
            
            # Show last few recognitions
            for i, text in enumerate(reversed(recent_texts[-5:])):  # Last 5 entries
                if y_offset > self.window_height - 50:
                    break
                
                # Truncate very long text
                display_text = text[:80] + "..." if len(text) > 80 else text
                alpha = 1.0 - (i * 0.15)  # Fade older entries
                color = tuple(int(c * alpha) for c in self.text_color)
                
                cv2.putText(img, f"• {display_text}", (20, y_offset), self.font, 0.5, 
                           color, 1)
                y_offset += 25
        
        # Display statistics
        stats_y = self.window_height - 60
        total_recognitions = len(speech_recognizer.recognition_history)
        cv2.putText(img, f"Total Recognitions: {total_recognitions}", (20, stats_y), 
                   self.font, 0.5, self.border_color, 1)
        
        # Display timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        cv2.putText(img, timestamp, (self.window_width - 100, stats_y), 
                   self.font, 0.5, self.border_color, 1)
        
        return img
    
    def display_worker(self, speech_recognizer: SpeechRecognizer):
        """Worker thread for display"""
        logger.info("Starting speech display worker...")
        
        while self.running:
            try:
                # Create display image
                img = self.create_display_image(speech_recognizer)
                
                # Show image
                cv2.imshow(self.window_name, img)
                
                # Check for quit
                key = cv2.waitKey(100) & 0xFF
                if key == ord('q') or key == 27:  # 'q' or ESC
                    logger.info("Display window closed")
                    self.running = False
                    break
                
            except Exception as e:
                if self.running:
                    logger.error(f"Error in display worker: {e}")
                    time.sleep(0.1)
        
        cv2.destroyAllWindows()
        logger.info("Speech display worker stopped")
    
    def start(self, speech_recognizer: SpeechRecognizer):
        """Start display"""
        self.running = True
        self.display_thread = threading.Thread(target=self.display_worker, args=(speech_recognizer,), daemon=True)
        self.display_thread.start()
        logger.info("Speech display started")
    
    def stop(self):
        """Stop display"""
        self.running = False
        if self.display_thread:
            self.display_thread.join(timeout=5)
        cv2.destroyAllWindows()
        logger.info("Speech display stopped")

class AudioRelay:
    """Real-time audio relay from microphone to speakers with speech recognition"""
    
    def __init__(self, 
                 target_device_name: str = "USB Composite Device",
                 chunk_size: int = 1024,
                 sample_rate: int = 44100,
                 channels: int = 2,
                 format: int = pyaudio.paInt16,
                 enable_speech_recognition: bool = True):
        
        self.target_device_name = target_device_name
        self.chunk_size = chunk_size
        self.sample_rate = sample_rate
        self.channels = channels
        self.format = format
        self.running = False
        self.enable_speech_recognition = enable_speech_recognition
        
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
        
        # Speech recognition
        self.speech_recognizer = None
        self.speech_display = None
        
        if self.enable_speech_recognition:
            self.speech_recognizer = SpeechRecognizer()
            self.speech_display = SpeechDisplay()

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
                
                # Send to speech recognizer if enabled
                if self.enable_speech_recognition and self.speech_recognizer:
                    self.speech_recognizer.add_audio_for_recognition(data)
                
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
        
        # Start speech recognition if enabled
        if self.enable_speech_recognition and self.speech_recognizer:
            logger.info("Starting speech recognition...")
            if not self.speech_recognizer.start():
                logger.error("Failed to start speech recognition")
                return False
            
            # Start display
            self.speech_display.start(self.speech_recognizer)
        
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
        if self.enable_speech_recognition:
            logger.info("Speech recognition window opened - press 'q' or ESC to close")
        logger.info("Press Ctrl+C to stop")
        
        return True
    
    def stop_relay(self):
        """Stop the audio relay"""
        logger.info("Stopping audio relay...")
        
        self.running = False
        
        # Stop speech recognition and display
        if self.speech_recognizer:
            self.speech_recognizer.stop()
        if self.speech_display:
            self.speech_display.stop()
        
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
                        logger.info(f"Performance: {fps:.1f} FPS, Buffer: {buffer_size} frames")
                
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
    
    logger.info("Starting USB Microphone Audio Relay with Speech Recognition")
    logger.info("Target device: USB Composite Device (Jieli Technology)")
    
    # Create audio relay with speech recognition
    audio_relay = AudioRelay(
        target_device_name="USB Composite Device",
        chunk_size=1024,  # Small chunk size for low latency
        sample_rate=44100,
        channels=2,
        enable_speech_recognition=True
    )
    
    # Run the relay
    audio_relay.run()

if __name__ == "__main__":
    main()