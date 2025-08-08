#!/usr/bin/env python3
"""
Audio Relay Control Script
Simple interface to control the USB microphone audio relay
"""

import subprocess
import sys
import os
import signal
import time

class AudioRelayController:
    def __init__(self):
        self.process = None
        self.script_path = os.path.join(os.path.dirname(__file__), "src", "mic.py")
        
    def start_relay(self):
        """Start the audio relay"""
        if self.process:
            print("Audio relay is already running!")
            return
            
        print("Starting USB microphone audio relay...")
        print("Press Ctrl+C to stop")
        print("=" * 50)
        
        try:
            self.process = subprocess.Popen([sys.executable, self.script_path])
            self.process.wait()
        except KeyboardInterrupt:
            print("\nStopping audio relay...")
            self.stop_relay()
        except Exception as e:
            print(f"Error starting audio relay: {e}")
            
    def stop_relay(self):
        """Stop the audio relay"""
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
                print("Audio relay stopped")
            except subprocess.TimeoutExpired:
                print("Force stopping audio relay...")
                self.process.kill()
                self.process.wait()
                print("Audio relay force stopped")
            except Exception as e:
                print(f"Error stopping audio relay: {e}")
            finally:
                self.process = None
                
    def status(self):
        """Check relay status"""
        if self.process and self.process.poll() is None:
            print("Audio relay is running")
        else:
            print("Audio relay is not running")
            
    def show_help(self):
        """Show help message"""
        print("USB Microphone Audio Relay Controller")
        print("=" * 40)
        print("Commands:")
        print("  start  - Start the audio relay")
        print("  stop   - Stop the audio relay")  
        print("  status - Check relay status")
        print("  help   - Show this help message")
        print()
        print("The relay will capture audio from your USB microphone")
        print("(Jieli Technology USB Composite Device) and output")
        print("it to your default speakers in real-time.")

def main():
    controller = AudioRelayController()
    
    if len(sys.argv) < 2:
        print("Usage: python3 mic_control.py [start|stop|status|help]")
        return
        
    command = sys.argv[1].lower()
    
    if command == "start":
        controller.start_relay()
    elif command == "stop":
        controller.stop_relay()
    elif command == "status":
        controller.status()
    elif command == "help":
        controller.show_help()
    else:
        print(f"Unknown command: {command}")
        print("Use 'help' for available commands")

if __name__ == "__main__":
    main()
