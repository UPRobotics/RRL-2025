#!/usr/bin/env python3
"""
Test script to verify hardware acceleration is enabled by default
"""

import sys
import os
sys.path.insert(0, 'src')

from robust_ffmpeg_viewer import HardwareAccelerationConfig, Constants
import configparser
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_hardware_acceleration_default():
    """Test that hardware acceleration is enabled by default"""
    print("Testing hardware acceleration default behavior...")
    print("=" * 60)
    
    # Test 1: Default initialization without config
    print("\n1. Testing default initialization (no config):")
    hw_config = HardwareAccelerationConfig()
    print(f"   Hardware acceleration enabled: {hw_config.is_enabled()}")
    print(f"   Hardware acceleration type: {hw_config.hw_accel_type}")
    print(f"   Hardware acceleration device: {hw_config.hw_accel_device}")
    print(f"   Available capabilities: {hw_config.detected_capabilities}")
    
    # Test 2: With default config file
    print("\n2. Testing with default config file:")
    config = configparser.ConfigParser()
    config.read('config.ini')
    
    hw_config2 = HardwareAccelerationConfig()
    hw_config2.configure_from_config(config)
    print(f"   Hardware acceleration enabled: {hw_config2.is_enabled()}")
    print(f"   Hardware acceleration type: {hw_config2.hw_accel_type}")
    print(f"   Hardware acceleration device: {hw_config2.hw_accel_device}")
    
    # Test 3: With hardware acceleration config file
    print("\n3. Testing with hardware acceleration config file:")
    config_hw = configparser.ConfigParser()
    config_hw.read('config_hw_accel.ini')
    
    hw_config3 = HardwareAccelerationConfig()
    hw_config3.configure_from_config(config_hw)
    print(f"   Hardware acceleration enabled: {hw_config3.is_enabled()}")
    print(f"   Hardware acceleration type: {hw_config3.hw_accel_type}")
    print(f"   Hardware acceleration device: {hw_config3.hw_accel_device}")
    
    # Test 4: Explicitly disabled
    print("\n4. Testing explicitly disabled hardware acceleration:")
    config_disabled = configparser.ConfigParser()
    config_disabled.add_section('performance')
    config_disabled.set('performance', 'enable_hardware_acceleration', 'false')
    
    hw_config4 = HardwareAccelerationConfig()
    hw_config4.configure_from_config(config_disabled)
    print(f"   Hardware acceleration enabled: {hw_config4.is_enabled()}")
    print(f"   Hardware acceleration type: {hw_config4.hw_accel_type}")
    
    # Test 5: FFmpeg arguments generation
    print("\n5. Testing FFmpeg arguments generation:")
    if hw_config.is_enabled():
        input_args = hw_config.get_ffmpeg_input_args()
        filter_args = hw_config.get_ffmpeg_filter_args()
        print(f"   Input args: {input_args}")
        print(f"   Filter args: {filter_args}")
    else:
        print("   Hardware acceleration not available - no FFmpeg args generated")
    
    print("\n" + "=" * 60)
    print("Hardware acceleration test completed!")
    
    return hw_config.is_enabled()

if __name__ == "__main__":
    test_hardware_acceleration_default()
