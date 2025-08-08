# Old Versions

This folder contains previous versions and implementations of the camera viewer system that are no longer actively used but kept for reference.

## Contents

### Main Files
- `main.py` - Original main implementation
- `main_ffmpeg.py` - FFmpeg-based implementation  
- `main_ffplay.py` - FFplay-based implementation
- `main_shared.py` - Shared stream implementation
- `main_simple.py` - Simple implementation
- `setup.py` - Old setup script

### Scripts
- `run.sh` - Original run script
- `run_ffmpeg.sh` - FFmpeg run script
- `scripts/` - Directory containing various helper scripts
  - `cam.sh` - Camera script
  - `run_cameras.sh` - Camera run script
  - `run_ffplay_direct.sh` - Direct FFplay script

### Source Files (src/)
- `camaras.py` - Original camera implementation
- `camaras_advanced.py` - Advanced camera features
- `ffmpeg_camera.py` - FFmpeg camera class
- `ffplay_camera.py` - FFplay camera class
- `robust_ffmpeg_viewer.py.backup` - Backup of robust viewer
- `shared_stream_viewer.py` - Shared stream viewer
- `simple_ffmpeg_viewer.py` - Simple FFmpeg viewer
- `timeout_reader.py` - Timeout reading utilities

### Test Files
- `test_keyboard.py` - Keyboard input testing
- `test_rotation_persistence.py` - Rotation persistence testing
- `example_camera_rotations.json` - Example rotation configuration

## Current Active Implementation

The current active implementation is:
- `../main_robust.py` - Main entry point
- `../src/robust_ffmpeg_viewer.py` - Robust viewer with rotation persistence

## Purpose

These files are kept for:
- Reference and comparison
- Understanding the evolution of the system
- Potential feature extraction for future improvements
- Backup in case rollback is needed

## Usage

These files are **not** meant to be used directly. They are archived versions. Use the main robust implementation instead.
