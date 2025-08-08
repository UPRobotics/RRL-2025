# Camera Rotation Persistence

The robust camera viewer now supports persistent camera rotations that are saved and restored across application restarts.

## Features

- **Automatic Saving**: Rotations are saved immediately when changed
- **Automatic Loading**: Rotations are restored when the app starts
- **Per-Camera Storage**: Each camera's rotation is saved individually
- **Validation**: Invalid rotation values are ignored for safety
- **Reset Function**: All rotations can be reset to 0° with a single key

## How It Works

### Rotation Values
- `0` = 0° (normal orientation)
- `1` = 90° clockwise
- `2` = 180° (upside down)
- `3` = 270° clockwise (90° counter-clockwise)

### Storage File
Rotations are saved to `camera_rotations.json` in the application directory.

Example file content:
```json
{
  "1": 0,
  "2": 1,
  "3": 0,
  "4": 2
}
```

This example shows:
- Camera 1: 0° (normal)
- Camera 2: 90° clockwise
- Camera 3: 0° (normal)  
- Camera 4: 180° (upside down)

## Usage

### Basic Rotation
1. Run the camera viewer
2. Press **SPACE** to switch to fullscreen mode
3. Press **R** to rotate the current camera 90° clockwise
4. Use **LEFT/RIGHT arrows** to switch between cameras
5. Repeat step 3 to continue rotating (0° → 90° → 180° → 270° → 0°)

### Reset All Rotations
- Press **T** to reset all camera rotations to 0°
- This is useful when you need to start over or correct multiple cameras

### Keyboard Controls
- **R** or **r**: Rotate current camera 90° clockwise (fullscreen only)
- **T** or **t**: Reset all camera rotations to 0°
- **SPACE**: Toggle between grid and fullscreen view
- **LEFT/RIGHT arrows**: Switch cameras in fullscreen mode

## Technical Details

### When Rotations Are Saved
- Immediately when a camera is rotated (pressing R)
- Immediately when all rotations are reset (pressing T)
- When the application exits gracefully

### When Rotations Are Loaded
- During application startup
- Only for cameras that exist in the current configuration
- Invalid values are ignored and default to 0°

### File Location
The rotation file is saved in the same directory as the application.

### Error Handling
- If the rotation file is corrupted, the app continues with default rotations
- If a camera no longer exists in the config, its rotation is ignored
- Invalid rotation values are ignored and default to 0°

## Benefits

1. **Convenience**: No need to manually rotate cameras every time you start the app
2. **Consistency**: Camera orientations remain consistent across sessions
3. **Flexibility**: Each camera can have its own rotation setting
4. **Safety**: Invalid configurations are handled gracefully

## Example Workflow

1. **Initial Setup**: Start the app, notice some cameras are sideways
2. **Rotate Cameras**: Switch to fullscreen, rotate each camera as needed
3. **Exit**: Close the app normally
4. **Restart**: Start the app again - all rotations are restored automatically
5. **Adjust**: Make any additional rotations needed, they're saved immediately
6. **Reset**: If needed, press T to reset all rotations to 0°

The rotation persistence feature makes the camera viewer much more convenient for setups where cameras are mounted in different orientations.
