import cv2
import socket
import numpy as np
import time

def try_open_camera(index, max_attempts=3, delay=1):
    """Attempt to open camera with given index, retrying if necessary."""
    for attempt in range(max_attempts):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if cap.isOpened():
            print(f"Camera opened successfully at index {index}")
            return cap, True
        print(f"Failed to open camera at index {index}, attempt {attempt + 1}/{max_attempts}")
        cap.release()
        time.sleep(delay)
    return None, False

def try_gstreamer_pipeline(device="/dev/video0"):
    """Attempt to open camera using GStreamer pipeline."""
    pipeline = f"v4l2src device={device} ! videoconvert ! video/x-raw,format=BGR ! appsink"
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print(f"Camera opened successfully using GStreamer pipeline: {device}")
        return cap, True
    print(f"Failed to open camera with GStreamer pipeline: {device}")
    return None, False

# Try to find a working camera
cap = None
success = False
for i in range(4):  # Try indices 0 to 3
    cap, success = try_open_camera(i)
    if success:
        break

# Fallback to GStreamer if V4L2 fails
if not success:
    print("Falling back to GStreamer pipeline...")
    for device in ["/dev/video0", "/dev/video1", "/dev/video2"]:
        cap, success = try_gstreamer_pipeline(device)
        if success:
            break

if not success:
    print("Error: Could not open any camera")
    exit()

# Set camera properties (lower resolution to reduce delay)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

# Socket setup
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '0.0.0.0'  # Listen on all interfaces
port = 8581  # Match your setup
server_socket.bind((host, port))
server_socket.listen(1)
print(f"Server listening on {host}:{port}")

# Accept client connection
client_socket, addr = server_socket.accept()
print(f"Connected to client: {addr}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        # Rotate frame 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Encode frame as JPEG with lower quality to reduce delay
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        data = buffer.tobytes()

        # Send frame size followed by frame data
        size = len(data)
        client_socket.sendall(size.to_bytes(4, byteorder='big'))
        client_socket.sendall(data)

except Exception as e:
    print(f"Error: {e}")
finally:
    cap.release()
    client_socket.close()
    server_socket.close()