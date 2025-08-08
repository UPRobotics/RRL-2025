import socket
import cv2
import numpy as np
import torch
import pathlib
import platform
import sys
import os

# Workaround for Windows with PosixPath
if platform.system() == 'Windows':
    pathlib.PosixPath = pathlib.WindowsPath

# Initialize YOLOv5 model with offline fallback methods (YOLOv5 specific)
model_path = "codigo.pt"  # Ensure this file is on the client computer
model = None

print("Attempting to load YOLOv5 model...")

# Method 1: Try torch.hub with force_reload=False (uses cache)
try:
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
    print("✓ Model loaded from torch.hub cache")
except Exception as e:
    print(f"✗ torch.hub failed: {e}")

# Method 2: Try local YOLOv5 installation
if model is None:
    try:
        # Check if local yolov5 directory exists
        if os.path.exists('./yolov5'):
            sys.path.append('./yolov5')
            from models.experimental import attempt_load
            from utils.torch_utils import select_device
            
            device = select_device('')
            model = attempt_load(model_path, map_location=device)
            model.eval()
            print("✓ Model loaded from local YOLOv5 installation")
        else:
            print("✗ Local yolov5 directory not found")
    except Exception as e:
        print(f"✗ Local YOLOv5 loading failed: {e}")

# Method 3: Try loading with torch directly (for custom models)
if model is None:
    try:
        checkpoint = torch.load(model_path, map_location='cpu')
        if 'model' in checkpoint:
            model = checkpoint['model']
        else:
            model = checkpoint
        model.eval()
        print("✓ Model loaded directly with torch.load")
    except Exception as e:
        print(f"✗ Direct torch.load failed: {e}")

# Check if model was loaded successfully
if model is None:
    print("\n❌ ERROR: Could not load the YOLOv5 model. Please try one of these solutions:")
    print("1. Clone YOLOv5 locally: git clone https://github.com/ultralytics/yolov5.git")
    print("2. Ensure you have internet connection for initial model download")
    print("3. Check if 'codigo.pt' exists in the current directory")
    print("4. Install required dependencies: pip install torch torchvision opencv-python")
    sys.exit(1)

print("YOLOv5 model loading completed successfully!")

# Socket setup
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '192.168.0.254'  # Jetson Nano's IP address
port = 8581  # Port matching server
client_socket.connect((host, port))
print(f"Connected to server at {host}:{port}")

conf_threshold = 0.70  # Confidence threshold
bbox_color = (0, 255, 0)  # Green for YOLOv5 detections
font = cv2.FONT_HERSHEY_SIMPLEX


try:
    while True:
        # Receive frame size (4 bytes)
        size_data = b''
        while len(size_data) < 4:
            chunk = client_socket.recv(4 - len(size_data))
            if not chunk:
                raise ConnectionError("Server disconnected")
            size_data += chunk
        size = int.from_bytes(size_data, byteorder='big')

        # Receive frame data
        data = b''
        while len(data) < size:
            chunk = client_socket.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Server disconnected")
            data += chunk

        # Decode frame
        frame = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        if frame is None:
            print("Error: Failed to decode frame")
            continue

        # Convert BGR to RGB for YOLOv5
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # YOLOv5 inference (traditional YOLOv5 only)
        try:
            # YOLOv5 inference with autocast for better performance
            with torch.amp.autocast('cuda' if torch.cuda.is_available() else 'cpu'):
                results = model(rgb_frame)
            detections = results.xyxy[0].cpu().numpy()
        except Exception as e:
            print(f"Inference error: {e}")
            detections = []

        # Draw YOLOv5 detections (traditional format)
        for *box, conf, cls in detections:
            if conf < conf_threshold:
                continue
                
            x1, y1, x2, y2 = map(int, box)
            
            # Get class name safely from YOLOv5 model
            try:
                if hasattr(model, 'names'):
                    class_name = model.names[int(cls)]
                elif hasattr(model, 'module') and hasattr(model.module, 'names'):
                    class_name = model.module.names[int(cls)]
                else:
                    class_name = f"Class_{int(cls)}"
            except (KeyError, IndexError, AttributeError):
                class_name = f"Class_{int(cls)}"
                
            label = f'{class_name}: {conf:.2f}'
            cv2.rectangle(frame, (x1, y1), (x2, y2), bbox_color, 2)
            (w, h), _ = cv2.getTextSize(label, font, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - 25), (x1 + w + 5, y1), (0, 128, 0), -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 7), font, 0.6, (0, 0, 0), 2)  # Border
            cv2.putText(frame, label, (x1 + 2, y1 - 7), font, 0.6, (255, 255, 255), 1)  # Text

        # Display frame
        cv2.imshow('YOLOv5 + QR - Received Stream', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"Error: {e}")
finally:
    client_socket.close()
    cv2.destroyAllWindows()