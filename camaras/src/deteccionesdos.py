import socket
import cv2
import numpy as np
import torch
import pathlib
import platform

# Workaround for Windows with PosixPath
if platform.system() == 'Windows':
    pathlib.PosixPath = pathlib.WindowsPath

# Initialize YOLOv5 model
model_path = "codigo.pt"  # Ensure this file is on the client computer
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

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

        # YOLOv5 inference with updated autocast
        with torch.amp.autocast('cuda'):
            results = model(rgb_frame)
        detections = results.xyxy[0].cpu().numpy()

        # Draw YOLOv5 detections
        for *box, conf, cls in detections:
            if conf < conf_threshold:
                continue
            x1, y1, x2, y2 = map(int, box)
            class_name = model.names[int(cls)]
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