import socket
import cv2
import numpy as np
import torch
from ultralytics import YOLO
import threading
import queue
import time
import pathlib
import platform
import os

# Workaround for Windows with PosixPath
if platform.system() == 'Windows':
    pathlib.PosixPath = pathlib.WindowsPath

# Load YOLOv8 model
model_path = "/home/chumbi/camaras/src/yolov8n1.pt"
if not os.path.exists(model_path):
    print(f"Model file {model_path} not found. Downloading...")
    model = YOLO(model_path)
else:
    model = YOLO(model_path)

# Set device and optimize model
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")
model.to(device)
model.eval()
if device == 'cuda':
    model.model.half()
    print("Model converted to FP16 for GPU inference")

# List of allowed classes (0 to 18 for 19 classes)
allowed_classes = list(range(19))

# Class names
class_names = [
    "flammable-gas", "explosive", "corrosive", "spontaneously-combustible",
    "non-flammable-gas", "infectious-substance", "blasting agent", "flammable-solid",
    "radioactive", "biohazard", "poison", "inhalation_hazard", "flammable",
    "organic-peroxide", "dangerous-wet", "flammable-liquid", "oxidizer", "fuel oil", "oxygen"
]

# Socket setup
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '192.168.0.254'  # Jetson Nano's IP address
port = 8581  # Port matching server
client_socket.connect((host, port))
print(f"Connected to server at {host}:{port}")

# Frame queue for multithreading
frame_queue = queue.Queue(maxsize=5)
target_fps = 20
frame_time = 1.0 / target_fps

def receive_frames():
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
            chunk = client_socket.recv(min(16384, size - len(data)))
            if not chunk:
                raise ConnectionError("Server disconnected")
            data += chunk

        # Decode frame
        frame = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        if frame is not None:
            if frame_queue.qsize() > 2:
                frame_queue.get()  # Drop old frames to reduce latency
            frame_queue.put(frame)

def process_frames():
    while True:
        start_time = time.time()
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        
        # Inferencia (usa source=frame para pasar imagen)
        results = model.predict(source=frame, imgsz=960, conf=0.2, iou=0.4, max_det=100, verbose=False)
        
        # Accede a detecciones
        detections = results[0]  # resultados de la primera imagen (frame)
        
        # Filtra detecciones para solo clases permitidas
        boxes = detections.boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            if cls_id in allowed_classes:
                # Extraer coordenadas
                xyxy = box.xyxy[0].cpu().numpy().astype(int)  # [x1, y1, x2, y2]
                conf = box.conf[0].cpu().numpy()
                
                # Dibujar caja y etiqueta
                label = f"{class_names[cls_id]} {conf:.2f}"
                cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (32,255,0), 3)
                cv2.putText(frame, label, (xyxy[0], xyxy[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                print(f"Detected: {label} at {xyxy}")
        
        # Mostrar frame con detecciones
        cv2.imshow('YOLOv8 Real-Time Detection', frame)
        
        # Control frame rate
        elapsed_time = time.time() - start_time
        sleep_time = max(0, frame_time - elapsed_time)
        time.sleep(sleep_time)
        
        # Salir con tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

try:
    threading.Thread(target=receive_frames, daemon=True).start()
    process_frames()
except Exception as e:
    print(f"Error: {e}")
finally:
    client_socket.close()
    cv2.destroyAllWindows()