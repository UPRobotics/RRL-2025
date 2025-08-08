import cv2
import numpy as np

STREAM_URL = 'rtsp://192.168.0.203:554/stream'  # Cambia esto por la URL real de tu cámara IP

# Rangos HSV para cada color
COLOR_RANGES = {
    "Naranja": (np.array([10, 100, 100]), np.array([25, 255, 255])),
    "Rojo":    (np.array([0, 100, 100]), np.array([10, 255, 255])),
    "Verde":   (np.array([40, 70, 70]), np.array([80, 255, 255])),
    "Azul":    (np.array([100, 150, 0]), np.array([140, 255, 255])),
    "Amarillo":(np.array([20, 100, 100]), np.array([30, 255, 255]))
}

COLOR_DRAW = {
    "Naranja": (0, 140, 255),
    "Rojo": (0, 0, 255),
    "Verde": (0, 255, 0),
    "Azul": (255, 0, 0),
    "Amarillo": (0, 255, 255)
}

cap = cv2.VideoCapture(STREAM_URL)
if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)

    for color_name, (lower, upper) in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                (x, y), radius = cv2.minEnclosingCircle(largest)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(frame, center, radius, COLOR_DRAW[color_name], 2)
                cv2.putText(frame, color_name, (center[0], center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_DRAW[color_name], 2)

    cv2.imshow("HSV Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()