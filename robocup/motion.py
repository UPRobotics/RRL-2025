import cv2
import numpy as np

STREAM_URL = 'rtsp://192.168.0.203:554/stream'  # Replace with your IP camera URL

# HSV ranges for colors, including black
COLOR_RANGES = {
    "Naranja": (np.array([10, 100, 100]), np.array([25, 255, 255])),
    "Rojo": (np.array([0, 100, 100]), np.array([10, 255, 255])),
    "Verde": (np.array([40, 70, 70]), np.array([80, 255, 255])),
    "Azul": (np.array([100, 150, 0]), np.array([140, 255, 255])),
    "Amarillo": (np.array([20, 100, 100]), np.array([30, 255, 255])),
    "Negro": (np.array([0, 0, 0]), np.array([180, 255, 30]))  # Black has low value (V)
}

# Drawing colors in BGR
COLOR_DRAW = {
    "Naranja": (0, 140, 255),
    "Rojo": (0, 0, 255),
    "Verde": (0, 255, 0),
    "Azul": (255, 0, 0),
    "Amarillo": (0, 255, 255),
    "Negro": (255, 255, 255)  # White for visibility on dark objects
}

# Initialize camera
cap = cv2.VideoCapture(STREAM_URL)
if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

# Kernel for morphological operations
kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to HSV once and reuse
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name, (lower, upper) in COLOR_RANGES.items():
        # Create mask for the color
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                if color_name == "Negro":
                    # Detect rotating square using minimum area rectangle
                    rect = cv2.minAreaRect(largest)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    # Draw rotated rectangle
                    cv2.drawContours(frame, [box], 0, COLOR_DRAW[color_name], 2)
                    # Get center for text placement
                    center = (int(rect[0][0]), int(rect[0][1]))
                    cv2.putText(frame, "Square", (center[0], center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_DRAW[color_name], 2)
                else:
                    # For colored objects, draw a circle
                    (x, y), radius = cv2.minEnclosingCircle(largest)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(frame, center, radius, COLOR_DRAW[color_name], 2)
                    cv2.putText(frame, color_name, (center[0], center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_DRAW[color_name], 2)

    # Display the result
    cv2.imshow("Color and Square Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()