import cv2
import time
import os

# Force UDP for RTSP
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

# Stream RTSP
RTSP_URL = "rtsp://192.168.0.203:554/stream1"

# Parameters
MIN_AREA = 10
SKIP_FRAMES = 10
DISPLAY_WIDTH = 800
MARGIN_PERCENT = 0.10
QR_FPS = 5
QR_SCALE_FACTOR = 1  # Downscale factor for QR detection

# Initialize video
vs = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
if not vs.isOpened():
    print("Error: Could not open RTSP stream")
    exit()

vs.set(cv2.CAP_PROP_BUFFERSIZE, 200)

avg = None
frame_count = 0
qr_detector = cv2.QRCodeDetector()
motion_boxes = []
last_qr_time = 0
qr_data_cached = None
qr_pts_cached = None

while True:
    start_time = time.time()
    grabbed, frame = vs.read()
    if not grabbed:
        print("Stream dropped, attempting to reconnect...")
        vs.release()
        vs = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
        vs.set(cv2.CAP_PROP_BUFFERSIZE, 200)
        time.sleep(1)
        continue

    frame_count += 1
    (h, w) = frame.shape[:2]
    aspect_ratio = h / w
    display_height = int(DISPLAY_WIDTH * aspect_ratio)
    frame_display = cv2.resize(frame, (DISPLAY_WIDTH, display_height)).copy()

    gray = cv2.cvtColor(frame_display, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    if avg is None:
        avg = gray.copy().astype("float")
        continue

    if frame_count % SKIP_FRAMES == 0:
        cv2.accumulateWeighted(gray, avg, 0.05)
        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
        thresh = cv2.threshold(frameDelta, 30, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        motion_boxes.clear()
        for c in cnts:
            if cv2.contourArea(c) < MIN_AREA:
                continue
            (x, y, w_box, h_box) = cv2.boundingRect(c)
            margin_x = int(DISPLAY_WIDTH * MARGIN_PERCENT)
            if x < margin_x or (x + w_box) > (DISPLAY_WIDTH - margin_x):
                continue
            motion_boxes.append((x, y, w_box, h_box))

    for (x, y, w_box, h_box) in motion_boxes:
        cv2.rectangle(frame_display, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
        cv2.putText(frame_display, "Movimiento", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    current_time = time.time()
    if current_time - last_qr_time >= 1.0 / QR_FPS:
        # Downscale frame for faster QR detection
        qr_frame = cv2.resize(frame, (int(w * QR_SCALE_FACTOR), int(h * QR_SCALE_FACTOR)))
        qr_data, qr_pts, _ = qr_detector.detectAndDecode(qr_frame)
        if qr_data and qr_pts is not None:
            last_qr_time = current_time
            qr_data_cached = qr_data
            # Scale QR points back to original frame size
            qr_pts_cached = qr_pts * (1 / QR_SCALE_FACTOR)
        else:
            qr_data_cached = None
            qr_pts_cached = None

    if qr_data_cached and qr_pts_cached is not None and current_time - last_qr_time <= 1.0:
        pts = qr_pts_cached[0].astype(int)
        scale_x = DISPLAY_WIDTH / w
        scale_y = display_height / h
        pts_scaled = [(int(x * scale_x), int(y * scale_y)) for (x, y) in pts]
        for i in range(len(pts_scaled)):
            pt1 = pts_scaled[i]
            pt2 = pts_scaled[(i + 1) % 4]
            cv2.line(frame_display, pt1, pt2, (0, 0, 255), 2)
        cv2.putText(frame_display, f"QR: {qr_data_cached}", (pts_scaled[0][0], pts_scaled[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        print(f"QR Code detected: {qr_data_cached}")

    elapsed_time = time.time() - start_time
    fps = 1.0 / elapsed_time if elapsed_time > 0 else 0.0
    cv2.putText(frame_display, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    cv2.imshow("Frame", frame_display)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

vs.release()
cv2.destroyAllWindows()