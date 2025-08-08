import argparse
import time
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="Ruta al video o RTSP")
ap.add_argument("-a", "--min-area", type=int, default=10)
ap.add_argument("-s", "--skip-frames", type=int, default=10)
args = vars(ap.parse_args())

vs = cv2.VideoCapture(0 if args["video"] is None else args["video"])
time.sleep(2.0)

avg = None
frame_count = 0
detector = cv2.QRCodeDetector()
display_width = 640

motion_boxes = []

while True:
    start_time = time.time()
    grabbed, frame = vs.read()
    if not grabbed:
        break

    frame_count += 1
    (h, w) = frame.shape[:2]
    aspect_ratio = h / w
    display_height = int(display_width * aspect_ratio)

    frame_display = cv2.resize(frame, (display_width, display_height))

    gray = cv2.cvtColor(frame_display, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)  # Kernel más pequeño para acelerar

    if avg is None:
        avg = gray.copy().astype("float")
        continue

    if frame_count % args["skip_frames"] == 0:
        cv2.accumulateWeighted(gray, avg, 0.05)
        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
        thresh = cv2.threshold(frameDelta, 30, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)

        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        motion_boxes.clear()
        for c in cnts:
            if cv2.contourArea(c) < args["min_area"]:
                continue
            (x, y, w_box, h_box) = cv2.boundingRect(c)
            motion_boxes.append((x, y, w_box, h_box))

    for (x, y, w_box, h_box) in motion_boxes:
        cv2.rectangle(frame_display, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
        cv2.putText(frame_display, "Movimiento", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    qr_data, qr_pts, _ = detector.detectAndDecode(frame)
    if qr_data and qr_pts is not None:
        pts = qr_pts[0].astype(int)
        scale_x = display_width / w
        scale_y = display_height / h
        pts_scaled = [(int(x * scale_x), int(y * scale_y)) for (x, y) in pts]
        for i in range(len(pts_scaled)):
            cv2.line(frame_display, pts_scaled[i], pts_scaled[(i + 1) % len(pts_scaled)], (0, 0, 255), 2)
        cv2.putText(frame_display, f"QR: {qr_data}", (pts_scaled[0][0], pts_scaled[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    fps = 1.0 / (time.time() - start_time)
    cv2.putText(frame_display, f"FPS: {fps:.2f}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow("Frame", frame_display)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

vs.release()
cv2.destroyAllWindows()