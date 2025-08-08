import depthai as dai
import cv2
import socket
import json
from threading import Thread, Lock
import time
from pyzbar import pyzbar
import requests
import numpy as np

HOST = '0.0.0.0'  # Escuchar en todas las interfaces
PORT_VIDEO = 8480
PORT_JSON = 8481

CLASSES = [
    'hydrant', 'aquatic_toxicity', 'fire_extinguisher', 'high_voltage',
    'exclamation_mark', 'healthy_hazard', 'biohazard', 'radioactive',
    'fire_hose_reel', 'stairway', 'emergency_exit', 'low_temp',
    'corrosive', 'non_flam_gas', 'magnetic_field', 'danger_of_death',
    'oxidizer', 'explosive', 'flammable', 'laser_radiation'
]

BLOB_PATH = "este.blob"
INPUT_SIZE = 640
CONF_THRESHOLD = 0.75
IOU_THRESHOLD = 0.3

# === Compartir frame y detecciones entre hilos ===
shared_frame = None
frame_lock = Lock()
detections_lock = Lock()
last_processed_detections = []
last_qr_infos = []

# === Utils ===
def iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union_area = box1_area + box2_area - inter_area
    return inter_area / union_area if union_area > 0 else 0

def non_max_suppression(detections):
    detections = sorted(detections, key=lambda x: x['conf'], reverse=True)
    filtered = []
    while detections:
        best = detections.pop(0)
        filtered.append(best)
        detections = [
            d for d in detections
            if d['class_id'] != best['class_id'] or iou(d['bbox'], best['bbox']) < IOU_THRESHOLD
        ]
    return filtered

# === Dibujar detecciones y QR ===
def dibujar_detecciones_y_qr(frame, detections, qr_infos):
    #print(f"🖌️ Dibujando: {len(detections)} detecciones, {len(qr_infos)} QRs")
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        class_name = CLASSES[det['class_id']]
        conf = det['conf']
        #print(f"🖌️ Dibujando bbox: {det['bbox']} para clase {class_name}")
        # Validar coordenadas
        if x1 < 0 or y1 < 0 or x2 > INPUT_SIZE or y2 > INPUT_SIZE or x1 >= x2 or y1 >= y2:
            print(f"⚠️ Coordenadas inválidas para {class_name}: {det['bbox']}")
            continue
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{class_name}: {conf:.2f}"
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    for qr in qr_infos:
        qr_data = qr['data']
        qrcodes = pyzbar.decode(frame)
        for qr_code in qrcodes:
            if qr_code.data.decode('utf-8') == qr_data:
                points = qr_code.polygon
                if len(points) > 4:
                    hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                    cv2.polylines(frame, [hull], True, (255, 0, 0), 2)
                else:
                    for i in range(len(points)):
                        cv2.line(frame, points[i], points[(i + 1) % len(points)], (255, 0, 0), 2)
                x, y = qr_code.rect.left, qr_code.rect.top
                cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    return frame

# === Pipeline ===
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(INPUT_SIZE, INPUT_SIZE)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

control = dai.CameraControl()
control.setAutoExposureLock(True)

xin_control = pipeline.create(dai.node.XLinkIn)
xin_control.setStreamName("control")
xin_control.out.link(cam_rgb.inputControl)

nn = pipeline.create(dai.node.YoloDetectionNetwork)
nn.setBlobPath(BLOB_PATH)
nn.setConfidenceThreshold(CONF_THRESHOLD)
nn.setNumClasses(len(CLASSES))
nn.setCoordinateSize(4)
anchors = [10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326]
nn.setAnchors(anchors)
nn.setAnchorMasks({"side80": [0, 1, 2], "side40": [3, 4, 5], "side20": [6, 7, 8], "side24": [9, 10, 11]})
nn.setIouThreshold(IOU_THRESHOLD)
cam_rgb.preview.link(nn.input)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

xout_nn = pipeline.create(dai.node.XLinkOut)
xout_nn.setStreamName("nn")
nn.out.link(xout_nn.input)

# === Funciones de manejo de clientes ===
def actualizar_frame(q_rgb):
    global shared_frame
    while True:
        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()
        with frame_lock:
            shared_frame = frame

def manejar_cliente_video(conn, q_rgb):
    print("📺 Cliente de video conectado")
    while True:
        try:
            with frame_lock:
                if shared_frame is None:
                    continue
                frame = shared_frame.copy()

            with detections_lock:
                detections = last_processed_detections
                qr_infos = last_qr_infos

            frame = dibujar_detecciones_y_qr(frame, detections, qr_infos)

            success, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not success:
                print("⚠️ Error codificando frame JPEG")
                continue

            jpg_bytes = encoded.tobytes()
            conn.sendall(len(jpg_bytes).to_bytes(4, 'big'))
            conn.sendall(jpg_bytes)
            #print(f"▶️ Frame enviado, tamaño: {len(jpg_bytes)} bytes")
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"❌ Error enviando frame: {e}")
            break
        except Exception as e:
            print(f"⚠️ Error inesperado: {e}")
            continue

    conn.close()
    cv2.destroyAllWindows()
    print("🛑 Cliente de video desconectado")

def manejar_cliente_json(conn, q_nn):
    print("🧠 Cliente de JSON conectado")
    global last_processed_detections, last_qr_infos
    last_qr_data = None
    last_final_url = None

    while True:
        in_nn = q_nn.get()
        detections = in_nn.detections
        processed_detections = []

        for det in detections:
            if det.confidence < CONF_THRESHOLD or det.label >= len(CLASSES):
                continue
            x1 = int(det.xmin * INPUT_SIZE)
            y1 = int(det.ymin * INPUT_SIZE)
            x2 = int(det.xmax * INPUT_SIZE)
            y2 = int(det.ymax * INPUT_SIZE)
            #print(f"👀 Det cruda: label={det.label}, conf={det.confidence}, bbox=[{det.xmin}, {det.ymin}, {det.xmax}, {det.ymax}]")
            processed_detections.append({
                'class_id': det.label,
                'conf': det.confidence,
                'bbox': [x1, y1, x2, y2]
            })

        processed_detections = non_max_suppression(processed_detections)

        with frame_lock:
            if shared_frame is None:
                continue
            frame = shared_frame.copy()

        qrcodes = pyzbar.decode(frame)
        qr_infos = []
        for qr in qrcodes:
            qr_data = qr.data.decode('utf-8')
            qr_type = qr.type
            final_url = None
            if qr_data != last_qr_data:
                try:
                    response = requests.get(qr_data, timeout=3, allow_redirects=True)
                    final_url = response.url
                except Exception as e:
                    final_url = f"Error: {e}"
                last_qr_data = qr_data
                last_final_url = final_url
            else:
                final_url = last_final_url
            qr_infos.append({'data': qr_data, 'type': qr_type, 'final_url': final_url})

        with detections_lock:
            last_processed_detections = processed_detections
            last_qr_infos = qr_infos

        msg = {
            'symbols': [{
                'class': CLASSES[det['class_id']],
                'conf': float(det['conf']),
                'bbox': det['bbox']
            } for det in processed_detections],
            'qrcodes': qr_infos
        }

        try:
            json_msg = json.dumps(msg)
            conn.sendall(json_msg.encode('utf-8'))
            #print(f"📤 JSON enviado: {json_msg}")
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"❌ Error enviando JSON: {e}")
            break
        except Exception as e:
            print(f"❌ Error inesperado enviando JSON: {e}")
            break

    conn.close()
    print("🛑 Cliente de JSON desconectado")

def main():
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT_VIDEO))
            s.listen(1)
            print(f"🟢 Esperando cliente de video en puerto {PORT_VIDEO}...")
            conn_video, addr_video = s.accept()
            print(f"Cliente de video conectado desde {addr_video}")

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
                s2.bind((HOST, PORT_JSON))
                s2.listen(1)
                print(f"🟢 Esperando cliente de JSON en puerto {PORT_JSON}...")
                conn_json, addr_json = s2.accept()
                print(f"Cliente de JSON conectado desde {addr_json}")

                with dai.Device(pipeline) as device:
                    control_queue = device.getInputQueue("control")
                    control_queue.send(control)
                    q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
                    q_nn = device.getOutputQueue("nn", maxSize=4, blocking=False)

                    Thread(target=actualizar_frame, args=(q_rgb,), daemon=True).start()
                    Thread(target=manejar_cliente_video, args=(conn_video, q_rgb), daemon=True).start()
                    Thread(target=manejar_cliente_json, args=(conn_json, q_nn), daemon=True).start()

                    try:
                        while True:
                            time.sleep(1)
                    except KeyboardInterrupt:
                        print("\n🛑 Servidor detenido")
                        break

if __name__ == '__main__':
    main()