import socket
import cv2
import numpy as np
import threading
import json
import time 

HOST = '192.168.0.254'  # Replace with the Jetson Nano's IP address
PORT_VIDEO = 8482
PORT_JSON = 8483

def recibir_video():
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT_VIDEO))
                print("✅ Conectado al servidor de video")
                while True:
                    # Leer el tamaño del frame (4 bytes)
                    size_data = s.recv(4)
                    if not size_data:
                        print("⚠️ Servidor de video desconectado")
                        break
                    img_size = int.from_bytes(size_data, 'big')
                    
                    # Leer los datos del frame
                    img_data = b''
                    while len(img_data) < img_size:
                        chunk = s.recv(img_size - len(img_data))
                        if not chunk:
                            print("⚠️ Conexión de video rota")
                            break
                        img_data += chunk
                    
                    # Decodificar el frame
                    img_array = np.frombuffer(img_data, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    if frame is None:
                        print("⚠️ Error decodificando frame")
                        continue
                    
                    # Mostrar el frame
                    cv2.imshow("Video recibido", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except (ConnectionRefusedError, ConnectionResetError) as e:
            print(f"❌ Error en conexión de video: {e}")
            print("🔄 Intentando reconectar en 5 segundos...")
            cv2.destroyAllWindows()
            time.sleep(5)
            continue
        except Exception as e:
            print(f"⚠️ Error inesperado en video: {e}")
            break
    
    cv2.destroyAllWindows()
    print("🛑 Cliente de video detenido")

def recibir_json():
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT_JSON))
                print("✅ Conectado al servidor de JSON")
                data_buffer = ""
                while True:
                    data = s.recv(4096).decode('utf-8')
                    if not data:
                        print("⚠️ Servidor de JSON desconectado")
                        break
                    data_buffer += data
                    try:
                        # Procesar todos los JSON completos en el búfer
                        while data_buffer:
                            try:
                                json_obj, idx = json.JSONDecoder().raw_decode(data_buffer)
                                print("📦 JSON recibido:", json_obj)
                                data_buffer = data_buffer[idx:].lstrip()
                            except json.JSONDecodeError:
                                break  # Esperar más datos si el JSON está incompleto
                    except Exception as e:
                        print(f"❌ Error procesando JSON: {e}")
        except (ConnectionRefusedError, ConnectionResetError) as e:
            print(f"❌ Error en conexión de JSON: {e}")
            print("🔄 Intentando reconectar en 5 segundos...")
            time.sleep(5)
            continue
        except Exception as e:
            print(f"⚠️ Error inesperado en JSON: {e}")
            break
    
    print("🛑 Cliente de JSON detenido")

# Lanzar ambos hilos
threading.Thread(target=recibir_video, daemon=True).start()
threading.Thread(target=recibir_json, daemon=True).start()

# Mantener main vivo
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("⛔ Cliente detenido")
