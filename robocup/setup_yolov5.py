#!/usr/bin/env python3
"""
Script para configurar YOLOv5 localmente sin dependencias de internet
"""

import os
import subprocess
import sys
import urllib.request
import zipfile

def download_yolov5():
    """Descarga YOLOv5 desde GitHub"""
    print("Descargando YOLOv5...")
    
    # URL del repositorio YOLOv5
    url = "https://github.com/ultralytics/yolov5/archive/refs/heads/master.zip"
    
    try:
        # Descargar el archivo ZIP
        urllib.request.urlretrieve(url, "yolov5-master.zip")
        print("✓ YOLOv5 descargado exitosamente")
        
        # Extraer el archivo
        with zipfile.ZipFile("yolov5-master.zip", 'r') as zip_ref:
            zip_ref.extractall(".")
        
        # Renombrar la carpeta
        if os.path.exists("yolov5-master"):
            if os.path.exists("yolov5"):
                subprocess.run(["rm", "-rf", "yolov5"], check=True)
            os.rename("yolov5-master", "yolov5")
        
        # Limpiar archivo ZIP
        os.remove("yolov5-master.zip")
        print("✓ YOLOv5 extraído y configurado")
        
        return True
        
    except Exception as e:
        print(f"✗ Error descargando YOLOv5: {e}")
        return False

def install_dependencies():
    """Instala las dependencias necesarias"""
    print("Instalando dependencias...")
    
    try:
        # Instalar dependencias básicas
        subprocess.run([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"], check=True)
        
        # Instalar dependencias específicas de YOLOv5
        if os.path.exists("yolov5/requirements.txt"):
            subprocess.run([sys.executable, "-m", "pip", "install", "-r", "yolov5/requirements.txt"], check=True)
        
        print("✓ Dependencias instaladas exitosamente")
        return True
        
    except Exception as e:
        print(f"✗ Error instalando dependencias: {e}")
        return False

def test_model_loading():
    """Prueba la carga del modelo"""
    print("Probando carga del modelo...")
    
    try:
        import torch
        
        # Intentar cargar desde caché o local
        sys.path.append('./yolov5')
        
        # Método 1: torch.hub con caché
        try:
            model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True, force_reload=False)
            print("✓ Modelo cargado desde torch.hub")
            return True
        except:
            pass
        
        # Método 2: carga local
        if os.path.exists('./yolov5'):
            from models.experimental import attempt_load
            from utils.torch_utils import select_device
            
            # Descargar modelo base si no existe
            if not os.path.exists("yolov5s.pt"):
                print("Descargando modelo base yolov5s.pt...")
                urllib.request.urlretrieve("https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt", "yolov5s.pt")
            
            device = select_device('')
            model = attempt_load("yolov5s.pt", map_location=device)
            model.eval()
            print("✓ Modelo cargado desde instalación local")
            return True
            
    except Exception as e:
        print(f"✗ Error probando modelo: {e}")
        return False

def main():
    print("=== Configuración de YOLOv5 para uso offline ===\n")
    
    # Verificar si ya existe YOLOv5 localmente
    if os.path.exists("yolov5"):
        print("✓ YOLOv5 ya existe localmente")
    else:
        if not download_yolov5():
            print("❌ No se pudo descargar YOLOv5")
            return False
    
    # Instalar dependencias
    if not install_dependencies():
        print("❌ No se pudieron instalar las dependencias")
        return False
    
    # Probar carga del modelo
    if not test_model_loading():
        print("❌ No se pudo cargar el modelo")
        return False
    
    print("\n✅ YOLOv5 configurado exitosamente para uso offline!")
    print("Ahora puedes ejecutar 'python3 hazmat.py' sin conexión a internet")
    
    return True

if __name__ == "__main__":
    main()
