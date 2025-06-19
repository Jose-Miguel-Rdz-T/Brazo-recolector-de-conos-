#!/usr/bin/env python3

from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
import math

# Configuración de la comunicación UART con ESP32
uart = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)

# Configuración de la cámara
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Rangos de color para los conos en HSV
LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([30, 255, 255])

LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([180, 255, 255])

# Distancia mínima para actuar (25 cm)
MIN_DISTANCE = 25

def calculate_distance(width_pixels):
    """Estimar distancia basada en el ancho del objeto en píxeles"""
    # Calibración empírica: a 25 cm, un cono ocupa aproximadamente 150 píxeles
    focal_length = (150 * 25) / 7.5  # 7.5 cm es el ancho real aproximado de un cono
    distance = (7.5 * focal_length) / width_pixels
    return distance

def detect_cones(frame):
    """Detecta conos amarillos y rojos en el frame"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Máscaras para los colores
    mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    mask_red1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask_red2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    # Operaciones morfológicas para mejorar las máscaras
    kernel = np.ones((5,5), np.uint8)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    
    # Encontrar contornos
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cones = []
    
    # Procesar conos amarillos
    for cnt in contours_yellow:
        if cv2.contourArea(cnt) > 500:  # Filtrar por tamaño
            x, y, w, h = cv2.boundingRect(cnt)
            distance = calculate_distance(w)
            cones.append(('yellow', (x, y, w, h), distance))
    
    # Procesar conos rojos
    for cnt in contours_red:
        if cv2.contourArea(cnt) > 500:  # Filtrar por tamaño
            x, y, w, h = cv2.boundingRect(cnt)
            distance = calculate_distance(w)
            cones.append(('red', (x, y, w, h), distance))
    
    return cones

def send_command(command):
    """Envía un comando a la ESP32 por UART"""
    uart.write((command + '\n').encode())
    time.sleep(0.1)  # Pequeña pausa para asegurar la transmisión

def avoid_line():
    """Rutina para evitar la línea negra"""
    send_command("AVOID_LINE")

def move_forward():
    """Mover el carro hacia adelante"""
    send_command("MOVE_FORWARD")

def stop_movement():
    """Detener el carro"""
    send_command("STOP")

def collect_cone():
    """Rutina para recoger cono amarillo"""
    send_command("COLLECT_CONE")
    time.sleep(2)  # Tiempo estimado para completar la acción

def pre_release_cone():
    """Rutina para preparar la liberación del cono"""
    send_command("PRE_RELEASE_CONE")
    time.sleep(2)  # Tiempo estimado para completar la acción

def push_cone():
    """Rutina para empujar cono rojo"""
    send_command("PUSH_CONE")
    time.sleep(2)  # Tiempo estimado para completar la acción

def home_position():
    """Regresar a posición inicial"""
    send_command("HOME_POSITION")
    time.sleep(2)  # Tiempo estimado para completar la acción

def main():
    try:
        while True:
            # Capturar frame de la cámara
            frame = picam2.capture_array()
            
            # Convertir de RGB a BGR (OpenCV usa BGR)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Detectar conos
            cones = detect_cones(frame)
            
            # Procesar cada cono detectado
            for color, (x, y, w, h), distance in cones:
                # Dibujar rectángulo y texto
                color_bgr = (0, 255, 255) if color == 'yellow' else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color_bgr, 2)
                cv2.putText(frame, f"{color} {distance:.1f}cm", (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)
                
                # Si está dentro de la distancia mínima, actuar
                if distance <= MIN_DISTANCE:
                    if color == 'yellow':
                        # Rutina para cono amarillo
                        stop_movement()
                        collect_cone()
                        pre_release_cone()
                        home_position()
                        move_forward()
                    elif color == 'red':
                        # Rutina para cono rojo
                        stop_movement()
                        push_cone()
                        home_position()
                        move_forward()
            
            # Mostrar el frame
            cv2.imshow('Frame', frame)
            
            # Verificar si se presiona 'q' para salir
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # Pequeña pausa para evitar sobrecarga
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        # Limpieza
        picam2.stop()
        cv2.destroyAllWindows()
        uart.close()

if __name__ == "__main__":
    main()