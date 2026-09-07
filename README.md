# Sistema de Visión Artificial RGB-D y Control Robótico

![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)
![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Ready-green.svg)

## 📌 Descripción General

Este repositorio contiene un nodo de ROS diseñado para actuar como el sistema de percepción y control de un brazo robótico (u otro sistema mecanizado). Utiliza imágenes RGB y de profundidad (Depth) provenientes de un sensor como la cámara **Kinect**. 

El núcleo actual del sistema emplea la librería **MediaPipe** para detectar y extraer puntos clave (keypoints) anatómicos (postura corporal, manos, rostro). Una de las características clave es que a cada punto detectado en 2D se le asigna su profundidad real en metros consultando directamente el mapa de profundidad, obteniendo así **coordenadas 3D (X, Y, Z)**.

## 🚀 Características y Etapas del Proyecto

El código está estructurado modularmente para abarcar toda la tubería desde la visión hasta el control físico. Actualmente incluye las primeras etapas, con planes de expansión:

### 🟢 1. Captura y Procesamiento de Visión (Implementado)
- **Sincronización RGB-D:** Sincronización temporal de los tópicos de imagen de color y profundidad mediante `message_filters`.
- **MediaPipe Streaming:** Ejecución de los modelos de MediaPipe (Holistic, Pose, Face Mesh, Objectron).
- **Mapeo 3D:** Asignación de profundidad real a cada *landmark* detectado.
- **Publicación ROS:** Envío de los puntos detectados y procesados a través de tópicos de ROS personalizados (`/mediapipe_gesture_recognition/...`).

### 🟡 2. Transformación de Coordenadas (Próximamente)
- Conversión de las coordenadas espaciales obtenidas (cuyo origen es la cámara) al **sistema de referencia de la base del robot**. Esto permitirá que el robot entienda dónde están los objetivos en su propio espacio de trabajo.

### 🟡 3. Cinemática Inversa (Próximamente)
- Cálculo matemático de los **ángulos articulares** necesarios. A partir de las coordenadas (X, Y, Z) deseadas, el sistema calculará cuánto debe rotar cada motor/articulación para alcanzar dicha posición.

### 🟡 4. Comunicación por Hardware (Próximamente)
- **Envío de datos por SPI:** Transmisión de las variables de control (ángulos, velocidades) a un microcontrolador esclavo (ej. Arduino, STM32, ESP32) que se encargará de accionar físicamente los motores a bajo nivel.

## ⚙️ Configuración y Parámetros

El nodo principal (`Stream_node.py`) es altamente configurable mediante parámetros de ROS, permitiendo activar o desactivar módulos de MediaPipe según la necesidad (para ahorrar recursos computacionales):

- `~enable_right_hand` / `~enable_left_hand`
- `~enable_pose`
- `~enable_face` / `~enable_face_detection`
- `~enable_objectron`

## 🛠️ Requisitos Previos

- **ROS** (Recomendado Noetic en Ubuntu 20.04)
- **Cámara RGB-D** compatible con ROS (ej. Xbox Kinect) con sus respectivos drivers (Se recomienda el uso de `openni_launch`, no de `freenect_launch`).
- **Dependencias de Python:**
  - `rospy`, `cv_bridge`
  - `opencv-python` (`cv2`)
  - `numpy`
  - `mediapipe`
  - `termcolor`

## 🏃 Modo de Uso

*(Instrucciones detalladas de compilación y ejecución se añadirán más adelante a medida que el proyecto avance).*

1. Asegúrate de tener los drivers de la cámara corriendo y publicando en `/camera/rgb/image_raw/compressed` y `/camera/depth/image_raw`.
2. Ejecuta el nodo de visión:
   ```bash
   rosrun <nombre_del_paquete> Stream_node.py
   ```

---
*Este proyecto está en desarrollo activo. La arquitectura final integrará visión por computadora, cálculo robótico y control de hardware de bajo nivel en un flujo continuo.*
