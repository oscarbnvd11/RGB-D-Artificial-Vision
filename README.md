# Sistema de Visión Artificial RGB-D y Control Robótico

![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)
![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Ready-green.svg)

## 📌 Descripción General

Este repositorio contiene un nodo de ROS diseñado para actuar como el sistema de percepción y control de un brazo robótico (u otro sistema mecanizado). Utiliza imágenes RGB y de profundidad (Depth) provenientes de un sensor como la cámara **Kinect**. 

El núcleo actual del sistema emplea la librería **MediaPipe** para detectar y extraer puntos clave (keypoints) anatómicos (postura corporal, manos, rostro). Una de las características clave es que a cada punto detectado en 2D se le asigna su profundidad real en metros consultando directamente el mapa de profundidad, obteniendo así **coordenadas 3D (X, Y, Z)**.

## 🚀 Características y Etapas del Proyecto

El código está estructurado modularmente para abarcar toda la tubería desde la visión hasta el control físico. Las etapas de percepción RGB-D y mapeo relativo del movimiento ya están implementadas; la cinemática y la comunicación con el hardware quedan pendientes.

### 🟢 1. Captura y Procesamiento de Visión (Implementado)
- **Sincronización RGB-D:** `Stream_node.py` sincroniza temporalmente el RGB comprimido y el mapa de profundidad registrado mediante `message_filters`.
- **Tópicos de entrada por defecto:** `/camera/rgb/image_raw/compressed`, `/camera/depth_registered/image_raw` y `/camera/rgb/camera_info`.
- **MediaPipe Streaming:** Ejecuta MediaPipe Holistic y permite activar pose, manos, rostro, detección facial y Objectron mediante parámetros ROS.
- **Mapeo 3D:** Cada *landmark* 2D se consulta en el depth registrado. La profundidad `16UC1` se convierte de milímetros a metros usando una escala de `0.001`.
- **Proyección métrica:** Las coordenadas se calculan con la matriz intrínseca publicada por `CameraInfo`. Para la cámara usada actualmente: `640x480`, `fx=fy=525.0`, `cx=319.5`, `cy=239.5`.
- **Frame de salida:** Los puntos se publican en `camera_rgb_optical_frame`.
- **Publicación ROS:** La pose se publica en `/mediapipe_gesture_recognition/pose`; también existen tópicos para manos y rostro, pero para este proyecto no serán usados.
- **Visualización opcional:** Con `~publish_debug_image:=true` se publica `/mediapipe_gesture_recognition/debug_image`. La imagen se reduce y se publica con menor frecuencia para no ralentizar mucho el procesamiento.

### 🟢 2. Mapeo de Coordenadas Relativas (Implementado)
- **Nodo:** `Transf_coord_node.py` recibe `/mediapipe_gesture_recognition/pose` y publica `/brazo`.
- **Origen común:** El hombro derecho humano, landmark `13`, se convierte en `(0, 0, 0)`. La posición absoluta de la persona no afecta al resultado.
- **Puntos utilizados:** hombro `13`, codo `15`, muñeca `17`, meñique `19`, índice `21` y pulgar `23`.
- **Mapeo de ejes:** Para el vector relativo cámara `(x, y, z)`, donde X es derecha, Y abajo y Z profundidad, el nodo publica:
  - `x_robot = z_camara` (profundidad/avance)
  - `y_robot = -x_camara` (lateral)
  - `z_robot = -y_camara` (altura)
- **Resultado:** Los campos `x`, `y` y `z` de `/brazo` son coordenadas relativas en metros y son la entrada prevista para la cinemática inversa. El campo `depth` conserva la profundidad absoluta original y solo sirve para validar la medición.
- **Validación:** Se descartan frames sin hombro o muñeca válidos, con profundidad no positiva o con coordenadas no finitas. Las colas de ROS tienen tamaño `1` para evitar retrasos acumulados.

### 🟡 3. Cinemática Inversa (Próximamente)
- Cálculo matemático de los **ángulos articulares** necesarios. A partir de `x`, `y` y `z` de `/brazo`, el sistema calculará cuánto debe rotar cada articulación para imitar el movimiento humano.

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
- **Cámara RGB-D** compatible con ROS (ej. Xbox Kinect) con sus respectivos drivers. Es necesario activar `depth_registration` para que `/camera/depth_registered/image_raw` esté alineado con RGB.
- **Dependencias de Python:**
  - `rospy`, `cv_bridge`
  - `opencv-python` (`cv2`)
  - `numpy`
  - `mediapipe`
  - `termcolor`

## 🏃 Modo de Uso

*(Instrucciones detalladas de compilación y ejecución se añadirán más adelante a medida que el proyecto avance).*

1. Asegúrate de tener los drivers de la cámara corriendo y publicando en `/camera/rgb/image_raw/compressed`, `/camera/depth_registered/image_raw` y `/camera/rgb/camera_info`.
2. Ejecuta el nodo de visión:
   ```bash
   rosrun <nombre_del_paquete> Stream_node.py
   ```
3. Ejecuta el nodo de mapeo relativo:
  ```bash
  rosrun <nombre_del_paquete> Transf_coord_node.py
  ```
4. Comprueba las salidas:
  ```bash
  rostopic echo /mediapipe_gesture_recognition/pose
  rostopic echo /brazo
  ```

Para inspeccionar visualmente la detección sin usarla durante el control:

```bash
rosrun <nombre_del_paquete> Stream_node.py _publish_debug_image:=true
rqt_image_view /mediapipe_gesture_recognition/debug_image
```

---
*Este proyecto está en desarrollo activo. La arquitectura final integrará visión por computadora, cálculo robótico y control de hardware de bajo nivel en un flujo continuo.*
