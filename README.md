# Sistema de Visión Artificial RGB-D y Control Robótico

![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)
![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Ready-green.svg)

## 📌 Descripción General

Este repositorio contiene un nodo de ROS diseñado para actuar como el sistema de percepción y control de un brazo robótico (u otro sistema mecanizado). Utiliza imágenes RGB y de profundidad (Depth) provenientes de un sensor como la cámara **Kinect**. 

El núcleo actual del sistema emplea la librería **MediaPipe** para detectar y extraer puntos clave (keypoints) anatómicos (postura corporal, manos, rostro). Una de las características clave es que a cada punto detectado en 2D se le asigna su profundidad real en metros consultando directamente el mapa de profundidad, obteniendo así **coordenadas 3D (X, Y, Z)**.

## 🚀 Características y Etapas del Proyecto

El código está estructurado modularmente para abarcar toda la tubería desde la visión hasta el control físico. Las etapas de percepción RGB-D, mapeo relativo y cinemática inversa están implementadas. El nodo transmisor SPI está creado y queda pendiente validar la trama con el Arduino conectado.

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
- **Validación:** Se descartan frames sin hombro, codo o muñeca válidos, con profundidad no positiva o con coordenadas no finitas. Las colas de ROS tienen tamaño `1` para evitar retrasos acumulados.

### 🟢 3. Cinemática Inversa (Implementado)
Los nodos `CI_desacoplada_node.py` y `CI_completa_node.py` son alternativas. Se ejecuta uno u otro, nunca los dos a la vez. Ambos reciben `/brazo` y publican en `/robot/joint_angles`:

```text
[q1, q2, q3, q4, q5]
```

- `q1`: giro de la base.
- `q2`: servo del hombro.
- `q3`: servo del codo.
- `q4`: giro de la muñeca.
- `q5`: apertura de la pinza.

Los tres tramos del robot miden `0.065 m`: hombro-codo, codo-muñeca y muñeca-pinza. El punto `17` representa el eje de la muñeca, por lo que la posición de la muñeca tiene un alcance de `0.130 m`; el tercer tramo solo corresponde a la pinza. `q4` y `q5` se calculan con las medidas humanas reales antes del escalado.

#### Opción desacoplada

`CI_desacoplada_node.py` calcula `q1`, `q2` y `q3` directamente con las coordenadas relativas del codo y la muñeca.

#### Opción completa

`CI_completa_node.py` mantiene la estructura de la opción completa, pero publica los ángulos de servo con el mismo convenio que la opción desacoplada. Ambos nodos calculan `q1`, `q2` y `q3` con la misma definición física.

La pinza usa la distancia real entre pulgar `23` e índice `21`:

- Más de `9 cm`: `q5 = 0` grados, pinza abierta.
- Entre `5` y `9 cm`: `q5 = 45` grados.
- Menos de `5 cm`: `q5 = 90` grados, pinza cerrada.

### 🟡 4. Comunicación SPI (Implementación inicial)
`SPI_node.py` se suscribe a `/robot/joint_angles` y transmite cada vector
`[q1, q2, q3, q4, q5]` al Arduino, que debe actuar como esclavo SPI. La trama
actual tiene seis bytes:

```text
0xAA | payload[2:0] | checksum | 0x55
```

El payload son 21 bits empaquetados en orden `q1(5), q2(5), q3(4), q4(5),
q5(2)`, de más a menos significativo. `q1`, `q2` y `q4` se limitan a
`[-90, 90]` y se cuantizan en 30 intervalos de 6 grados; `q3` se limita a
`[0, 90]` y se cuantiza en 15 intervalos. `q5` usa los códigos `0`, `1` y `2`
para los estados `0`, `45` y `90` grados. Los tres bits restantes del payload
quedan a cero. El checksum es `(0xAA + payload[0] + payload[1] + payload[2]) mod
256`.

El bus, dispositivo, velocidad y modo se configuran con `~spi_bus` (0),
`~spi_device` (0), `~spi_speed_hz` (500000) y `~spi_mode` (0). Se requiere el
módulo Python `spidev` en el equipo que ejecuta ROS.

Las tramas válidas se conservan en una cola FIFO y se reintentan si la llamada
SPI devuelve un error local. El protocolo no usa confirmación del Arduino: un
envío que termina sin error no demuestra que el microcontrolador haya recibido
o procesado los datos, por lo que no se pueden detectar pérdidas silenciosas.

El sketch `arduino/SPI_Servo_Slave/SPI_Servo_Slave.ino` está preparado para
Arduino Uno/Nano AVR. Usa SPI esclavo en modo 0, SS en D10 y los servos en D9,
D6, D5, D3 y D2 para `q1` a `q5`. Los niveles de `q1`, `q2` y `q4` se convierten
a posiciones de servo `0..174` grados; `q3` usa `0..84` y `q5` usa `0`, `45` o
`90`. El movimiento se suaviza con un factor `0.06` cada 15 ms, sin bloquear
la recepción SPI. El Arduino arranca en posición neutra, con la pinza abierta. Los bytes
recibidos se comprueban antes de actualizar los servos; los contadores de
tramas inválidas, incompletas y desbordamientos se muestran por Serial a
115200 baudios.

Con una Raspberry Pi como maestro, conecta MOSI, SCLK y CE0/SS a D11, D13 y
D10 del Arduino, y comparte GND. D12/MISO del Uno entrega 5 V: usa un adaptador
de nivel lógico o divisor antes de conectarlo al GPIO de 3,3 V de la Pi. Alimenta
los servos con una fuente externa adecuada y conecta su GND al GND común. Ajusta
los pines del sketch si el cableado real es distinto.

## ⚙️ Configuración y Parámetros

El nodo principal (`Stream_node.py`) es altamente configurable mediante parámetros de ROS, permitiendo activar o desactivar módulos de MediaPipe según la necesidad (para ahorrar recursos computacionales):

- `~enable_right_hand` / `~enable_left_hand`
- `~enable_pose`
- `~enable_face` / `~enable_face_detection`
- `~enable_objectron`

Para este flujo solo es necesario activar `~enable_pose:=true`. Los puntos `19`, `21` y `23` usados para la muñeca y la pinza ya forman parte del mensaje `Pose`; no es necesario activar `~enable_right_hand`.

## 🛠️ Requisitos Previos

- **ROS** (Recomendado Noetic en Ubuntu 20.04)
- **Cámara RGB-D** compatible con ROS (ej. Xbox Kinect) con sus respectivos drivers. Es necesario activar `depth_registration` para que `/camera/depth_registered/image_raw` esté alineado con RGB.
- **Dependencias de Python:**
  - `rospy`, `cv_bridge`
  - `spidev` (para el nodo SPI en el equipo Linux conectado al bus)
  - `opencv-python` (`cv2`)
  - `numpy`
  - `mediapipe`
  - `termcolor`

## 🏃 Modo de Uso

1. Asegúrate de tener los drivers de la cámara corriendo y publicando en `/camera/rgb/image_raw/compressed`, `/camera/depth_registered/image_raw` y `/camera/rgb/camera_info`.
2. Ejecuta el nodo de visión:
   ```bash
   rosrun <nombre_del_paquete> Stream_node.py
   ```
3. Ejecuta el nodo de mapeo relativo:
  ```bash
  rosrun <nombre_del_paquete> Transf_coord_node.py
  ```
4. Ejecuta una sola opción de cinemática:
  ```bash
  rosrun <nombre_del_paquete> CI_desacoplada_node.py
  ```
  o:
  ```bash
  rosrun <nombre_del_paquete> CI_completa_node.py
  ```
5. Comprueba las salidas:
  ```bash
  rostopic echo /mediapipe_gesture_recognition/pose
  rostopic echo /brazo
  rostopic echo /robot/joint_angles
  ```
6. Con el Arduino conectado y configurado para recibir esta trama, inicia el
   transmisor SPI:
   ```bash
   rosrun <nombre_del_paquete> SPI_node.py
   ```

Para inspeccionar visualmente la detección sin usarla durante el control:

```bash
rosrun <nombre_del_paquete> Stream_node.py _publish_debug_image:=true
rqt_image_view /mediapipe_gesture_recognition/debug_image
```

---
*Este proyecto está en desarrollo activo. La arquitectura final integrará visión por computadora, cálculo robótico y control de hardware de bajo nivel en un flujo continuo.*
