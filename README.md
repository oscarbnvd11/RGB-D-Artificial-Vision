# Sistema de Visión Artificial RGB-D y Control Robótico

![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)
![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Ready-green.svg)

## 📌 Descripción general

Este repositorio contiene los nodos ROS y el firmware Arduino de un sistema
experimental de visión RGB-D y control de un brazo robótico. MediaPipe detecta
la pose corporal; la profundidad de la cámara y su calibración permiten obtener
coordenadas tridimensionales para los puntos necesarios. Los nodos siguientes
transforman esas coordenadas, calculan las órdenes articulares y las transmiten
por SPI al microcontrolador.

El flujo completo se ha probado y validado en el montaje del proyecto,
incluyendo la captura RGB-D, la generación de consignas, la comunicación SPI y
el movimiento del brazo. La calibración, el cableado y los límites mecánicos
dependen de cada montaje y deben comprobarse antes de accionar otro hardware.
La estimación de la pinza descrita más abajo sigue siendo preliminar y queda
como mejora futura.

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

## 🚀 Características y Etapas del Proyecto

### 1. Captura y procesamiento de visión
- `Stream_node.py` sincroniza RGB comprimido, profundidad registrada y
  `CameraInfo`. MediaPipe publica la pose en
  `/mediapipe_gesture_recognition/pose`.
- La Z de cada punto se estima con el percentil 30 de profundidades positivas
  en una ventana de 5×5 (filtro activo por defecto). `16UC1` se convierte de
  milímetros a metros; `32FC1` ya está en metros. Las coordenadas 3D se
  calculan con las intrínsecas de cámara.
- Entradas por defecto: `/camera/rgb/image_raw/compressed`,
  `/camera/depth_registered/image_raw` y `/camera/rgb/camera_info`.

### 2. Mapeo de coordenadas relativas
- `Transf_coord_node.py` toma el hombro derecho (landmark 13) como origen y
  publica la pose relativa en `/brazo`, en metros. Convierte los ejes de cámara
  a robot como `(x, y, z) = (profundidad, -lateral, altura)`. Necesita
  coordenadas válidas del hombro, codo y muñeca.

### 3. Cinemática inversa
Usa solo una alternativa: `CI_desacoplada_node.py` (recomendada) o
`CI_completa_node.py`. Ambas reciben `/brazo` y generan:

```text
[q1, q2, q3, q4, q5]
```

- `q1`: base; `q2`: hombro; `q3`: codo; `q4`: giro de muñeca; `q5`: pinza.
- Ambas opciones calculan `q1`–`q3` a partir de la dirección del brazo. La
  etiqueta “completa” no implica un algoritmo distinto en el flujo actual.
- `q4` se estima con los landmarks de muñeca, meñique e índice; `q5`, con la
  distancia entre pulgar e índice de la pose corporal. Los tramos del robot
  miden 0.065 m por defecto.

### 4. Comunicación SPI
`SPI_node.py` envía los cinco ángulos de `/robot/joint_angles` al Arduino
mediante una trama de seis bytes:

```text
0xAA | payload (3 bytes) | checksum | 0x55
```

El payload empaqueta, en ese orden y empezando por el bit más significativo,
`q1(5 bits), q2(5), q3(4), q4(5), q5(2)`. `q1`, `q2` y `q4` representan
−90° a 84°; `q3`, 0° a 84°, en pasos de 6°. `q5` codifica 0°, 45° o 90°.
El checksum es la suma del byte inicial y los tres bytes del payload, módulo
256.

El firmware `arduino/SPI_Servo_Slave/SPI_Servo_Slave.ino` (Uno/Nano AVR) recibe
como esclavo SPI modo 0 y valida cabecera, terminador, checksum y campos antes
de aplicar la trama; informa contadores por Serial a 115200 baudios. El nodo
maestro usa `spidev` (bus/dispositivo 0, 500 kHz por defecto), cola FIFO y
reintentos ante errores locales. El protocolo no confirma la recepción.

En Raspberry Pi, conecta MOSI/MISO/SCLK/CE0 a D11/D12/D13/D10 y comparte GND.
MISO del Uno entrega 5 V: adapta el nivel antes de conectarlo a la Pi.
Alimenta los servos con una fuente externa adecuada y comparte su GND con el
Arduino.

## ⚙️ Configuración y parámetros

En `Stream_node.py`, `~enable_pose` está activo por defecto. Las detecciones
opcionales de mano derecha, rostro y objetos se activan con sus parámetros
`~enable_right_hand`, `~enable_face`, `~enable_face_detection` y
`~enable_objectron`. La imagen de depuración se activa con
`~publish_debug_image:=true`.

## 🏃 Modo de uso

El repositorio no incluye el paquete catkin ni las definiciones de mensajes.
Instala los scripts en `scripts/` y `launch/rgbd_robot.launch` en `launch/` de
un paquete compilado llamado `mediapipe_gesture_recognition` (o cambia
`ros_package`). Inicia el driver de cámara y ejecuta:

```bash
roslaunch mediapipe_gesture_recognition rgbd_robot.launch
```

Por defecto se usa la cinemática desacoplada. Para usar la alternativa completa:

```bash
roslaunch mediapipe_gesture_recognition rgbd_robot.launch kinematics:=completa
```

Para medir la latencia desde que MediaPipe termina de detectar una pose hasta
que `SPI_node.py` genera la trama (sin incluir la transferencia física SPI),
activa la opción:

```bash
roslaunch mediapipe_gesture_recognition rgbd_robot.launch medir_delay:=true
```

La latencia se mide hasta generar la trama (no incluye su transferencia SPI).
Con `medir_delay:=true`, `/robot/joint_angles` usa `Float64MultiArray` con la
marca temporal seguida de los cinco ángulos; para medir en varios equipos,
sincroniza sus relojes ROS.

El launch no carga el firmware: programa por separado
`arduino/SPI_Servo_Slave/SPI_Servo_Slave.ino`. `/robot/spi_ready` indica que el
nodo ROS abrió SPI; el protocolo no confirma que el Arduino haya recibido cada
trama.

## Mejora futura: estimación de q4 y q5 con la mano derecha

Los nodos de cinemática actuales estiman `q4` y `q5` a partir de landmarks de la
pose corporal. Integrar los landmarks detallados de la mano derecha podría
mejorar la estimación del giro de muñeca (`q4`) y de la apertura de la pinza
(`q5`). `Stream_node.py` puede publicar esos landmarks en
`/mediapipe_gesture_recognition/right_hand` al activar `~enable_right_hand`,
pero todavía no se utilizan en los cálculos de cinemática.

---
*La validación descrita corresponde al montaje utilizado por el proyecto; las
adaptaciones de hardware requieren sus propias comprobaciones.*
