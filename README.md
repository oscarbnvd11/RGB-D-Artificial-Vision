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

El procesamiento y el protocolo están implementados en software. La
integración y el movimiento físico no se consideran validados: la calibración,
el cableado y la respuesta de los servos deben comprobarse en el montaje real.
La estimación de la pinza descrita más abajo es preliminar y queda como mejora
futura.

## 🚀 Características y Etapas del Proyecto

El código está estructurado modularmente para cubrir el flujo desde la visión
hasta el control físico. Las etapas están implementadas en software, pero la
comunicación SPI y el movimiento deben validarse en el montaje real.

### 1. Captura y procesamiento de visión
- **Sincronización RGB-D:** `Stream_node.py` sincroniza temporalmente el RGB comprimido y el mapa de profundidad registrado mediante `message_filters`.
- **Tópicos de entrada por defecto:** `/camera/rgb/image_raw/compressed`, `/camera/depth_registered/image_raw` y `/camera/rgb/camera_info`.
- **MediaPipe Streaming:** Ejecuta MediaPipe Holistic y permite activar pose, manos, rostro, detección facial y Objectron mediante parámetros ROS.
- **Mapeo 3D:** Para cada *landmark* se toma una ventana de `5x5` píxeles del mapa de profundidad y se usa el percentil 30 de las muestras positivas para reducir errores en los bordes. La profundidad `16UC1` se escala por `0.001` (milímetros a metros); `32FC1` se interpreta directamente en metros.
- **Proyección métrica:** Las coordenadas se calculan con las intrínsecas de `CameraInfo`, ajustadas a la resolución RGB recibida. El mapa de profundidad debe estar registrado/alineado con RGB. La escala de profundidad se puede configurar con `~depth_scale` (por defecto `0.001`).
- **Frame de salida:** Los puntos se publican en `camera_rgb_optical_frame`.
- **Publicación ROS:** La pose corporal se publica en `/mediapipe_gesture_recognition/pose`. La mano derecha e izquierda se publican, respectivamente, en `/mediapipe_gesture_recognition/right_hand` y `/mediapipe_gesture_recognition/left_hand` solo si se habilitan sus parámetros; ya que están desactivadas por defecto.
- **Visualización opcional:** Con `~publish_debug_image:=true` se publica `/mediapipe_gesture_recognition/debug_image`. La imagen se reduce y se publica con menor frecuencia para no ralentizar mucho el procesamiento.

### 2. Mapeo de coordenadas relativas
- **Nodo:** `Transf_coord_node.py` recibe `/mediapipe_gesture_recognition/pose` y publica `/brazo`.
- **Origen común:** El hombro derecho humano, landmark `13`, se convierte en `(0, 0, 0)`. La posición absoluta de la persona no afecta al resultado.
- **Puntos utilizados:** hombro `13`, codo `15`, muñeca `17`, meñique `19`, índice `21` y pulgar `23`.
- **Mapeo de ejes:** Para el vector relativo cámara `(x, y, z)`, donde X es derecha, Y abajo y Z profundidad, el nodo publica:
  - `x_robot = z_camara` (profundidad/avance)
  - `y_robot = -x_camara` (lateral)
  - `z_robot = -y_camara` (altura)
- **Resultado:** Los campos `x`, `y` y `z` de `/brazo` son coordenadas relativas en metros y son la entrada prevista para la cinemática inversa. El campo `depth` conserva la profundidad absoluta original y solo sirve para validar la medición.
- **Validación:** Se descartan poses sin hombro y codo válidos; por defecto también se exige la muñeca. Los puntos deben tener profundidad positiva y coordenadas finitas. Las colas de ROS tienen tamaño `1` para evitar retrasos acumulados.

### 3. Cinemática inversa
Los nodos `CI_desacoplada_node.py` y `CI_completa_node.py` son alternativas. Se ejecuta uno u otro, nunca los dos a la vez. Ambos reciben `/brazo` y publican en `/robot/joint_angles`, cabe destacar que se recomienda el uso de `CI_desacoplada_node.py` debido a que es más ligero para el hardware, más sencillo de entender y está más testeado:

```text
[q1, q2, q3, q4, q5]
```

- `q1`: giro de la base.
- `q2`: servo del hombro.
- `q3`: servo del codo (`0°` flexionado, `90°` extendido).
- `q4`: giro de la muñeca.
- `q5`: apertura de la pinza.

Los tres tramos del robot se configuran por defecto en `0.065 m`: hombro-codo, codo-muñeca y muñeca-pinza. El punto `17` representa el eje de la muñeca, por lo que la posición de la muñeca tiene un alcance de `0.130 m`; el tercer tramo corresponde a la pinza. Actualmente los dos nodos de cinemática leen los puntos de la pose corporal (`Pose`), no el tópico de mano derecha. Por tanto, habilitar `~enable_right_hand` por sí solo no cambia los datos utilizados para controlar la pinza.

#### Nodo desacoplado

`CI_desacoplada_node.py` escala por separado los segmentos hombro-codo y
codo-muñeca a las dimensiones del robot y obtiene `q1`, `q2` y `q3` a partir
de las direcciones del brazo y el antebrazo. Calcula `q5` a partir de la
distancia tridimensional pulgar-índice de los landmarks de `Pose`, filtrada y
clasificada en tres estados mediante `~gripper_closed_threshold` y
`~gripper_open_threshold`.

#### Nodo completo

`CI_completa_node.py` también está disponible como alternativa. En su callback
actual, escala los segmentos y calcula `q1`, `q2` y `q3` con el mismo método
que el nodo desacoplado; aunque contiene una función `solve()`, esta no se usa
en ese flujo. Por tanto, todavía no constituye una implementación
independiente de cinemática inversa completa y debe tenerse en cuenta al
comparar ambas opciones. En esta alternativa, `q5` también se calcula con la
distancia tridimensional pulgar-índice de los landmarks de `Pose`, usando los
mismos umbrales de distancia que el nodo desacoplado.

La muñeca estima el roll a partir de los landmarks `17`, `19` y `21` de `Pose`.
La primera medición válida fija el cero de `q4`; las siguientes se filtran
para reducir cambios bruscos y gestionar el límite entre ángulos positivos y
negativos. Esta estimación también depende de los landmarks de la pose corporal
y no se ha validado con el brazo físico.

#### Limitación conocida y mejora futura

Para el cálculo de `q5`, ambos nodos usan la distancia tridimensional entre los
keypoints de pulgar e índice de `Pose`. Esos keypoints están ubicados en las
falanges, no en las puntas de los dedos, por lo que la distancia no representa
correctamente la separación real entre las puntas ni la apertura de la pinza.

La forma prevista de estimar la apertura usando las puntas de los dedos es
habilitar la detección de la mano derecha (`~enable_right_hand`) y utilizar sus
landmarks detallados, que incluyen las puntas. Actualmente, `Stream_node.py`
puede publicar esos puntos en `/mediapipe_gesture_recognition/right_hand`,
pero no están conectados al cálculo de `q5`. Queda como mejora futura
integrar esa información y validar la estimación y sus umbrales con datos
reales; no se presenta como una función terminada.

### 4. Comunicación SPI
`SPI_node.py` se suscribe a `/robot/joint_angles` y transmite cada vector
`[q1, q2, q3, q4, q5]` al Arduino, que debe actuar como esclavo SPI. La trama
actual tiene seis bytes:

```text
0xAA | payload[2:0] | checksum | 0x55
```

El payload son 21 bits empaquetados en orden `q1(5), q2(5), q3(4), q4(5),
q5(2)`, de más a menos significativo. `q1`, `q2` y `q4` se limitan a
`[-90, 90]` y se cuantizan en pasos de 6 grados; `q3` se limita a `[0, 90]`
con el mismo paso. Debido a la cuantización, el nivel máximo transmitido
representa 84 grados para estos campos. `q5` usa los códigos `0`, `1` y `2`
para los estados `0`, `45` y `90` grados. Los tres bits restantes del payload
quedan a cero. El checksum es `(0xAA + payload[0] + payload[1] + payload[2]) mod
256`.

El bus, dispositivo, velocidad y modo se configuran con `~spi_bus` (0),
`~spi_device` (0), `~spi_speed_hz` (500000) y `~spi_mode` (0). Se requiere el
módulo Python `spidev` en el equipo que ejecuta ROS.

Las tramas se conservan en una cola FIFO y se reintentan si la llamada SPI
devuelve un error local. El protocolo no usa confirmación del Arduino: un
envío que termina sin error no demuestra que el microcontrolador haya recibido
o procesado los datos, por lo que no se pueden detectar pérdidas silenciosas.

El sketch `arduino/SPI_Servo_Slave/SPI_Servo_Slave.ino` está preparado para
Arduino Uno/Nano AVR. Usa SPI esclavo en modo 0, SS en D10 y los servos en D9,
D6, D5, D3 y D2 para `q1` a `q5`. Los niveles de `q1`, `q2` y `q4` se convierten
a posiciones de servo `0..174` grados; `q3` usa `0..84` y `q5` usa `0`, `45` o
`90`. El movimiento se suaviza con un factor `0.06` cada 15 ms, sin bloquear
la recepción SPI. El Arduino arranca en posición neutra, con la pinza abierta. Los bytes
recibidos se comprueban antes de actualizar los servos; los contadores de
tramas válidas, inválidas, incompletas y desbordamientos, además de los objetivos
actuales, se muestran cada segundo por Serial a 115200 baudios. Al arrancar se
imprime un mensaje de inicio.

Con una Raspberry Pi como maestro, conecta MOSI, SCLK y CE0/SS a D11, D13 y
D10 del Arduino, y comparte GND. D12/MISO del Uno entrega 5 V: usa un adaptador
de nivel lógico o divisor antes de conectarlo al GPIO de 3,3 V de la Pi. Alimenta
los servos con una fuente externa adecuada y conecta su GND al GND común. Ajusta
los pines del sketch si el cableado real es distinto.

## ⚙️ Configuración y parámetros

El nodo principal (`Stream_node.py`) es altamente configurable mediante parámetros de ROS, permitiendo activar o desactivar módulos de MediaPipe según la necesidad (para ahorrar recursos computacionales):

- `~enable_right_hand` / `~enable_left_hand`
- `~enable_pose`
- `~enable_face` / `~enable_face_detection`
- `~enable_objectron`

Para publicar la pose corporal del brazo, activa `~enable_pose:=true` (ya es el
valor por defecto). `~enable_right_hand` también es `false` por defecto; si se
activa, `Stream_node.py` publica el mensaje de la mano derecha, pero los nodos
de cinemática actuales no lo consumen. En consecuencia, esta opción sirve para
futuras pruebas de detección, no para corregir por sí sola el cálculo actual de
la pinza.

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

## 🏃 Modo de uso

Este repositorio contiene los scripts, pero no incluye los metadatos de un
paquete catkin ni las definiciones de los mensajes ROS
`mediapipe_gesture_recognition/Pose`, `Keypoint`, `Hand` y `Face`. Para
ejecutar este lanzamiento, instala los nodos Python y `launch_sequence.py` en
la carpeta `scripts` del paquete ROS
`mediapipe_gesture_recognition` (o ajusta el argumento `ros_package` si usas
otro nombre), junto con sus dependencias y permisos de ejecución. Copia
`launch/rgbd_robot.launch` a la carpeta `launch` de ese paquete. El paquete
debe proporcionar los mensajes indicados y estar compilado en tu espacio
catkin. Asegúrate de que `launch_sequence.py` y los nodos instalados tengan
permiso de ejecución.

Antes de iniciar la secuencia, arranca el driver de la cámara por separado y
comprueba que publica `/camera/rgb/image_raw/compressed`,
`/camera/depth_registered/image_raw` y `/camera/rgb/camera_info`. Este
repositorio no incluye el nodo/driver de la cámara.

Lanza los nodos del proyecto con:

```bash
roslaunch mediapipe_gesture_recognition rgbd_robot.launch
```

El launch inicia Stream, Transform y el nodo de cinemática seleccionado, sin
esperar mensajes entre esos arranques. Después supervisa las confirmaciones:

1. `Stream_node.py` publica `/mediapipe_gesture_recognition/stream_ready` al
   recibir y convertir correctamente el primer fotograma RGB-D.
2. Se espera un mensaje en `/mediapipe_gesture_recognition/pose`, que confirma
   que MediaPipe ha detectado una pose corporal, y después el primer mensaje
   válido en `/brazo`. Para publicarlo, hombro, codo y muñeca derechos deben
   tener coordenadas 3D válidas obtenidas de la profundidad y `CameraInfo`.
3. Se inicia `SPI_node.py` inmediatamente después de la cinemática, sin esperar
   todavía a que aparezca el primer vector de ángulos.
4. Se espera el primer vector de ángulos en `/robot/joint_angles` y
   `/robot/spi_ready`, que confirma que el bus SPI se abrió y el nodo está
   listo para transmitir.

Por defecto se usa la cinemática desacoplada. Para elegir la completa:

```bash
roslaunch mediapipe_gesture_recognition rgbd_robot.launch kinematics:=completa
```

Cada confirmación tiene un límite de espera de 180 segundos. Se puede ajustar,
por ejemplo, a 300 segundos con `ready_timeout:=300`. Si una etapa no publica
su confirmación a tiempo, el supervisor informa el error y detiene los nodos
que ya había iniciado. La secuencia presupone que no hay otras instancias del
flujo publicando en esos tópicos.

El launch no ejecuta el firmware Arduino: carga
`arduino/SPI_Servo_Slave/SPI_Servo_Slave.ino` en el microcontrolador por
separado. `/robot/spi_ready` confirma que el transmisor ROS abrió el bus, no
que el Arduino haya recibido la trama; el protocolo SPI no dispone de
confirmación de recepción.

Para ejecutar los nodos manualmente, inicia `Stream_node.py`,
`Transf_coord_node.py`, una sola alternativa de cinemática y `SPI_node.py`, en
ese orden, comprobando entre etapas `/mediapipe_gesture_recognition/pose`,
`/brazo` y `/robot/joint_angles`.

Antes de accionar el brazo, comprueba cada etapa por separado: imágenes y
`CameraInfo`, landmarks 3D válidos, pose relativa, cinco valores en
`/robot/joint_angles` y, finalmente, tramas aceptadas por Arduino. Empieza con
los servomotores desacoplados de la carga mecánica y verifica alimentación,
sentido de giro, topes y niveles lógicos. La transmisión SPI no confirma la
recepción al nodo ROS; usa los contadores serie del Arduino para observar las
tramas aceptadas, inválidas o incompletas.

Para inspeccionar visualmente la detección sin usarla durante el control:

```bash
rosrun <nombre_del_paquete> Stream_node.py _publish_debug_image:=true
rqt_image_view /mediapipe_gesture_recognition/debug_image
```

---
*La integración y el comportamiento físico deben validarse en el montaje final
con la cámara, el equipo maestro SPI, el Arduino y los servomotores utilizados.*
