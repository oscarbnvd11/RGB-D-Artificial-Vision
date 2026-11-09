#!/usr/bin/env python3

"""
===============================================================================
SISTEMA DE VISIÓN ARTIFICIAL RGB-D Y CONTROL ROBÓTICO
===============================================================================


NODO DE PERCEPCION RGB-D DEL PROYECTO
 
Descripción General:

Este nodo recibe las imagenes RGB y profundidad sincronizadas de la camara,
ejecuta MediaPipe y publica los puntos de la postura en `/mediapipe_gesture_recognition/pose`. 
Los puntos incluyen coordenadas 3D en metros calculadas con la profundidad 
registrada y la calibracion de la camara.

Flujo general del proyecto:
1. `Openni_node.py` publica las imagenes RGB y profundidad de la camara en
   `/camera/rgb/image_raw` y `/camera/depth/image_raw`.
2. `Stream_node.py` detecta la postura y publica la pose.
3. `Transf_coord_node.py` toma el hombro derecho como origen y transforma los
    ejes de la camara al sistema del robot, publicando `/brazo`.
4. Se ejecuta una sola opcion de cinematica: `CI_desacoplada_node.py` o
    `CI_completa_node.py`. Ambas publican `[q1, q2, q3, q4, q5]` en
    `/robot/joint_angles`.
5. `SPI_node.py`. Envía los angulos de articulacion al robot por SPI. 
6. Arduino. Ejecuta los movimientos de los motores.

"""

# ==============================================================================
#  IMPORTACIÓN DE LIBRERÍAS
# ==============================================================================
import message_filters # type: ignore
from cv_bridge import CvBridge # type: ignore
import rospy, cv2  # type: ignore
import numpy as np # type: ignore
import traceback
from collections import deque
from termcolor import colored # type: ignore
from typing import Optional
from sensor_msgs.msg import CameraInfo, CompressedImage, Image # type: ignore
from std_msgs.msg import Empty  # type: ignore

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Importar módulos de MediaPipe
from mediapipe.python.solutions import drawing_utils, drawing_styles, holistic, face_detection, objectron # type: ignore
from mediapipe_gesture_recognition.msg import Pose, Face, Keypoint, Hand # type: ignore

# Importar clases de datos de MediaPipe
from utils.mediapipe_types import NormalizedLandmark, HolisticResults, DetectionResult, ObjectronResults # type: ignore

# ==============================================================================
#  CAPTURA Y PROCESAMIENTO (MEDIAPIPE STREAMING NODE)
# ==============================================================================
class MediapipeStreaming:

    """ 
    Clase para transmitir datos de MediaPipe.
    Procesa imágenes de cámara desde ROS (Kinect) y publica los puntos clave detectados en tópicos de ROS.
    
    MediaPipe Holistic devuelve:
    Una tupla con campos que describen los puntos clave de la persona más prominente detectada:
        1) "pose_landmarks": Puntos clave de la postura corporal.
        2) "pose_world_landmarks": Puntos clave en coordenadas 3D del mundo real (en metros), con el origen en el centro entre las caderas.
        3) "left_hand_landmarks": Puntos clave de la mano izquierda.
        4) "right_hand_landmarks": Puntos clave de la mano derecha.
        5) "face_landmarks": Puntos clave del rostro.
        6) "segmentation_mask": Máscara de segmentación (si "enable_segmentation" está activado).
    """

    # Constantes
    RIGHT_HAND, LEFT_HAND = True, False
    RIGHT_WRIST = 17
    Z_DIAGNOSTICS_WINDOW = 50

    # Nombres de los puntos clave de la mano (Mediapipe hand)
    hand_landmarks_names = ['WRIST', 'THUMB_CMC', 'THUMB_MCP', 'THUMB_IP', 'THUMB_TIP', 'INDEX_FINGER_MCP',
                            'INDEX_FINGER_PIP', 'INDEX_FINGER_DIP', 'INDEX_FINGER_TIP', 'MIDDLE_FINGER_MCP',
                            'MIDDLE_FINGER_PIP', 'MIDDLE_FINGER_DIP', 'MIDDLE_FINGER_TIP', 'RING_FINGER_MCP',
                            'RING_FINGER_PIP', 'RING_FINGER_DIP', 'RING_FINGER_TIP', 'PINKY_MCP',
                            'PINKY_PIP', 'PINKY_DIP', 'PINKY_TIP']

    # Nombres de los puntos clave del cuerpo (Mediapipe pose)
    pose_landmarks_names = ['NOSE', 'LEFT_EYE_INNER', 'LEFT_EYE', 'LEFT_EYE_OUTER', 'RIGHT_EYE_INNER',
                            'RIGHT_EYE', 'RIGHT_EYE_OUTER', 'LEFT_EAR', 'RIGHT_EAR', 'MOUTH_LEFT', 'MOUTH_RIGHT',
                            'LEFT_SHOULDER', 'RIGHT_SHOULDER', 'LEFT_ELBOW', 'RIGHT_ELBOW', 'LEFT_WRIST',
                            'RIGHT_WRIST', 'LEFT_PINKY', 'RIGHT_PINKY', 'LEFT_INDEX', 'RIGHT_INDEX', 'LEFT_THUMB',
                            'RIGHT_THUMB', 'LEFT_HIP', 'RIGHT_HIP', 'LEFT_KNEE', 'RIGHT_KNEE', 'LEFT_ANKLE',
                            'RIGHT_ANKLE', 'LEFT_HEEL', 'RIGHT_HEEL', 'LEFT_FOOT_INDEX', 'RIGHT_FOOT_INDEX']

    # Modelos disponibles para Objectron (detección de objetos en 3D, aunque estará desactivado)
    available_objectron_models = ['Shoe', 'Chair', 'Cup', 'Camera']

    def __init__(self):

        # Inicialización de ROS
        rospy.init_node('mediapipe_stream_node', anonymous=True)
        self.ros_rate = rospy.Rate(30)
        self.bridge = CvBridge()
        
        # Leer parámetros para activar los distintos módulos de MediaPipe
        self.enable_right_hand     = rospy.get_param('~enable_right_hand', False)
        self.enable_left_hand      = rospy.get_param('~enable_left_hand', False)
        self.enable_pose           = rospy.get_param('~enable_pose', True)
        self.enable_face           = rospy.get_param('~enable_face', False)
        self.enable_face_detection = rospy.get_param('~enable_face_detection', False)
        self.enable_objectron      = rospy.get_param('~enable_objectron', False)
        self.medir_delay           = rospy.get_param('~medir_delay', False)
        self.parallax_filter_enabled = rospy.get_param(
            '~parallax_filter_enabled', True)
        self.z_diagnostics_enabled = rospy.get_param(
            '~z_diagnostics_enabled', True)
        self.raw_wrist_z_window = deque(maxlen=self.Z_DIAGNOSTICS_WINDOW)
        self.filtered_wrist_z_window = deque(maxlen=self.Z_DIAGNOSTICS_WINDOW)
        self.objectron_model       = rospy.get_param('~objectron_model', 'Shoe')
        self.publish_debug_image   = rospy.get_param('~publish_debug_image', False)
        self.debug_every_n_frames  = max(1, rospy.get_param('~debug_every_n_frames', 5))
        self.debug_scale            = min(1.0, max(0.1, rospy.get_param('~debug_scale', 0.5)))
        self.debug_frame_count      = 0

        # Leer parámetros de entrada (Kinect a través de ROS)
        self.input_topic = rospy.get_param('~input_topic', '/camera/rgb/image_raw/compressed')
        # Este nodo usa las coordenadas RGB de MediaPipe para consultar depth;
        # por eso el mapa debe estar registrado/alineado con la cámara RGB.
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth_registered/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')
        self.depth_scale = rospy.get_param('~depth_scale', 0.001)
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_rgb_optical_frame')
        self.camera_info = None
        self.current_color_shape = None
        self.current_stamp = rospy.Time.now()
        self.stream_ready = False

        # Mostrar por pantalla las funciones activadas
        print(colored(f'\nFunciones activadas:\n', 'yellow'))
        print(colored(f'  Mano derecha: {self.enable_right_hand}',  'green' if self.enable_right_hand else 'red'))
        print(colored(f'  Mano izquierda: {self.enable_left_hand}\n', 'green' if self.enable_left_hand  else 'red'))
        print(colored(f'  Esqueleto (Mediapipe pose):   {self.enable_pose}',        'green' if self.enable_pose else 'red'))
        print(colored(f'  Malla facial (Mediapipe face):  {self.enable_face}\n',      'green' if self.enable_face else 'red'))
        print(colored(f'  Objectron:       {self.enable_objectron}',        'green' if self.enable_objectron      else 'red'))
        print(colored(f'  Detección facial:  {self.enable_face_detection}\n', 'green' if self.enable_face_detection else 'red'))
        rospy.loginfo(
            'Filtro espacial de paralaje: %s; diagnostico Z de muñeca: %s '
            '(ventana de %s frames)',
            'activado' if self.parallax_filter_enabled else 'desactivado',
            'activado' if self.z_diagnostics_enabled else 'desactivado',
            self.Z_DIAGNOSTICS_WINDOW)

        # Comprobar si el modelo de Objectron es válido (aunque estará desactivado)
        if not self.objectron_model in self.available_objectron_models:
            rospy.logerr('ERROR: Modelo de Objectron no disponible. Cerrando nodo...')
            rospy.signal_shutdown('ERROR: Modelo de Objectron no disponible')

        # Publicadores de ROS para enviar los resultados de MediaPipe
        self.hand_right_pub = rospy.Publisher('/mediapipe_gesture_recognition/right_hand', Hand, queue_size=1)
        self.hand_left_pub  = rospy.Publisher('/mediapipe_gesture_recognition/left_hand', Hand, queue_size=1)
        self.pose_pub       = rospy.Publisher('/mediapipe_gesture_recognition/pose', Pose, queue_size=1)
        self.face_pub       = rospy.Publisher('/mediapipe_gesture_recognition/face', Face, queue_size=1)
        self.stream_ready_pub = rospy.Publisher(
            '/mediapipe_gesture_recognition/stream_ready',
            Empty,
            queue_size=1,
            latch=True)
        if self.publish_debug_image:
            self.debug_image_pub = rospy.Publisher('/mediapipe_gesture_recognition/debug_image', Image, queue_size=1)

        # Configurar la entrada de video usando los tópicos de Kinect
        rospy.loginfo(f"Sincronizando RGB ({self.input_topic}) y depth ({self.depth_topic})...")
        published_topics = dict(rospy.get_published_topics())
        if self.input_topic not in published_topics:
            rospy.logwarn(f"No se está publicando el tópico RGB configurado: {self.input_topic}")
        if self.depth_topic not in published_topics:
            rospy.logwarn(f"No se está publicando el tópico depth configurado: {self.depth_topic}")
        
        # Suscriptores a los tópicos de la Kinect
        self.image_sub = message_filters.Subscriber(self.input_topic, CompressedImage)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # Sincronizador de tiempo aproximado (tolera un pequeño desfase temporal)
        sync_queue_size = rospy.get_param('~sync_queue_size', 2)
        sync_slop = rospy.get_param('~sync_slop', 0.05)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], queue_size=sync_queue_size, slop=sync_slop)
        self.ts.registerCallback(self.callback_ros)
            
        # Inicializar los modelos de MediaPipe
        self.initMediaPipeSolutions()

    def camera_info_callback(self, msg):
        """Guarda la calibración de la cámara RGB publicada por ROS."""
        if len(msg.K) == 9 and msg.K[0] > 0 and msg.K[4] > 0:
            self.camera_info = msg

    def callback_ros(self, rgb_msg, depth_msg):
        try:
            # Extraer la imagen de color
            image_color = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
            
            # Extraer la imagen de profundidad
            self.current_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            self.current_depth_encoding = depth_msg.encoding
            if self.current_depth_image.ndim != 2:
                raise ValueError('La imagen de profundidad no es monocanal')
            self.current_color_shape = image_color.shape[:2]
            self.current_stamp = rgb_msg.header.stamp
            if not rgb_msg.header.stamp:
                self.current_stamp = rospy.Time.now()
            
            # MediaPipe requiere las imágenes en formato RGB en lugar de BGR
            image_rgb = cv2.cvtColor(image_color, cv2.COLOR_BGR2RGB)
            rospy.loginfo_once("¡Primer fotograma RGB-D recibido correctamente!")
            if not self.stream_ready:
                self.stream_ready_pub.publish(Empty())
                self.stream_ready = True
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Error convirtiendo el fotograma RGB-D: {e}")
            return

        try:
            self.processResults(image_rgb)
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Error ejecutando MediaPipe/publicando resultados: {e}")
            rospy.logerr_throttle(5.0, traceback.format_exc())

        if self.publish_debug_image:
            try:
                self.debug_frame_count += 1
                if self.debug_frame_count % self.debug_every_n_frames != 0:
                    return

                # El debug se reduce y se publica con menor frecuencia para no
                # bloquear el procesamiento de los frames de control.
                debug_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
                if self.debug_scale < 1.0:
                    debug_image = cv2.resize(
                        debug_image, None, fx=self.debug_scale, fy=self.debug_scale,
                        interpolation=cv2.INTER_AREA)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = rgb_msg.header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                rospy.logerr_throttle(5.0, f"Error publicando imagen debug: {e}")

    def initMediaPipeSolutions(self):
        """ Inicializa los módulos de MediaPipe con sus parámetros configurables """

        self.mp_drawing, self.mp_drawing_styles = drawing_utils, drawing_styles
        self.mp_holistic, self.mp_face_detection, self.mp_objectron = holistic, face_detection, objectron

        # Parámetros del modelo Holistic
        static_image_mode        = rospy.get_param('/holistic/static_image_mode', False)
        model_complexity         = rospy.get_param('~model_complexity', 0)
        smooth_landmarks         = rospy.get_param('/holistic/smooth_landmarks', True)
        enable_segmentation      = rospy.get_param('/holistic/enable_segmentation', False)
        smooth_segmentation      = rospy.get_param('/holistic/smooth_segmentation', True)
        refine_face_landmarks    = rospy.get_param('~refine_face_landmarks', False)
        min_detection_confidence = rospy.get_param('/holistic/min_detection_confidence', 0.5)
        min_tracking_confidence  = rospy.get_param('/holistic/min_tracking_confidence', 0.5)

        # Parámetros para FaceMesh (0: Contornos, 1: Teselado)
        face_mesh_mode = rospy.get_param('mediapipe_gesture_recognition/face_mesh_mode', 0)
        self.face_mesh_connections = self.mp_holistic.FACEMESH_TESSELATION if face_mesh_mode == 1 else self.mp_holistic.FACEMESH_CONTOURS
        self.face_mesh_connection_drawing_spec = self.mp_drawing_styles.get_default_face_mesh_tesselation_style() if face_mesh_mode == 1 else self.mp_drawing_styles.get_default_face_mesh_contours_style()

        if self.enable_right_hand or self.enable_left_hand or self.enable_pose or self.enable_face:
            self.holistic = self.mp_holistic.Holistic(static_image_mode, model_complexity, smooth_landmarks, enable_segmentation, smooth_segmentation,
                                                      refine_face_landmarks, min_detection_confidence, min_tracking_confidence)

        # Parámetros de detección facial
        min_detection_confidence = rospy.get_param('/face_detection/min_detection_confidence', 0.5)
        model_selection          = rospy.get_param('/face_detection/model_selection', 0)

        if self.enable_face_detection:
            self.face_detection = self.mp_face_detection.FaceDetection(min_detection_confidence, model_selection)

        # Parámetros de Objectron
        static_image_mode        = rospy.get_param('/objectron/static_image_mode', False)
        max_num_objects          = rospy.get_param('/objectron/max_num_objects', 5)
        min_detection_confidence = rospy.get_param('/objectron/min_detection_confidence', 0.5)
        min_tracking_confidence  = rospy.get_param('/objectron/min_tracking_confidence', 0.99)
        focal_length             = rospy.get_param('/objectron/focal_length', [1,1])
        principal_point          = rospy.get_param('/objectron/principal_point', [0,0])
        image_size               = rospy.get_param('/objectron/image_size', None)

        if self.enable_objectron and self.objectron_model in ['Shoe', 'Chair', 'Cup', 'Camera']:
            self.objectron = self.mp_objectron.Objectron(static_image_mode, max_num_objects, min_detection_confidence, min_tracking_confidence,
                                                         self.objectron_model, focal_length, principal_point, None if image_size=='None' else image_size)

    def newKeypoint(self, landmark:NormalizedLandmark, number:int, name:str) -> Keypoint:
        """ Crea y asigna valores a un nuevo keypoint, calculando sus coordenadas 3D en metros desde la Kinect """

        new_keypoint = Keypoint()
        new_keypoint.v = landmark.visibility

        # Por defecto guardamos los valores crudos por si no hay profundidad válida
        new_keypoint.x = landmark.x
        new_keypoint.y = landmark.y
        new_keypoint.z = landmark.z
        new_keypoint.depth = 0.0

        # --- EXTRACCIÓN DE PROFUNDIDAD Y CONVERSIÓN A METROS ---
        if hasattr(self, 'current_depth_image'):
            # Se saca la resolución de la imagen (suele ser 480x640)
            depth_height, depth_width = self.current_depth_image.shape
            color_height, color_width = self.current_color_shape
            
            # Convertir coordenadas normalizadas (0-1) a píxeles
            px = int(landmark.x * depth_width)
            py = int(landmark.y * depth_height)
            
            # Limitar dentro de los márgenes de la imagen
            px = max(0, min(px, depth_width - 1))
            py = max(0, min(py, depth_height - 1))
            
            # La medición cruda es el píxel central; el filtro usa la ventana 5x5.
            min_y, max_y = max(0, py - 2), min(depth_height, py + 3)
            min_x, max_x = max(0, px - 2), min(depth_width, px + 3)
            window = self.current_depth_image[min_y:max_y, min_x:max_x]

            raw_depth = self.current_depth_image[py, px]
            valid_depths = window[np.isfinite(window) & (window > 0)]
            if len(valid_depths) > 0:
                filtered_depth = np.percentile(valid_depths, 30)
            else:
                filtered_depth = 0

            if self.parallax_filter_enabled:
                depth_val = filtered_depth
            else:
                depth_val = raw_depth

            # Convertir la profundidad a metros según el encoding de ROS.
            if depth_val > 0:
                depth_encoding = getattr(self, 'current_depth_encoding', '')
                scale = 1.0 if depth_encoding == '32FC1' else self.depth_scale
                z_m = float(depth_val) * scale
                new_keypoint.depth = z_m

                if (self.z_diagnostics_enabled
                        and number == self.RIGHT_WRIST
                        and name == 'RIGHT_WRIST'
                        and np.isfinite(raw_depth)
                        and raw_depth > 0
                        and filtered_depth > 0):
                    raw_z_m = float(raw_depth) * scale
                    self._record_wrist_z(raw_z_m, z_m)
                
                if self.camera_info is None:
                    rospy.logwarn_throttle(5.0, 'Esperando CameraInfo; no se publican coordenadas 3D.')
                    new_keypoint.depth = 0.0
                    return new_keypoint

                info = self.camera_info
                scale_x = color_width / float(info.width) if info.width else 1.0
                scale_y = color_height / float(info.height) if info.height else 1.0
                fx, fy = info.K[0] * scale_x, info.K[4] * scale_y
                cx, cy = info.K[2] * scale_x, info.K[5] * scale_y
                
                # Ecuación estenopeica (Pinhole camera model) para obtener X, Y en metros
                u = landmark.x * color_width
                v = landmark.y * color_height
                new_keypoint.x = (u - cx) * z_m / fx
                new_keypoint.y = (v - cy) * z_m / fy
                new_keypoint.z = z_m

        # Asignar número y nombre al keypoint
        new_keypoint.keypoint_number = number
        new_keypoint.keypoint_name = name

        return new_keypoint

    def _record_wrist_z(self, raw_z_m: float, filtered_z_m: float) -> None:
        """Guarda y muestra la Z cruda y filtrada de la muñeca derecha."""
        self.raw_wrist_z_window.append(raw_z_m)
        self.filtered_wrist_z_window.append(filtered_z_m)
        rospy.loginfo_throttle(
            1.0,
            'Z muñeca [mm]: antes=%.1f despues=%.1f',
            raw_z_m * 1000.0,
            filtered_z_m * 1000.0)
        if len(self.raw_wrist_z_window) < self.Z_DIAGNOSTICS_WINDOW:
            return

        raw_window_mm = np.asarray(self.raw_wrist_z_window) * 1000.0
        filtered_window_mm = np.asarray(self.filtered_wrist_z_window) * 1000.0
        raw_variance = float(np.var(raw_window_mm))
        filtered_variance = float(np.var(filtered_window_mm))
        variance_reduction = (
            0.0 if raw_variance == 0.0
            else (raw_variance - filtered_variance) / raw_variance * 100.0)
        raw_range_mm = float(np.max(raw_window_mm) - np.min(raw_window_mm))
        rospy.loginfo_throttle(
            1.0,
            'Estadisticas Z [mm/mm^2], ventana=%d: '
            'cruda_min=%.1f cruda_max=%.1f salto=%.1f '
            'var_cruda=%.2f var_salida=%.2f reduccion_var=%.1f%%',
            self.Z_DIAGNOSTICS_WINDOW,
            float(np.min(raw_window_mm)),
            float(np.max(raw_window_mm)),
            raw_range_mm,
            raw_variance,
            filtered_variance,
            variance_reduction)

    def processHand(self, RightLeft:bool, handResults:HolisticResults, image:cv2.typing.MatLike) -> Optional[Hand]:
        """ Procesa y publica los keypoints de las manos """

        # Dibujar keypoints sobre la imagen
        self.mp_drawing.draw_landmarks(
            image,
            handResults.right_hand_landmarks if RightLeft else handResults.left_hand_landmarks,
            self.mp_holistic.HAND_CONNECTIONS,
            self.mp_drawing_styles.get_default_hand_landmarks_style(),
            self.mp_drawing_styles.get_default_hand_connections_style())

        # Construir mensaje de la mano para ROS
        hand_msg = Hand()
        hand_msg.header.stamp = self.current_stamp
        hand_msg.header.frame_id = self.camera_frame
        hand_msg.right_or_left = hand_msg.RIGHT if RightLeft else hand_msg.LEFT

        # Comprobar si hay resultados para la mano indicada
        if (((RightLeft == self.RIGHT_HAND) and (handResults.right_hand_landmarks))
         or ((RightLeft == self.LEFT_HAND)  and (handResults.left_hand_landmarks))):

            landmarks = handResults.right_hand_landmarks.landmark if RightLeft else handResults.left_hand_landmarks.landmark
            for i in range(len(landmarks)):
                hand_msg.keypoints.append(self.newKeypoint(landmarks[i], i+1, self.hand_landmarks_names[i]))

            if RightLeft:
                self.hand_right_pub.publish(hand_msg)
            else:
                self.hand_left_pub.publish(hand_msg)

    def processPose(self, poseResults:HolisticResults, image:cv2.typing.MatLike) -> Optional[Pose]:

        """ Procesa y publica los keypoints de la postura corporal """

        self.mp_drawing.draw_landmarks(
            image,
            poseResults.pose_landmarks,
            self.mp_holistic.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())

        pose_msg = Pose()
        pose_msg.header.stamp = self.current_stamp
        pose_msg.header.frame_id = self.camera_frame

        if poseResults.pose_landmarks:
            for i in range(len(poseResults.pose_landmarks.landmark)):
                pose_msg.keypoints.append(self.newKeypoint(poseResults.pose_landmarks.landmark[i], i+1, self.pose_landmarks_names[i]))

            self.pose_pub.publish(pose_msg)
        else:
            rospy.logwarn_throttle(5.0, 'MediaPipe no ha detectado ninguna pose en el fotograma.')

    def processFace(self, faceResults:HolisticResults, image:cv2.typing.MatLike) -> Optional[Face]:

        """ Procesa y publica los keypoints de la malla facial """

        self.mp_drawing.draw_landmarks(
            image,
            faceResults.face_landmarks,
            connections=self.face_mesh_connections,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.face_mesh_connection_drawing_spec)

        face_msg = Face()
        face_msg.header.stamp = self.current_stamp
        face_msg.header.frame_id = self.camera_frame

        if faceResults.face_landmarks:
            for i in range(len(faceResults.face_landmarks.landmark)):
                face_msg.keypoints.append(self.newKeypoint(faceResults.face_landmarks.landmark[i], i+1, f'FACE_KEYPOINT_{i+1}'))

            self.face_pub.publish(face_msg)

    def processFaceDetection(self, faceDetectionResults:DetectionResult, image:np.ndarray):

        """ Dibuja rectángulos alrededor de las caras detectadas """

        if faceDetectionResults.detections:
            for detection in faceDetectionResults.detections: 
                self.mp_drawing.draw_detection(image, detection)

    def processObjectron(self, objectronResults:ObjectronResults, image:np.ndarray):

        """ Dibuja ejes y cajas 3D de los objetos detectados por Objectron (aunque no se está usando)"""

        if objectronResults.detected_objects:
            for detected_object in objectronResults.detected_objects:
                self.mp_drawing.draw_landmarks(image, detected_object.landmarks_2d, self.mp_objectron.BOX_CONNECTIONS)
                self.mp_drawing.draw_axis(image, detected_object.rotation, detected_object.translation)

    def processResults(self, image:np.ndarray):
        
        """ Ejecuta los módulos activos de MediaPipe sobre la imagen actual """

        # Obtener resultados de MediaPipe
        holistic_results       = self.holistic.process(image)       if self.enable_right_hand or self.enable_left_hand or self.enable_pose or self.enable_face else None
        if (self.medir_delay
                and self.enable_pose
                and holistic_results
                and holistic_results.pose_landmarks):
            self.current_stamp = rospy.Time.now()
        face_detection_results = self.face_detection.process(image) if self.enable_face_detection else None
        objectron_results      = self.objectron.process(image)      if self.enable_objectron else None

        # Procesar resultados anatómicos
        if self.enable_left_hand:  self.processHand(self.LEFT_HAND,  holistic_results, image)
        if self.enable_right_hand: self.processHand(self.RIGHT_HAND, holistic_results, image)
        if self.enable_pose:       self.processPose(holistic_results, image)
        if self.enable_face:       self.processFace(holistic_results, image)

        # Procesar detección facial y de objetos
        if self.enable_face_detection: self.processFaceDetection(face_detection_results, image)
        if self.enable_objectron:      self.processObjectron(objectron_results, image)

# ==============================================================================
#  MAIN
# ==============================================================================
if __name__ == '__main__':

    # Inicializar el nodo principal
    MediapipeStream = MediapipeStreaming()
    
    # En modo ROS, el bucle y la frecuencia se controlan mediante las callbacks
    rospy.loginfo("MediaPipe funcionando mediante tópicos de Kinect. Esperando imágenes...")
    rospy.spin()
