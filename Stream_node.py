#!/usr/bin/env python3

"""
-------------------------------------------------------------------------------
SISTEMA DE VISIÓN ARTIFICIAL RGB-D Y CONTROL ROBÓTICO
-------------------------------------------------------------------------------
Descripción General:
Este script implementa un nodo de ROS que procesa imágenes RGB y de profundidad 
(Depth) provenientes de una cámara Kinect. Utiliza la librería MediaPipe para 
detectar y extraer puntos clave (keypoints) anatómicos, como la postura corporal, 
manos y rostro. A cada punto detectado en 2D se le asigna su profundidad real en 
metros obtenida del sensor de profundidad de la cámara.

El código está estructurado en secciones para facilitar su futura expansión con 
las siguientes etapas:

1. IMPORTACIÓN DE LIBRERÍAS: Importa las librerías necesarias para el funcionamiento del script.
2. CAPTURA Y PROCESAMIENTO (MediaPipe Streaming): Sincroniza imágenes, ejecuta 
   MediaPipe y extrae coordenadas (X, Y, Z) de los puntos clave.
3. TRANSFORMACIÓN DE COORDENADAS (Próximamente): Convertirá las coordenadas de 
   la cámara al sistema de referencia del robot.
4. CINEMÁTICA INVERSA (Próximamente): Calculará los ángulos articulares para 
   el movimiento del robot.
5. ENVÍO DE DATOS POR SPI (Próximamente): Transmitirá los comandos de control 
   a un microcontrolador externo.
6. MAIN: Función principal que ejecuta el programa.
-------------------------------------------------------------------------------
"""

# ==============================================================================
# 1. IMPORTACIÓN DE LIBRERÍAS
# ==============================================================================
import message_filters # type: ignore
from cv_bridge import CvBridge # type: ignore
import rospy, cv2  # type: ignore
import numpy as np # type: ignore
from termcolor import colored # type: ignore
from typing import Optional
from sensor_msgs.msg import CompressedImage, Image # type: ignore

# Importar módulos de MediaPipe
from mediapipe.python.solutions import drawing_utils, drawing_styles, holistic, face_detection, objectron # type: ignore
from mediapipe_gesture_recognition.msg import Pose, Face, Keypoint, Hand # type: ignore

# Importar clases de datos de MediaPipe
from utils.mediapipe_types import NormalizedLandmark, HolisticResults, DetectionResult, ObjectronResults # type: ignore

# ==============================================================================
# 2. CAPTURA Y PROCESAMIENTO (MEDIAPIPE STREAMING NODE)
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
        self.objectron_model       = rospy.get_param('~objectron_model', 'Shoe')

        # Leer parámetros de entrada (Kinect a través de ROS)
        self.input_topic = rospy.get_param('~input_topic', '/camera/rgb/image_raw/compressed')

        # Mostrar por pantalla las funciones activadas
        print(colored(f'\nFunciones activadas:\n', 'yellow'))
        print(colored(f'  Mano derecha: {self.enable_right_hand}',  'green' if self.enable_right_hand else 'red'))
        print(colored(f'  Mano izquierda: {self.enable_left_hand}\n', 'green' if self.enable_left_hand  else 'red'))
        print(colored(f'  Esqueleto (Mediapipe pose):   {self.enable_pose}',        'green' if self.enable_pose else 'red'))
        print(colored(f'  Malla facial (Mediapipe face):  {self.enable_face}\n',      'green' if self.enable_face else 'red'))
        print(colored(f'  Objectron:       {self.enable_objectron}',        'green' if self.enable_objectron      else 'red'))
        print(colored(f'  Detección facial:  {self.enable_face_detection}\n', 'green' if self.enable_face_detection else 'red'))

        # Comprobar si el modelo de Objectron es válido (aunque estará desactivado)
        if not self.objectron_model in self.available_objectron_models:
            rospy.logerr('ERROR: Modelo de Objectron no disponible. Cerrando nodo...')
            rospy.signal_shutdown('ERROR: Modelo de Objectron no disponible')

        # Publicadores de ROS para enviar los resultados de MediaPipe
        self.hand_right_pub = rospy.Publisher('/mediapipe_gesture_recognition/right_hand', Hand, queue_size=1)
        self.hand_left_pub  = rospy.Publisher('/mediapipe_gesture_recognition/left_hand', Hand, queue_size=1)
        self.pose_pub       = rospy.Publisher('/mediapipe_gesture_recognition/pose', Pose, queue_size=1)
        self.face_pub       = rospy.Publisher('/mediapipe_gesture_recognition/face', Face, queue_size=1)

        # Configurar la entrada de video usando los tópicos de Kinect
        rospy.loginfo(f"Sincronizando imágenes de color y profundidad desde el tópico: {self.input_topic}...")
        
        # Suscriptores a los tópicos de la Kinect
        self.image_sub = message_filters.Subscriber(self.input_topic, CompressedImage)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        
        # Sincronizador de tiempo aproximado (tolera un pequeño desfase temporal)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback_ros)
            
        # Inicializar los modelos de MediaPipe
        self.initMediaPipeSolutions()

    def callback_ros(self, rgb_msg, depth_msg):
        try:
            rospy.loginfo_once("¡Primer fotograma recibido correctamente!")
            
            # Extraer la imagen de color
            image_color = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
            
            # Extraer la imagen de profundidad
            self.current_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            # MediaPipe requiere las imágenes en formato RGB en lugar de BGR
            image_rgb = cv2.cvtColor(image_color, cv2.COLOR_BGR2RGB)
            self.processResults(image_rgb)
            
        except Exception as e:
            rospy.logwarn(f"Fotograma corrupto saltado. Error de CvBridge: {e}")

    def initMediaPipeSolutions(self):
        """ Inicializa los módulos de MediaPipe con sus parámetros configurables """

        self.mp_drawing, self.mp_drawing_styles = drawing_utils, drawing_styles
        self.mp_holistic, self.mp_face_detection, self.mp_objectron = holistic, face_detection, objectron

        # Parámetros del modelo Holistic
        static_image_mode        = rospy.get_param('/holistic/static_image_mode', False)
        model_complexity         = rospy.get_param('/holistic/model_complexity', 1)
        smooth_landmarks         = rospy.get_param('/holistic/smooth_landmarks', True)
        enable_segmentation      = rospy.get_param('/holistic/enable_segmentation', False)
        smooth_segmentation      = rospy.get_param('/holistic/smooth_segmentation', True)
        refine_face_landmarks    = rospy.get_param('/holistic/refine_face_landmarks', True)
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
            height, width = self.current_depth_image.shape
            
            # Convertir coordenadas normalizadas (0-1) a píxeles
            px = int(landmark.x * width)
            py = int(landmark.y * height)
            
            # Limitar dentro de los márgenes de la imagen
            px = max(0, min(px, width - 1))
            py = max(0, min(py, height - 1))
            
            # --- CORRECCIÓN DE RUIDO DE BORDE (PARALLAX) ---
            # En lugar de leer un solo píxel (que en el codo suele fallar y dar la profundidad de la pared de fondo),
            # extraemos una ventana de 5x5 alrededor del punto y tomamos la mediana de los valores válidos.
            min_y, max_y = max(0, py - 2), min(height, py + 3)
            min_x, max_x = max(0, px - 2), min(width, px + 3)
            window = self.current_depth_image[min_y:max_y, min_x:max_x]
            
            valid_depths = window[window > 0]
            if len(valid_depths) > 0:
                # Usamos el percentil 30 o la mediana para agarrarnos al brazo (más cercano) y evitar el fondo
                depth_val = np.percentile(valid_depths, 30)
            else:
                depth_val = 0
            
            # Convertir milímetros a metros y aplicar modelo Pinhole
            if depth_val > 0:
                z_m = float(depth_val) / 1000.0
                new_keypoint.depth = z_m
                
                # Parámetros ópticos aproximados de Kinect (para 640x480)
                # TODO: En el futuro, suscribirse a /camera/rgb/camera_info
                fx, fy = 525.0, 525.0
                cx, cy = 320.0, 240.0
                
                # Ecuación estenopeica (Pinhole camera model) para obtener X, Y en metros
                u = landmark.x * width
                v = landmark.y * height
                new_keypoint.x = (u - cx) * z_m / fx
                new_keypoint.y = (v - cy) * z_m / fy
                new_keypoint.z = z_m

        # Asignar número y nombre al keypoint
        new_keypoint.keypoint_number = number
        new_keypoint.keypoint_name = name

        return new_keypoint

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
        hand_msg.header.stamp = rospy.Time.now()
        hand_msg.header.frame_id = 'Hand Right Message' if RightLeft else 'Hand Left Message'
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
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'Pose Message'

        if poseResults.pose_landmarks:
            for i in range(len(poseResults.pose_landmarks.landmark)):
                pose_msg.keypoints.append(self.newKeypoint(poseResults.pose_landmarks.landmark[i], i+1, self.pose_landmarks_names[i]))

            self.pose_pub.publish(pose_msg)

    def processFace(self, faceResults:HolisticResults, image:cv2.typing.MatLike) -> Optional[Face]:

        """ Procesa y publica los keypoints de la malla facial """

        self.mp_drawing.draw_landmarks(
            image,
            faceResults.face_landmarks,
            connections=self.face_mesh_connections,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.face_mesh_connection_drawing_spec)

        face_msg = Face()
        face_msg.header.stamp = rospy.Time.now()
        face_msg.header.frame_id = 'Face Message'

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
# 3. TRANSFORMACIÓN DE COORDENADAS (Próximamente)
# ==============================================================================
# TODO: Implementar la transformación de las coordenadas espaciales de la cámara
# al sistema de coordenadas base del robot.


# ==============================================================================
# 4. CINEMÁTICA INVERSA (Próximamente)
# ==============================================================================
# TODO: Implementar el cálculo de los ángulos articulares necesarios para que 
# el robot alcance las posiciones deseadas.


# ==============================================================================
# 5. ENVÍO DE DATOS POR SPI (Próximamente)
# ==============================================================================
# TODO: Implementar la comunicación por bus SPI para enviar las variables de 
# control al microcontrolador esclavo.


# ==============================================================================
# 6. MAIN
# ==============================================================================
if __name__ == '__main__':

    # Inicializar el nodo principal
    MediapipeStream = MediapipeStreaming()
    
    # En modo ROS, el bucle y la frecuencia se controlan mediante las callbacks
    rospy.loginfo("MediaPipe funcionando mediante tópicos de Kinect. Esperando imágenes...")
    rospy.spin()
