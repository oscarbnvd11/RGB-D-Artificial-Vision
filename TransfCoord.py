#!/usr/bin/env python3

"""
-------------------------------------------------------------------------------
SISTEMA DE VISIÓN ARTIFICIAL RGB-D Y CONTROL ROBÓTICO
-------------------------------------------------------------------------------
Descripción General:
Este script implementa un nodo de ROS que recibe las posturas (poses) publicadas 
por el nodo de MediaPipe. Su función principal es extraer los puntos de interés 
del brazo derecho (hombro, codo, muñeca, dedos), convertirlos a un sistema de 
coordenadas 3D métrico (usando el modelo de cámara estenopeica o pinhole) y 
trasladar el origen de coordenadas al hombro. Esto prepara los datos para la 
siguiente etapa de cinemática inversa.
Estructura:
1. IMPORTACIÓN DE LIBRERÍAS
2. TRANSFORMACIÓN DE COORDENADAS (TransfCoordNode)
3. MAIN
-------------------------------------------------------------------------------
"""
# ==============================================================================
# 1. IMPORTACIÓN DE LIBRERÍAS
# ==============================================================================
import rospy # type: ignore
from mediapipe_gesture_recognition.msg import Pose, Keypoint # type: ignore
# ==============================================================================
# 2. TRANSFORMACIÓN DE COORDENADAS (TransfCoordNode)
# ==============================================================================
class TransfCoordNode:
    def __init__(self):
        # Inicialización de ROS
        rospy.init_node('transf_coord_node', anonymous=True)
        # Publicador para la postura del brazo normalizada
        self.brazo_pub = rospy.Publisher('/brazo', Pose, queue_size=10)
        
        # Suscriptor al tópico de pose de MediaPipe
        rospy.Subscriber('/mediapipe_gesture_recognition/pose', Pose, self.pose_callback)
        # ATENCIÓN: MediaPipe numera de 0 a 32, pero en Stream_node.py sumamos 1, 
        # por lo que en ROS se indexan de 1 a 33.
        # Keypoints del brazo derecho en ROS: 
        # Hombro(13), Codo(15), Muñeca(17), Pinky(19), Index(21), Thumb(23)
        self.target_keypoints = [13, 15, 17, 19, 21, 23]
        # Parámetros ópticos aproximados de Kinect (para resolución 640x480)
        # TODO: Se recomienda suscribirse a /camera/rgb/camera_info para 
        # obtener los valores intrínsecos de forma dinámica en un futuro.
        self.width = 640.0
        self.height = 480.0
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0
    def pose_callback(self, msg):
        """ 
        Callback que se ejecuta al recibir un mensaje de Pose. 
        Extrae los puntos del brazo y centra el origen en el hombro (puntos ya vienen en metros).
        """
        puntos_interes = {}
        # 1. Extraer keypoints objetivo (solo brazo derecho)
        for kp in msg.keypoints:
            if kp.keypoint_number in self.target_keypoints:
                puntos_interes[kp.keypoint_number] = kp

        # 2. Verificar detección del hombro derecho (keypoint 13)
        if 13 not in puntos_interes:
            return
            
        hombro_kp = puntos_interes[13]
        
        # Si el hombro no tiene profundidad válida, descartar el frame
        if hombro_kp.depth <= 0.0:
            return
            
        h_x, h_y, h_z = hombro_kp.x, hombro_kp.y, hombro_kp.z

        # 3. Construir mensaje normalizado en metros relativos al hombro
        brazo_msg = Pose()
        brazo_msg.header.stamp = rospy.Time.now()
        brazo_msg.header.frame_id = 'shoulder_base_link'
        
        for kp_num in self.target_keypoints:
            if kp_num in puntos_interes:
                kp_orig = puntos_interes[kp_num]
                
                # Descartar puntos sin profundidad válida
                if kp_orig.depth <= 0.0:
                    continue

                punto_norm = Keypoint()
                punto_norm.keypoint_number = kp_orig.keypoint_number
                punto_norm.keypoint_name = kp_orig.keypoint_name
                punto_norm.v = kp_orig.v

                # Traslación al origen (Hombro en 0, 0, 0 metros)
                # ¡Ahora podemos restar directamente porque ya vienen en metros desde Stream_node!
                punto_norm.x = kp_orig.x - h_x
                punto_norm.y = kp_orig.y - h_y
                punto_norm.z = kp_orig.z - h_z
                
                # La profundidad relativa es el propio eje Z
                punto_norm.depth = punto_norm.z  
                brazo_msg.keypoints.append(punto_norm)
                
        # Publicar solo si hay datos válidos (al menos el hombro)
        self.brazo_pub.publish(brazo_msg)
# ==============================================================================
# 3. MAIN
# ==============================================================================
if __name__ == '__main__':
    try:
        node = TransfCoordNode()
        rospy.loginfo("Nodo TransfCoord iniciado con proyección métrica 3D...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
