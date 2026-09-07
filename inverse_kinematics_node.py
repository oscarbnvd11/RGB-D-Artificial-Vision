#!/usr/bin/env python3

"""
-------------------------------------------------------------------------------
SISTEMA DE VISIÓN ARTIFICIAL RGB-D Y CONTROL ROBÓTICO
-------------------------------------------------------------------------------
Descripción General:
Este script implementa un nodo de ROS que recibe las coordenadas espaciales 
del brazo y calcula la cinemática inversa para un brazo robótico de 5 grados 
de libertad (5-DOF). Calcula los ángulos articulares necesarios (base, hombro, 
codo, muñeca) para que el robot alcance la posición deseada imitando el brazo 
humano, y publica los resultados para el control de motores.

Estructura:
1. IMPORTACIÓN DE LIBRERÍAS
2. CINEMÁTICA INVERSA (InverseKinematics5DOF)
3. MAIN
-------------------------------------------------------------------------------
"""

# ==============================================================================
# 1. IMPORTACIÓN DE LIBRERÍAS
# ==============================================================================
import rospy # type: ignore
import math # type: ignore
from std_msgs.msg import Float32MultiArray # type: ignore
from mediapipe_gesture_recognition.msg import Pose # type: ignore

# ==============================================================================
# 2. CINEMÁTICA INVERSA (InverseKinematics5DOF)
# ==============================================================================
class InverseKinematics5DOF:
    def __init__(self):
        rospy.init_node('inverse_kinematics_node', anonymous=True)

        # Publicador: Ángulos [Base, Hombro, Codo, Muñeca Pitch, Muñeca Roll]
        self.joint_pub = rospy.Publisher('/robot/joint_angles', Float32MultiArray, queue_size=10)

        # Suscriptor: Coordenadas del brazo desde MediaPipe (en metros, origen = hombro)
        rospy.Subscriber('/brazo', Pose, self.pose_callback)

        # ---------------------------------------------------------
        # CONFIGURACIÓN FÍSICA DEL ROBOT (Valores aproximados en metros)
        # -> Ajusta L2, L3 y L4 midiendo tu robot real con una regla.
        # ---------------------------------------------------------
        self.L2 = 0.065  # Húmero (Hombro a Codo)
        self.L3 = 0.065  # Antebrazo (Codo a Muñeca)
        self.L4 = 0.065  # Efector final (Muñeca a punta de la pinza)

        # Alcance máximo del brazo humano (Hombro a muñeca en metros aprox.)
        self.HUMAN_ARM_LENGTH = 0.50  

        # Calculamos el Factor de Escala automáticamente
        self.ROBOT_REACH = self.L2 + self.L3
        self.SCALE_FACTOR = self.ROBOT_REACH / self.HUMAN_ARM_LENGTH

    def pose_callback(self, msg):
        wrist_kp = None
        
        # Buscamos el punto objetivo de la muñeca (Keypoint 17 - RIGHT_WRIST)
        for kp in msg.keypoints:
            if kp.keypoint_number == 17:
                wrist_kp = kp
                break

        if not wrist_kp:
            return

        # ---------------------------------------------------------
        # 1. MAPEO DE EJES (Cámara -> Robot)
        # ---------------------------------------------------------
        # La cámara ve: X=Derecha, Y=Abajo, Z=Profundidad
        # El robot usa: X=Adelante, Y=Izquierda, Z=Arriba
        x_robot = wrist_kp.z         # Profundidad de cámara = Avance del robot
        y_robot = -wrist_kp.x        # Derecha de cámara (-x) = Izquierda del robot (+y)
        z_robot = -wrist_kp.y        # Abajo de cámara (-y) = Arriba del robot (+z)

        # ---------------------------------------------------------
        # 2. FACTOR DE ESCALA (Teleoperación proporcional)
        # ---------------------------------------------------------
        x = x_robot * self.SCALE_FACTOR
        y = y_robot * self.SCALE_FACTOR
        z = z_robot * self.SCALE_FACTOR

        try:
            # ---------------------------------------------------------
            # 3. ÁNGULO DE LA BASE (Giro horizontal Yaw)
            # ---------------------------------------------------------
            q_base = math.atan2(y, x)

            # Proyección del objetivo en el plano radial (distancia 2D vista desde arriba)
            r = math.sqrt(x**2 + y**2)

            # ---------------------------------------------------------
            # 4. DESFASE DE LA PINZA (Orientación de la muñeca)
            # ---------------------------------------------------------
            pitch_target = 0.0  # 0.0 = Pinza paralela al suelo.
            
             # Mapeo Muñeca-a-Muñeca: El objetivo de la IK es directamente la muñeca del robot.
            # (No restamos L4 porque queremos que la muñeca del robot vaya a donde está la humana)
            r_eff = r
            z_eff = z

            # Distancia directa desde el hombro al punto efectivo
            d = math.sqrt(r_eff**2 + z_eff**2)

            # PROTECCIÓN: Si el humano estira el brazo MÁS de la cuenta, capamos el valor
            # para que el robot simplemente se estire al máximo sin que las mates den error.
            if d > self.ROBOT_REACH:
                d = self.ROBOT_REACH - 0.001 # Restamos un milímetro para evitar división por cero
                # Recalculamos r_eff y z_eff manteniendo la misma proporción (ángulo de elevación)
                angle_elev = math.atan2(z_eff, r_eff)
                r_eff = d * math.cos(angle_elev)
                z_eff = d * math.sin(angle_elev)

            # ---------------------------------------------------------
            # 5. CINEMÁTICA INVERSA 2D (Hombro y Codo)
            # ---------------------------------------------------------
            # Ángulo del Codo (q_codo) por Ley de Cosenos
            cos_q_codo = (r_eff**2 + z_eff**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            # Limitamos el dominio por imprecisiones flotantes
            cos_q_codo = max(-1.0, min(1.0, cos_q_codo))
            q_codo = math.acos(cos_q_codo)

            # Ángulo del Hombro (q_hombro)
            alpha = math.atan2(z_eff, r_eff)
            beta = math.atan2(self.L3 * math.sin(q_codo), self.L2 + self.L3 * math.cos(q_codo))
            q_hombro = alpha + beta

            # ---------------------------------------------------------
            # 6. MUÑECA (Pitch y Roll)
            # ---------------------------------------------------------
            # Compensamos la muñeca para que mantenga el pitch_target deseado
            q_wrist_pitch = pitch_target - (q_hombro - q_codo)
            q_wrist_roll = 0.0  # Lo mantenemos a 0 por ahora

            # ---------------------------------------------------------
            # 7. CONVERSIÓN A GRADOS Y PUBLICACIÓN
            # ---------------------------------------------------------
            angles_deg = [
                math.degrees(q_base),
                math.degrees(q_hombro),
                math.degrees(q_codo),
                math.degrees(q_wrist_pitch),
                math.degrees(q_wrist_roll)
            ]

            angles_msg = Float32MultiArray()
            angles_msg.data = angles_deg
            self.joint_pub.publish(angles_msg)

            # Log para ver los datos en vivo en la terminal
            rospy.loginfo(f"IK -> Base: {angles_deg[0]:.0f}°, Hombro: {angles_deg[1]:.0f}°, Codo: {angles_deg[2]:.0f}°")

        except ValueError as e:
            rospy.logerr(f"Error matemático en Cinemática Inversa: {e}")

# ==============================================================================
# 3. MAIN
# ==============================================================================
if __name__ == '__main__':
    try:
        node = InverseKinematics5DOF()
        rospy.loginfo("Nodo IK 5-DOF iniciado. Hombro=Origen, Escala Proporcional Activada.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass