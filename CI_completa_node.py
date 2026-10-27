#!/usr/bin/env python3

"""
===============================================================================
SISTEMA DE VISIÓN ARTIFICIAL RGB-D Y CONTROL ROBÓTICO
===============================================================================


NODO DE CINEMÁTICA INVERSA COMPLETA

Descripción General:

Este nodo recibe la pose relativa publicada en `/brazo`, adapta las
dimensiones del brazo humano a los segmentos del robot y publica los cinco
comandos de los servos en `/robot/joint_angles`.

El hombro es el origen y el punto 17 representa el eje del servo de muñeca.
La posicion se escala con los segmentos hombro-codo y codo-muñeca de
`0.065 m`; el alcance hasta la muñeca es `0.130 m`. El tramo adicional de
`0.065 m` corresponde a la pinza.

Aunque conserva la estructura de la opcion completa, la salida usa el mismo
convenio fisico que `CI_desacoplada_node.py`: `q1` base, `q2` hombro, `q3`
codo, `q4` muñeca y `q5` pinza. De este modo publica comandos de servo y no
angulos internos alternativos del triangulo geometrico.

`q4` se calcula con el antebrazo real y la orientacion de muñeca-indice-
meñique. `q5` estima la apertura con la distancia tridimensional pulgar-indice:
`0` grados abierta, `45` media y `90` cerrada. Los ultimos valores validos de
`q4` y `q5` se conservan ante perdidas temporales.

Este nodo es una de las dos opciones de cinematica del proyecto. No se debe
ejecutar al mismo tiempo que `CI_desacoplada_node.py`.

"""

import math
from collections import deque

import rospy  # type: ignore
from mediapipe_gesture_recognition.msg import (  # type: ignore
    Keypoint,
    Pose,
)
from std_msgs.msg import Float32MultiArray  # type: ignore


class InverseKinematics5DOF:
    RIGHT_SHOULDER = 13
    RIGHT_ELBOW = 15
    RIGHT_WRIST = 17
    RIGHT_PINKY = 19
    RIGHT_INDEX = 21
    RIGHT_THUMB = 23

    def __init__(self):
        rospy.init_node('ci_completa_node', anonymous=True)

        self.input_topic = rospy.get_param('~input_topic', '/brazo')
        self.angle_output_topic = rospy.get_param(
            '~angle_output_topic', '/robot/joint_angles')
        self.scaled_pose_topic = rospy.get_param(
            '~scaled_pose_topic', '/brazo_robot')
        self.output_frame = rospy.get_param('~output_frame', 'robot_base')
        self.L2 = rospy.get_param('~shoulder_to_elbow', 0.065)
        self.L3 = rospy.get_param('~elbow_to_wrist', 0.065)
        self.L4 = rospy.get_param('~wrist_to_gripper', 0.065)
        self.robot_reach = self.L2 + self.L3 + self.L4

        # Una unica rama evita que el codo salte de un lado al otro.
        self.elbow_configuration = rospy.get_param(
            '~elbow_configuration', 'up')
        self.gripper_pitch_deg = rospy.get_param(
            '~gripper_pitch_deg', 0.0)
        self.reach_epsilon = rospy.get_param('~reach_epsilon', 0.001)
        self.wrist_roll_reference_deg = rospy.get_param(
            '~wrist_roll_reference_deg', None)
        self.wrist_roll_filter_alpha = rospy.get_param(
            '~wrist_roll_filter_alpha', 0.2)
        self.wrist_roll_initialized = False
        self.filtered_wrist_roll_deg = 0.0
        self.wrist_roll_samples = deque(maxlen=5)
        self.gripper_open_threshold = rospy.get_param(
            '~gripper_open_threshold', 0.11)
        self.gripper_closed_threshold = rospy.get_param(
            '~gripper_closed_threshold', 0.07)
        self.gripper_hysteresis_m = rospy.get_param(
            '~gripper_hysteresis_m', 0.0)
        self.gripper_distance_alpha = rospy.get_param(
            '~gripper_distance_alpha', 0.25)
        self.filtered_gripper_distance = None
        self.gripper_open_angle_deg = rospy.get_param(
            '~gripper_open_angle_deg', 0.0)
        self.gripper_half_angle_deg = rospy.get_param(
            '~gripper_half_angle_deg', 45.0)
        self.gripper_closed_angle_deg = rospy.get_param(
            '~gripper_closed_angle_deg', 90.0)
        self.latest_wrist_roll_deg = 0.0
        self.latest_gripper_angle_deg = 0.0
        self.joint_pub = rospy.Publisher(
            self.angle_output_topic, Float32MultiArray, queue_size=1)
        self.scaled_pose_pub = rospy.Publisher(
            self.scaled_pose_topic, Pose, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            self.input_topic, Pose, self.pose_callback, queue_size=1)

        rospy.loginfo(
            'IK completa: %s -> %s; longitudes %.3f, %.3f, %.3f m',
            self.input_topic,
            self.angle_output_topic,
            self.L2,
            self.L3,
            self.L4)

    def _calculate_gripper_angle(self, hand_points):
        """Convierte la distancia pulgar-indice en tres estados de pinza."""
        if (hand_points is None
                or self.RIGHT_THUMB not in hand_points
                or self.RIGHT_INDEX not in hand_points):
            return None

        thumb = self._coordinates(hand_points[self.RIGHT_THUMB])
        index = self._coordinates(hand_points[self.RIGHT_INDEX])
        raw_distance = self._length(self._subtract(index, thumb))

        if self.filtered_gripper_distance is None:
            self.filtered_gripper_distance = raw_distance
        else:
            alpha = self.gripper_distance_alpha
            self.filtered_gripper_distance = (
                alpha * raw_distance
                + (1.0 - alpha) * self.filtered_gripper_distance)
        distance = self.filtered_gripper_distance
        rospy.loginfo_throttle(
            1.0,
            'Pinza: distancia pulgar-indice %.1f mm (filtrada %.1f mm), '
            'visibilidad pulgar %.2f, indice %.2f',
            raw_distance * 1000.0,
            distance * 1000.0,
            hand_points[self.RIGHT_THUMB].v,
            hand_points[self.RIGHT_INDEX].v)

        open_angle = self.gripper_open_angle_deg
        half_angle = self.gripper_half_angle_deg
        closed_angle = self.gripper_closed_angle_deg
        hysteresis = self.gripper_hysteresis_m
        current_angle = self.latest_gripper_angle_deg

        if current_angle == open_angle:
            if distance <= self.gripper_closed_threshold - hysteresis:
                return closed_angle
            if distance <= self.gripper_open_threshold - hysteresis:
                return half_angle
            return open_angle
        if current_angle == closed_angle:
            if distance >= self.gripper_open_threshold + hysteresis:
                return open_angle
            if distance >= self.gripper_closed_threshold + hysteresis:
                return half_angle
            return closed_angle
        if distance <= self.gripper_closed_threshold - hysteresis:
            return closed_angle
        if distance >= self.gripper_open_threshold + hysteresis:
            return open_angle
        return half_angle

    def _filter_wrist_roll(self, measured_angle_deg):
        if measured_angle_deg is None:
            return None

        if self.wrist_roll_reference_deg is None:
            self.wrist_roll_reference_deg = measured_angle_deg
            rospy.loginfo(
                'Referencia de muñeca calibrada en %.1f grados',
                measured_angle_deg)

        relative_angle = self._wrap_degrees(
            measured_angle_deg - self.wrist_roll_reference_deg)
        if not self.wrist_roll_initialized:
            self.wrist_roll_samples.append(relative_angle)
            self.filtered_wrist_roll_deg = relative_angle
            self.wrist_roll_initialized = True
        else:
            self.wrist_roll_samples.append(relative_angle)
            aligned_angles = [
                self.filtered_wrist_roll_deg
                + self._wrap_degrees(
                    sample - self.filtered_wrist_roll_deg)
                for sample in self.wrist_roll_samples
            ]
            median_angle = sorted(aligned_angles)[
                len(aligned_angles) // 2]
            delta = self._wrap_degrees(
                median_angle - self.filtered_wrist_roll_deg)
            self.filtered_wrist_roll_deg = self._wrap_degrees(
                self.filtered_wrist_roll_deg
                + self.wrist_roll_filter_alpha * delta)
        return self.filtered_wrist_roll_deg

    @staticmethod
    def _wrap_degrees(angle):
        return (angle + 180.0) % 360.0 - 180.0

    def pose_callback(self, msg):
        points = {
            point.keypoint_number: point
            for point in msg.keypoints
            if point.keypoint_number in (
                self.RIGHT_SHOULDER,
                self.RIGHT_ELBOW,
                self.RIGHT_WRIST,
                self.RIGHT_PINKY,
                self.RIGHT_INDEX,
                self.RIGHT_THUMB)
            and self._is_valid_point(point)
        }
        required = (
            self.RIGHT_SHOULDER,
            self.RIGHT_ELBOW,
            self.RIGHT_WRIST)
        if any(keypoint_number not in points for keypoint_number in required):
            rospy.logwarn_throttle(
                5.0,
                'IK descartada: se necesitan hombro, codo y muneca validos.')
            return

        shoulder = self._coordinates(points[self.RIGHT_SHOULDER])
        elbow = self._coordinates(points[self.RIGHT_ELBOW])
        wrist = self._coordinates(points[self.RIGHT_WRIST])
        shoulder_elbow = self._subtract(elbow, shoulder)
        elbow_wrist = self._subtract(wrist, elbow)
        shoulder_wrist = self._subtract(wrist, shoulder)

        distance_shoulder_elbow = self._length(shoulder_elbow)
        distance_elbow_wrist = self._length(elbow_wrist)
        distance_shoulder_wrist = self._length(shoulder_wrist)
        if min(
                distance_shoulder_elbow,
                distance_elbow_wrist,
                distance_shoulder_wrist) <= 0.0:
            rospy.logwarn_throttle(
                5.0,
                'IK descartada: alguna distancia del brazo es cero.')
            return

        # Mismo escalado por segmentos usado en CI_desacoplada_node.py.
        scale_shoulder_elbow = self.L2 / distance_shoulder_elbow
        scale_elbow_wrist = self.L3 / distance_elbow_wrist
        scale_shoulder_wrist = (
            (self.L2 + self.L3) / distance_shoulder_wrist)

        scaled_shoulder = (0.0, 0.0, 0.0)
        scaled_elbow = self._scale(
            shoulder_elbow, scale_shoulder_elbow)
        scaled_wrist = self._add(
            scaled_elbow,
            self._scale(elbow_wrist, scale_elbow_wrist))

        # La relacion hombro-muneca limita el objetivo al alcance total.
        scaled_shoulder_wrist_limit = (
            distance_shoulder_wrist * scale_shoulder_wrist)
        scaled_wrist_distance = self._length(scaled_wrist)
        if scaled_wrist_distance > scaled_shoulder_wrist_limit:
            safety_scale = scaled_shoulder_wrist_limit / scaled_wrist_distance
            scaled_elbow = self._scale(scaled_elbow, safety_scale)
            scaled_wrist = self._scale(scaled_wrist, safety_scale)

        try:
            # q4 es un angulo: se calcula con el antebrazo humano real.
            forearm = elbow_wrist
            wrist_roll_deg = self._filter_wrist_roll(
                self._calculate_wrist_roll(points, forearm))
            if wrist_roll_deg is not None:
                self.latest_wrist_roll_deg = wrist_roll_deg
            wrist_roll_deg = self.latest_wrist_roll_deg

            gripper_angle_deg = self._calculate_gripper_angle(points)
            if gripper_angle_deg is not None:
                self.latest_gripper_angle_deg = gripper_angle_deg
            gripper_angle_deg = self.latest_gripper_angle_deg
            q1, q2, q3 = self._calculate_joint_angles(
                scaled_elbow, scaled_wrist)
            angles = (
                q1,
                q2,
                q3,
                math.radians(wrist_roll_deg),
                math.radians(gripper_angle_deg))
        except ValueError as error:
            rospy.logwarn_throttle(2.0, 'IK descartada: %s', error)
            return

        self._publish_scaled_pose(msg, points, {
            self.RIGHT_SHOULDER: scaled_shoulder,
            self.RIGHT_ELBOW: scaled_elbow,
            self.RIGHT_WRIST: scaled_wrist,
        })

        output = Float32MultiArray()
        output.data = [math.degrees(angle) for angle in angles]
        self.joint_pub.publish(output)
        rospy.loginfo_throttle(
            2.0,
            'IK -> q1 base %.1f, q2 hombro %.1f, '
            'q3 codo %.1f, q4 muneca %.1f, q5 pinza %.1f',
            *output.data)

    def solve(self, target, wrist_roll_deg=0.0, gripper_angle_deg=0.0):
        """Resuelve [q1 base, q2 hombro, q3 codo, q4 muneca, q5 pinza]."""
        target_x, target_y, target_z = target
        if not all(math.isfinite(value)
                   for value in (target_x, target_y, target_z)):
            raise ValueError('el objetivo contiene coordenadas no finitas')

        q_base = math.atan2(target_y, target_x)
        radial_target = math.hypot(target_x, target_y)
        gripper_pitch = math.radians(self.gripper_pitch_deg)

        # El objetivo es la punta; se obtiene el centro de la muneca
        # retirando el ultimo eslabon con la orientacion elegida.
        wrist_radial = radial_target
        wrist_height = target_z
        wrist_distance = math.hypot(wrist_radial, wrist_height)

        maximum_wrist_distance = self.L2 + self.L3 - self.reach_epsilon
        if wrist_distance > maximum_wrist_distance:
            scale = maximum_wrist_distance / wrist_distance
            wrist_radial *= scale
            wrist_height *= scale
        elif wrist_distance < self.reach_epsilon:
            wrist_radial = self.reach_epsilon
            wrist_height = 0.0

        cos_elbow = (
            (wrist_radial ** 2 + wrist_height ** 2 - self.L2 ** 2
             - self.L3 ** 2)
            / (2.0 * self.L2 * self.L3))
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_sign = 1.0 if self.elbow_configuration == 'up' else -1.0
        q_elbow = elbow_sign * math.acos(cos_elbow)

        target_angle = math.atan2(wrist_height, wrist_radial)
        shoulder_offset = math.atan2(
            self.L3 * math.sin(q_elbow),
            self.L2 + self.L3 * math.cos(q_elbow))
        q_shoulder = target_angle - shoulder_offset

        # q4 es el unico giro de la muneca alrededor del eje del antebrazo.
        q_wrist = math.radians(wrist_roll_deg)
        q_gripper = math.radians(gripper_angle_deg)
        return q_base, q_shoulder, q_elbow, q_wrist, q_gripper
    @classmethod
    def _calculate_joint_angles(cls, elbow, wrist):
        """Devuelve angulos de servo; q3 va de flexionado a extendido."""
        a, b, c = elbow
        d, e, f = cls._subtract(wrist, elbow)
        radial_elbow = math.hypot(a, b)
        radial_wrist = math.hypot(d, e)
        q1 = math.atan2(b, a)
        q2 = math.atan2(c, radial_elbow)
        q3 = math.pi / 2.0 - (math.atan2(f, radial_wrist) - q2)
        return q1, q2, q3

    def _calculate_wrist_roll(self, hand_points, forearm):
        """Expresa el giro de la palma alrededor del eje codo-muneca."""
        if (hand_points is None
                or self.RIGHT_WRIST not in hand_points
                or self.RIGHT_INDEX not in hand_points
                or self.RIGHT_PINKY not in hand_points):
            return None

        wrist = self._coordinates(hand_points[self.RIGHT_WRIST])
        index = self._coordinates(hand_points[self.RIGHT_INDEX])
        pinky = self._coordinates(hand_points[self.RIGHT_PINKY])

        hand_forward = self._subtract(index, wrist)
        palm_side = self._subtract(pinky, index)
        palm_normal = self._cross(hand_forward, palm_side)
        forearm_axis = self._unit(forearm)
        palm_normal = self._unit(palm_normal)
        if forearm_axis is None or palm_normal is None:
            return None

        # Referencia: vertical del robot proyectada sobre el plano de giro.
        reference = self._project_perpendicular((0.0, 0.0, 1.0), forearm_axis)
        if reference is None:
            reference = self._project_perpendicular(
                (1.0, 0.0, 0.0), forearm_axis)
        palm_normal = self._project_perpendicular(palm_normal, forearm_axis)
        if reference is None or palm_normal is None:
            return None

        signed_angle = math.atan2(
            self._dot(forearm_axis, self._cross(reference, palm_normal)),
            self._dot(reference, palm_normal))
        return math.degrees(signed_angle)

    def _publish_scaled_pose(self, msg, points, scaled_coordinates):
        output = Pose()
        output.header.stamp = msg.header.stamp or rospy.Time.now()
        output.header.frame_id = self.output_frame
        for point in points.values():
            if point.keypoint_number not in scaled_coordinates:
                continue
            scaled_point = Keypoint()
            scaled_point.keypoint_number = point.keypoint_number
            scaled_point.keypoint_name = point.keypoint_name
            scaled_point.v = point.v
            scaled_point.x, scaled_point.y, scaled_point.z = (
                scaled_coordinates[point.keypoint_number])
            scaled_point.depth = point.depth
            output.keypoints.append(scaled_point)
        self.scaled_pose_pub.publish(output)

    @staticmethod
    def _coordinates(point):
        return float(point.x), float(point.y), float(point.z)

    @staticmethod
    def _subtract(first, second):
        return tuple(a - b for a, b in zip(first, second))

    @staticmethod
    def _add(first, second):
        return tuple(a + b for a, b in zip(first, second))

    @staticmethod
    def _scale(vector, factor):
        return tuple(value * factor for value in vector)

    @staticmethod
    def _length(vector):
        return math.sqrt(sum(value * value for value in vector))

    @staticmethod
    def _is_valid_point(point):
        return all(math.isfinite(float(value))
                   for value in (point.x, point.y, point.z))

    @staticmethod
    def _cross(first, second):
        return (
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0])

    @staticmethod
    def _dot(first, second):
        return sum(a * b for a, b in zip(first, second))

    @classmethod
    def _unit(cls, vector):
        length = cls._length(vector)
        if length <= 1e-9:
            return None
        return cls._scale(vector, 1.0 / length)

    @classmethod
    def _project_perpendicular(cls, vector, axis):
        projected = cls._subtract(
            vector,
            cls._scale(axis, cls._dot(vector, axis)))
        return cls._unit(projected)


if __name__ == '__main__':
    try:
        InverseKinematics5DOF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass