#!/usr/bin/env python3

"""
===============================================================================
SISTEMA DE VISIÓN ARTIFICIAL RGB-D Y CONTROL ROBÓTICO
===============================================================================


NODO DE CINEMÁTICA INVERSA DESACOPLADA

Descripción General:

Este nodo recibe la pose relativa publicada en `/brazo`, escala los segmentos
del brazo humano a las dimensiones del robot y publica los cinco comandos de
los servos en `/robot/joint_angles`.

El hombro es el origen. La muñeca (17) representa el eje del servo de muñeca,
por lo que el alcance de posicion llega a `0.130 m` con los segmentos
hombro-codo y codo-muñeca. El tramo adicional de `0.065 m` corresponde a la
pinza.

Convenio de salida:
`q1` base, `q2` hombro, `q3` codo, `q4` muñeca y `q5` pinza. Los tres primeros
angulos se calculan de forma desacoplada a partir de las direcciones del codo
y del antebrazo relativas al hombro. `q4` usa muñeca-indice-meñique y `q5`
estima la apertura a partir de la distancia pulgar-indice: `0` grados abierta,
`45` media y `90` cerrada.

Este nodo es una de las dos opciones de cinematica del proyecto. No se debe
ejecutar al mismo tiempo que `CI_completa_node.py`.

"""

import math

import rospy  # type: ignore
from mediapipe_gesture_recognition.msg import (  # type: ignore
    Keypoint,
    Pose,
)
from std_msgs.msg import Float32MultiArray  # type: ignore


class EscalaBrazoRobot:
    RIGHT_SHOULDER = 13
    RIGHT_ELBOW = 15
    RIGHT_WRIST = 17
    RIGHT_PINKY = 19
    RIGHT_INDEX = 21
    RIGHT_THUMB = 23

    def __init__(self):
        rospy.init_node('ci_desacoplada_node', anonymous=True)

        self.input_topic = rospy.get_param('~input_topic', '/brazo')
        self.output_topic = rospy.get_param(
            '~output_topic', '/brazo_robot')
        self.angle_output_topic = rospy.get_param(
            '~angle_output_topic', '/robot/joint_angles')
        self.output_frame = rospy.get_param('~output_frame', 'robot_base')
        self.wrist_roll_reference_deg = rospy.get_param(
            '~wrist_roll_reference_deg', None)
        self.wrist_roll_filter_alpha = rospy.get_param(
            '~wrist_roll_filter_alpha', 0.2)
        self.wrist_roll_initialized = False
        self.filtered_wrist_roll_deg = 0.0
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
        self.shoulder_to_elbow_robot = rospy.get_param(
            '~shoulder_to_elbow_robot', 0.065)
        self.elbow_to_wrist_robot = rospy.get_param(
            '~elbow_to_wrist_robot', 0.065)
        self.wrist_to_gripper_robot = rospy.get_param(
            '~wrist_to_gripper_robot', 0.065)
        self.robot_reach = (
            self.shoulder_to_elbow_robot
            + self.elbow_to_wrist_robot
            + self.wrist_to_gripper_robot)
        self.wrist_reach = (
            self.shoulder_to_elbow_robot
            + self.elbow_to_wrist_robot)

        self.pose_pub = rospy.Publisher(
            self.output_topic, Pose, queue_size=1)
        self.angle_pub = rospy.Publisher(
            self.angle_output_topic, Float32MultiArray, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            self.input_topic, Pose, self.pose_callback, queue_size=1)

        rospy.loginfo(
            'Escala de brazo: %s -> %s; robot %.3f m de alcance',
            self.input_topic,
            self.output_topic,
            self.robot_reach)

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
            'visibilidad pulgar %.2f, indice %.2f, q5 %.1f grados',
            raw_distance * 1000.0,
            distance * 1000.0,
            hand_points[self.RIGHT_THUMB].v,
            hand_points[self.RIGHT_INDEX].v,
            self.latest_gripper_angle_deg)

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
            self.filtered_wrist_roll_deg = relative_angle
            self.wrist_roll_initialized = True
        else:
            delta = self._wrap_degrees(
                relative_angle - self.filtered_wrist_roll_deg)
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
                'Escala descartada: se necesitan hombro, codo y muneca validos.')
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
                'Escala descartada: alguna distancia del brazo es cero.')
            return

        scale_shoulder_elbow = (
            self.shoulder_to_elbow_robot / distance_shoulder_elbow)
        scale_elbow_wrist = self.elbow_to_wrist_robot / distance_elbow_wrist
        scale_shoulder_wrist = self.wrist_reach / distance_shoulder_wrist
        scaled_shoulder_wrist_limit = (
            distance_shoulder_wrist * scale_shoulder_wrist)

        scaled_shoulder = shoulder
        scaled_elbow = self._add(
            scaled_shoulder,
            self._scale(shoulder_elbow, scale_shoulder_elbow))
        scaled_wrist = self._add(
            scaled_elbow,
            self._scale(elbow_wrist, scale_elbow_wrist))

        scaled_shoulder_wrist_distance = self._length(
            self._subtract(scaled_wrist, scaled_shoulder))
        if scaled_shoulder_wrist_distance > scaled_shoulder_wrist_limit:
            safety_scale = (
                scaled_shoulder_wrist_limit / scaled_shoulder_wrist_distance)
            scaled_elbow = self._add(
                scaled_shoulder,
                self._scale(
                    self._subtract(scaled_elbow, scaled_shoulder),
                    safety_scale))
            scaled_wrist = self._add(
                scaled_shoulder,
                self._scale(
                    self._subtract(scaled_wrist, scaled_shoulder),
                    safety_scale))

        rospy.loginfo_throttle(
            2.0,
            'Escalas hombro-codo %.3f, codo-muneca %.3f, '
            'hombro-muneca %.3f',
            scale_shoulder_elbow,
            scale_elbow_wrist,
            scale_shoulder_wrist)

        scaled_coordinates = {
            self.RIGHT_SHOULDER: scaled_shoulder,
            self.RIGHT_ELBOW: scaled_elbow,
            self.RIGHT_WRIST: scaled_wrist,
        }

        a, b, c = self._subtract(scaled_elbow, scaled_shoulder)
        d, e, f = self._subtract(scaled_wrist, scaled_elbow)
        q1, q2, q3 = self._calculate_joint_angles(a, b, c, d, e, f)
        wrist_roll_deg = self._filter_wrist_roll(
            self._calculate_wrist_roll(points, elbow_wrist))
        if wrist_roll_deg is not None:
            self.latest_wrist_roll_deg = wrist_roll_deg
        q4 = self.latest_wrist_roll_deg

        gripper_angle_deg = self._calculate_gripper_angle(points)
        if gripper_angle_deg is not None:
            self.latest_gripper_angle_deg = gripper_angle_deg
        q5 = self.latest_gripper_angle_deg

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

        self.pose_pub.publish(output)

        angles = Float32MultiArray()
        angles.data = [
            math.degrees(q1),
            math.degrees(q2),
            math.degrees(q3),
            q4,
            q5,
        ]
        self.angle_pub.publish(angles)

        rospy.loginfo_throttle(
            2.0,
            'IK: q1=%.1f, q2=%.1f, q3=%.1f, q4=%.1f, pinza=%.1f grados',
            *angles.data)

    @staticmethod
    def _calculate_joint_angles(a, b, c, d, e, f):
        """Calcula q1, q2 y q3 en radianes segun las formulas indicadas."""
        radial_elbow = math.hypot(a, b)
        radial_wrist = math.hypot(d, e)

        q1 = math.atan2(b, a)
        q2 = math.atan2(c, radial_elbow)
        q3 = math.pi / 2.0 - (math.atan2(f, radial_wrist) - q2)
        return q1, q2, q3

    def _calculate_wrist_roll(self, hand_points, forearm):
        """Calcula el unico giro de la palma alrededor del antebrazo."""
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
        palm_normal = self._unit(self._cross(hand_forward, palm_side))
        forearm_axis = self._unit(forearm)
        if palm_normal is None or forearm_axis is None:
            return None

        reference = self._project_perpendicular(
            (0.0, 0.0, 1.0), forearm_axis)
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
        EscalaBrazoRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass