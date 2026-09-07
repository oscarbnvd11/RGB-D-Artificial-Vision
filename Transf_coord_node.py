#!/usr/bin/env python3

"""Mapea el movimiento relativo del brazo humano al espacio de mando.

El hombro humano y la base del robot comparten el origen `(0, 0, 0)`.
No se usa la posicion absoluta ni la orientacion fisica del robot.
"""

import math

import rospy  # type: ignore
from mediapipe_gesture_recognition.msg import Keypoint, Pose  # type: ignore


class TransfCoordNode:
    RIGHT_SHOULDER = 13
    RIGHT_WRIST = 17

    def __init__(self):
        rospy.init_node('transf_coord_node', anonymous=True)

        self.input_topic = rospy.get_param(
            '~input_topic', '/mediapipe_gesture_recognition/pose')
        self.output_topic = rospy.get_param('~output_topic', '/brazo')
        self.output_frame = rospy.get_param(
            '~output_frame', 'robot_base')
        self.expected_input_frame = rospy.get_param(
            '~expected_input_frame', '')
        self.require_wrist = rospy.get_param('~require_wrist', True)
        self.target_keypoints = [13, 15, 17, 19, 21, 23]

        self.brazo_pub = rospy.Publisher(
            self.output_topic, Pose, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            self.input_topic, Pose, self.pose_callback, queue_size=1)

        rospy.loginfo(
            f'TransfCoord: {self.input_topic} -> {self.output_topic}, '
            f'frame de salida: {self.output_frame}')

    def pose_callback(self, msg):
        if (self.expected_input_frame
                and msg.header.frame_id != self.expected_input_frame):
            rospy.logwarn_throttle(
                5.0,
                f'Frame de entrada inesperado: {msg.header.frame_id}; '
                f'se esperaba {self.expected_input_frame}.')

        points = {
            kp.keypoint_number: kp
            for kp in msg.keypoints
            if kp.keypoint_number in self.target_keypoints
            and self._is_valid_point(kp)
        }

        if self.RIGHT_SHOULDER not in points:
            rospy.logwarn_throttle(
                5.0, 'Pose descartada: hombro derecho sin coordenadas 3D validas.')
            return
        shoulder = points[self.RIGHT_SHOULDER]

        if self.require_wrist and self.RIGHT_WRIST not in points:
            rospy.logwarn_throttle(
                5.0, 'Pose descartada: muneca derecha sin coordenadas 3D validas.')
            return

        output = Pose()
        output.header.stamp = msg.header.stamp or rospy.Time.now()
        output.header.frame_id = self.output_frame

        for keypoint_number in self.target_keypoints:
            point = points.get(keypoint_number)
            if point is None:
                continue

            normalized = Keypoint()
            normalized.keypoint_number = point.keypoint_number
            normalized.keypoint_name = point.keypoint_name
            normalized.v = point.v
            normalized.x, normalized.y, normalized.z = self._map_motion_vector(
                point.x - shoulder.x,
                point.y - shoulder.y,
                point.z - shoulder.z)

            # depth conserva la profundidad absoluta medida por la camara.
            normalized.depth = point.depth
            output.keypoints.append(normalized)

        self.brazo_pub.publish(output)

    @staticmethod
    def _map_motion_vector(x, y, z):
        """Mapea camara (derecha, abajo, profundidad) a mando del robot."""
        return z, -x, -y

    @staticmethod
    def _is_valid_point(keypoint):
        return keypoint.depth > 0.0 and all(
            math.isfinite(float(value))
            for value in (keypoint.x, keypoint.y, keypoint.z, keypoint.depth)
        )


if __name__ == '__main__':
    try:
        TransfCoordNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
