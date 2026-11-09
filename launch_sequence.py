#!/usr/bin/env python3

"""Lanza los nodos del brazo en secuencia al recibir sus primeras salidas."""

import rospy  # type: ignore
from roslaunch.core import Node, RLException  # type: ignore
from roslaunch.scriptapi import ROSLaunch  # type: ignore
from mediapipe_gesture_recognition.msg import Pose  # type: ignore
from std_msgs.msg import Empty, Float32MultiArray, Float64MultiArray  # type: ignore


class LaunchSequence:
    STREAM_READY_TOPIC = '/mediapipe_gesture_recognition/stream_ready'
    DETECTED_POSE_TOPIC = '/mediapipe_gesture_recognition/pose'
    TRANSFORMED_POSE_TOPIC = '/brazo'
    JOINT_ANGLES_TOPIC = '/robot/joint_angles'
    SPI_READY_TOPIC = '/robot/spi_ready'

    def __init__(self):
        rospy.init_node('rgbd_robot_launch_sequence')

        self.package = rospy.get_param('~ros_package')
        self.kinematics = rospy.get_param('~kinematics', 'desacoplada')
        self.medir_delay = rospy.get_param('~medir_delay', False)
        self.parallax_filter_enabled = rospy.get_param(
            '~parallax_filter_enabled', True)
        self.z_diagnostics_enabled = rospy.get_param(
            '~z_diagnostics_enabled', True)
        self.ready_timeout = rospy.get_param('~ready_timeout', 180.0)
        if self.kinematics not in ('desacoplada', 'completa'):
            raise ValueError(
                '~kinematics debe ser "desacoplada" o "completa"')
        if self.ready_timeout <= 0.0:
            raise ValueError('~ready_timeout debe ser mayor que cero')

        self.launcher = ROSLaunch()

    def _launch_node(self, executable, name, args=''):
        node = Node(
            self.package,
            executable,
            name=name,
            output='screen',
            args=args)
        self.launcher.launch(node)
        rospy.loginfo('Nodo iniciado: %s', name)

    def _wait_for_message(self, topic, message_type, description):
        rospy.loginfo('Esperando confirmación: %s (%s)', description, topic)
        rospy.wait_for_message(
            topic,
            message_type,
            timeout=self.ready_timeout)
        rospy.loginfo('Confirmación recibida: %s', description)

    def run(self):
        try:
            self.launcher.start()
            self._launch_node(
                'Stream_node.py',
                'mediapipe_stream_node',
                '_enable_pose:=true _medir_delay:=%s '
                '_parallax_filter_enabled:=%s _z_diagnostics_enabled:=%s'
                % (
                    str(self.medir_delay).lower(),
                    str(self.parallax_filter_enabled).lower(),
                    str(self.z_diagnostics_enabled).lower()))
            self._launch_node(
                'Transf_coord_node.py',
                'transf_coord_node')

            if self.kinematics == 'completa':
                kinematics_node = 'CI_completa_node.py'
                kinematics_name = 'ci_completa_node'
            else:
                kinematics_node = 'CI_desacoplada_node.py'
                kinematics_name = 'ci_desacoplada_node'

            medir_delay_arg = '_medir_delay:=%s' % str(
                self.medir_delay).lower()
            self._launch_node(
                kinematics_node, kinematics_name, medir_delay_arg)
            self._launch_node('SPI_node.py', 'spi_node', medir_delay_arg)
            self._wait_for_message(
                self.STREAM_READY_TOPIC,
                Empty,
                'primer fotograma RGB-D recibido')
            self._wait_for_message(
                self.DETECTED_POSE_TOPIC,
                Pose,
                'primera pose detectada por MediaPipe')
            self._wait_for_message(
                self.TRANSFORMED_POSE_TOPIC,
                Pose,
                'primera pose relativa válida')
            angle_message_type = (
                Float64MultiArray
                if self.medir_delay else Float32MultiArray)
            self._wait_for_message(
                self.JOINT_ANGLES_TOPIC,
                angle_message_type,
                'primer vector de ángulos articulares')
            self._wait_for_message(
                self.SPI_READY_TOPIC,
                Empty,
                'transmisor SPI abierto y listo')
            rospy.loginfo(
                'Secuencia completada. Los nodos están activos; '
                'pulsa Ctrl+C para detenerlos.')
            rospy.spin()
        finally:
            self.launcher.stop()


if __name__ == '__main__':
    try:
        LaunchSequence().run()
    except rospy.ROSInterruptException:
        pass
    except (rospy.ROSException, RLException, ValueError) as error:
        rospy.logfatal('No se pudo completar el lanzamiento secuencial: %s',
                       error)
