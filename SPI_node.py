#!/usr/bin/env python3

"""Transmite por SPI al Arduino los cinco angulos publicados por la IK."""

import math
import queue
import threading

import rospy  # type: ignore
import spidev  # type: ignore
from std_msgs.msg import (
    Empty,
    Float32MultiArray,
    Float64MultiArray,
)  # type: ignore


class SPIAngleTransmitter:
    HEADER = 0xAA
    FOOTER = 0x55

    def __init__(self):
        rospy.init_node('spi_node', anonymous=True)

        self.input_topic = rospy.get_param(
            '~input_topic', '/robot/joint_angles')
        self.medir_delay = rospy.get_param('~medir_delay', False)
        self.bus = rospy.get_param('~spi_bus', 0)
        self.device = rospy.get_param('~spi_device', 0)
        self.speed_hz = rospy.get_param('~spi_speed_hz', 500000)
        self.mode = rospy.get_param('~spi_mode', 0)
        if self.mode not in (0, 1, 2, 3):
            raise ValueError('~spi_mode debe estar entre 0 y 3')

        self.spi = spidev.SpiDev()
        try:
            self.spi.open(self.bus, self.device)
            self.spi.max_speed_hz = self.speed_hz
            self.spi.mode = self.mode
        except (OSError, IOError) as error:
            rospy.logfatal(
                'No se pudo abrir SPI bus %s, dispositivo %s: %s',
                self.bus,
                self.device,
                error)
            raise

        self.frame_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.sender_thread = threading.Thread(
            target=self._transmit_frames,
            daemon=True)
        self.sender_thread.start()

        angle_message_type = (
            Float64MultiArray if self.medir_delay else Float32MultiArray)
        self.angle_sub = rospy.Subscriber(
            self.input_topic, angle_message_type, self.angles_callback)
        rospy.loginfo(
            'Medicion de latencia %s; esperando %s en %s',
            'activada' if self.medir_delay else 'desactivada',
            angle_message_type.__name__,
            self.input_topic)
        self.ready_pub = rospy.Publisher(
            '/robot/spi_ready', Empty, queue_size=1, latch=True)
        rospy.on_shutdown(self.close)
        rospy.loginfo(
            'SPI listo: %s -> bus %s, dispositivo %s, modo %s, %s Hz',
            self.input_topic,
            self.bus,
            self.device,
            self.mode,
            self.speed_hz)
        self.ready_pub.publish(Empty())

    @classmethod
    def encode_angles(cls, angles):
        """Codifica [q1..q5] en header + payload + checksum + footer."""
        if len(angles) != 5:
            raise ValueError('Se esperaban exactamente cinco angulos')

        values = [float(angle) for angle in angles]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Los angulos deben ser valores finitos')

        q1 = cls._signed_5bit_level(values[0])
        q2 = cls._signed_5bit_level(values[1])
        q3 = cls._unsigned_4bit_level(values[2])
        q4 = cls._signed_5bit_level(values[3])
        q5 = min(range(3), key=lambda state: abs(values[4] - state * 45.0))

        packed = (
            (q1 << 16)
            | (q2 << 11)
            | (q3 << 7)
            | (q4 << 2)
            | q5)
        payload = packed.to_bytes(3, byteorder='big')
        checksum = (cls.HEADER + sum(payload)) & 0xFF
        return bytes((cls.HEADER, *payload, checksum, cls.FOOTER))

    @staticmethod
    def _signed_5bit_level(angle):
        clamped = min(90.0, max(-90.0, angle))
        return min(29, int((clamped + 90.0) / 6.0))

    @staticmethod
    def _unsigned_4bit_level(angle):
        clamped = min(90.0, max(0.0, angle))
        return min(14, int(clamped / 6.0))

    def angles_callback(self, message):
        start_time = None
        angle_values = message.data
        if self.medir_delay:
            if len(message.data) != 6:
                rospy.logwarn_throttle(
                    5.0,
                    'Vector de angulos invalido para medir_delay: '
                    'se esperaba marca temporal y cinco angulos.')
                return
            start_time = message.data[0]
            angle_values = message.data[1:]

        try:
            frame = self.encode_angles(angle_values)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(5.0, 'Angulos descartados: %s', error)
            return

        if start_time is not None:
            if not math.isfinite(start_time) or start_time <= 0.0:
                rospy.logwarn_throttle(
                    5.0,
                    'Marca temporal invalida; no se puede medir la latencia.')
            else:
                delay_ms = (rospy.Time.now().to_sec() - start_time) * 1000.0
                if delay_ms < 0.0:
                    rospy.logwarn_throttle(
                        5.0,
                        'La marca temporal esta en el futuro; comprueba '
                        'la sincronizacion del reloj ROS.')
                else:
                    rospy.loginfo_throttle(
                        1.0,
                        'Latencia pose -> generacion de trama SPI: %.3f ms',
                        delay_ms)

        self.frame_queue.put(frame)

    def _transmit_frames(self):
        while True:
            frame = self.frame_queue.get()
            if frame is None:
                self.frame_queue.task_done()
                return

            while not self.stop_event.is_set():
                try:
                    self.spi.xfer2(list(frame))
                except (OSError, IOError) as error:
                    rospy.logerr_throttle(
                        5.0,
                        'Error al transmitir por SPI; se reintentara: %s',
                        error)
                    if self.stop_event.wait(0.1):
                        break
                    continue

                rospy.loginfo_throttle(
                    2.0, 'Trama SPI enviada: %s', frame.hex(' '))
                break

            self.frame_queue.task_done()

    def close(self):
        self.stop_event.set()
        self.frame_queue.put(None)
        self.sender_thread.join()
        if self.spi:
            self.spi.close()


if __name__ == '__main__':
    try:
        SPIAngleTransmitter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
