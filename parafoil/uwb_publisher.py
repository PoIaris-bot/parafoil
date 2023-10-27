#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import serial
from rclpy.node import Node
from parafoil_msgs.msg import UWB
from parafoil.tool import unpack_uint8, unpack_int16, unpack_int24, unpack_float

UWB_START_BYTE = b'\x55'
UWB_DATA_LENGTH = 128


class UWBPublisher(Node):
    def __init__(self):
        super().__init__('uwb_publisher')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.serial = serial.Serial(port, baud_rate, timeout=1)

        self.publisher = self.create_publisher(UWB, 'uwb', 10)

        self.run()

    def run(self):
        buffer = []
        while rclpy.ok():
            buffer.append(self.serial.read())
            try:
                start_index = buffer.index(UWB_START_BYTE)
                if start_index + UWB_DATA_LENGTH <= len(buffer):
                    data = buffer[start_index:start_index + UWB_DATA_LENGTH]

                    check_value = 0
                    for num in data[:-1]:
                        check_value = (check_value + unpack_uint8(num)) % 256
                    if check_value == unpack_uint8(data[-1]):
                        position_x = unpack_int24(data[4:7]) / 1000
                        position_y = unpack_int24(data[7:10]) / 1000
                        position_z = unpack_int24(data[10:13]) / 1000

                        velocity_x = unpack_int24(data[13:16]) / 10000
                        velocity_y = unpack_int24(data[16:19]) / 10000
                        velocity_z = unpack_int24(data[19:22]) / 10000

                        angular_velocity_x = unpack_float(data[46:50])
                        angular_velocity_y = unpack_float(data[50:54])
                        angular_velocity_z = unpack_float(data[54:58])

                        acceleration_x = unpack_float(data[58:62])
                        acceleration_y = unpack_float(data[62:66])
                        acceleration_z = unpack_float(data[66:70])

                        angle_x = unpack_int16(data[82:84]) / 100
                        angle_y = unpack_int16(data[84:86]) / 100
                        angle_z = unpack_int16(data[86:88]) / 100

                        q1 = unpack_float(data[88:92])
                        q2 = unpack_float(data[92:96])
                        q3 = unpack_float(data[96:100])
                        q4 = unpack_float(data[100:104])

                        uwb = UWB()
                        uwb.id = unpack_uint8(data[2])
                        uwb.position = [position_x, position_y, position_z]
                        uwb.velocity = [velocity_x, velocity_y, velocity_z]
                        uwb.angle = [angle_x, angle_y, angle_z]
                        uwb.quaternion = [q1, q2, q3, q4]
                        uwb.acceleration = [acceleration_x, acceleration_y, acceleration_z]
                        uwb.angular_velocity = [angular_velocity_x, angular_velocity_y, angular_velocity_z]
                        self.publisher.publish(uwb)
                    buffer = buffer[start_index + UWB_DATA_LENGTH + 1:]
            except ValueError:
                pass

    def stop(self):
        self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    uwb_publisher = UWBPublisher()
    uwb_publisher.stop()
    uwb_publisher.destroy_node()
    rclpy.shutdown()