#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
import serial
from rclpy.node import Node
from parafoil_msgs.msg import RCIn
from serial.serialutil import SerialException


class RCReceiver(Node):
    def __init__(self):
        super().__init__('rc_receiver')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        try:
            self.serial = serial.Serial(
                port=self.port, baudrate=100000, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO
            )
        except SerialException:
            self.serial = None

        self.publisher = self.create_publisher(RCIn, 'parafoil/rc/in', 10)

        self.run()

    def run(self):
        buffer = []
        while rclpy.ok():
            if self.serial is not None:
                try:
                    byte = self.serial.read()
                    buffer.append(byte)
                except SerialException:
                    self.serial.close()
                    self.serial = None
            else:
                try:
                    self.serial = serial.Serial(
                        port=self.port, baudrate=100000, bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO
                    )
                except SerialException:
                    self.serial = None

            try:
                start_index = buffer.index(b'\x0f')
                if start_index + 25 <= len(buffer):
                    data = [eval('0x' + x.hex()) for x in buffer[start_index:start_index + 25]]
                    channels = [0] * 16

                    channels[0] = ((data[1] | data[2] << 8) & 0x07FF)
                    channels[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF)
                    channels[2] = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF)
                    channels[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF)
                    channels[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF)
                    channels[5] = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF)
                    channels[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF)
                    channels[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF)
                    channels[8] = ((data[12] | data[13] << 8) & 0x07FF)
                    channels[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF)
                    channels[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF)
                    channels[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF)
                    channels[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF)
                    channels[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF)
                    channels[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF)
                    channels[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF)

                    rc_in = RCIn()
                    rc_in.timestamp = time.time()
                    rc_in.channels = channels
                    self.publisher.publish(rc_in)
                    buffer = buffer[start_index + 25 + 1:]
            except ValueError:
                pass


def main(args=None):
    rclpy.init(args=args)
    rc_receiver = RCReceiver()
    rc_receiver.destroy_node()
    rclpy.shutdown()
