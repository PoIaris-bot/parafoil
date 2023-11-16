import struct
import numpy as np


def constraint(value, lower, upper):
    return min(max(value, lower), upper)


def map_value(value, lower1, upper1, lower2, upper2):
    value = constraint(value, lower1, upper1)
    return (value - lower1) * (upper2 - lower2) / (upper1 - lower1) + lower2


def unpack_uint8(byte):
    value = byte
    return struct.unpack('<B', value)[0]


def unpack_int16(byte):
    value = byte[0] + byte[1]
    return struct.unpack('<h', value)[0]


def unpack_int24(byte):
    value = b'\x00' + byte[0] + byte[1] + byte[2]
    return struct.unpack('<i', value)[0] / 256


def unpack_float(byte):
    value = byte[0] + byte[1] + byte[2] + byte[3]
    return struct.unpack('<f', value)[0]


def remap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))
