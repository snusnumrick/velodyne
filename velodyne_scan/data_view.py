"""
This module provides a DataView class that wraps a memoryview and
provides methods to read various types of data from a buffer.
"""
import struct


class DataView:
    """
    Wrapper around a memoryview providing methods to read various types of data from a buffer.
    """
    def __init__(self, buffer: bytes, byte_offset: int = 0, byte_length: int | None = None):
        self.buffer: memoryview = memoryview(buffer)
        self.byte_offset = byte_offset
        self.byte_length = len(self.buffer) - byte_offset if byte_length is None else byte_length

    def get_float32(self, byte_offset, little_endian=True):
        fmt = '<f' if little_endian else '>f'
        return struct.unpack_from(fmt, self.buffer, self.byte_offset + byte_offset)[0]

    def get_float64(self, byte_offset, little_endian=True):
        fmt = '<d' if little_endian else '>d'
        return struct.unpack_from(fmt, self.buffer, self.byte_offset + byte_offset)[0]

    def get_int8(self, byte_offset):
        return struct.unpack_from('b', self.buffer, self.byte_offset + byte_offset)[0]

    def get_int16(self, byte_offset, little_endian=True):
        fmt = '<h' if little_endian else '>h'
        return struct.unpack_from(fmt, self.buffer, self.byte_offset + byte_offset)[0]

    def get_int32(self, byte_offset, little_endian=True):
        fmt = '<i' if little_endian else '>i'
        return struct.unpack_from(fmt, self.buffer, self.byte_offset + byte_offset)[0]

    def get_uint8(self, byte_offset):
        return struct.unpack_from('B', self.buffer, self.byte_offset + byte_offset)[0]

    def get_uint16(self, byte_offset, little_endian=True):
        fmt = '<H' if little_endian else '>H'
        return struct.unpack_from(fmt, self.buffer, self.byte_offset + byte_offset)[0]

    def get_uint32(self, byte_offset, little_endian=True):
        fmt = '<I' if little_endian else '>I'
        return struct.unpack_from(fmt, self.buffer, self.byte_offset + byte_offset)[0]