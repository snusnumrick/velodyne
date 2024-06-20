"""
Raw block from a Velodyne packet.
"""

from .types import BlockId
from .data_view import DataView


class RawBlock:
    """
    Raw block from a Velodyne packet.
    """
    def __init__(self, data: bytearray):
        self.data = data
        self.view = DataView(data)
        self.block_id = self.view.get_uint16(0)
        self.rotation = self.view.get_uint16(2)  # // [0-35999], divide by 100 to get degrees

    def is_upper_block(self) -> bool:
        return self.block_id == BlockId.Block_32_To_63.value

    def is_valid(self, index: int) -> bool:
        offset = 4 + 3 * index
        return self.data[offset] != 0 or self.data[offset + 1] != 0

    def distance(self, index: int) -> int:
        return self.view.get_uint16(4 + 3 * index)

    def intensity(self, index: int) -> int:
        return self.data[4 + 3 * index + 2]
