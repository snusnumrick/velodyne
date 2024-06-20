"""
Raw packet from a Velodyne device.
"""
from .types import FactoryId, Model, ReturnMode
from .raw_block import RawBlock
from .data_view import DataView
from datetime import datetime


class RawPacket:
    """
    Raw packet from a Velodyne device.
    """
    BLOCKS_PER_PACKET = 12
    SCANS_PER_BLOCK = 32
    RAW_SCAN_SIZE = 3
    BLOCK_DATA_SIZE = SCANS_PER_BLOCK * RAW_SCAN_SIZE
    BLOCK_SIZE = BLOCK_DATA_SIZE + 4
    MAX_POINTS_PER_PACKET = BLOCKS_PER_PACKET * SCANS_PER_BLOCK

    def __init__(self, data: bytearray):
        blockSize = RawPacket.BLOCK_SIZE
        byteOffset = 0

        self.blocks = []
        for i in range(RawPacket.BLOCKS_PER_PACKET):
            blockData = data[byteOffset + blockSize * i: byteOffset + blockSize * i + blockSize]
            self.blocks.append(RawBlock(blockData))

        view = DataView(data, byteOffset)
        self.gpsTimestamp = view.get_uint32(1200, True)
        self.factoryField1 = data[1204]
        self.factoryField2 = data[1205]
        self.returnMode = ReturnMode(self.factoryField1) if self.factoryField1 in (member.value for member in
                                                                                   ReturnMode) else None
        self.factoryId = FactoryId(self.factoryField2) if self.factoryField2 in (member.value for member in
                                                                                 FactoryId) else None

        self.data = data
        self.packets = [RawBlock(data[i * RawPacket.BLOCK_SIZE: (i + 1) * RawPacket.BLOCK_SIZE]) for i in
                        range(RawPacket.BLOCKS_PER_PACKET)]

    def infer_model(self) -> Model:
        return RawPacket.InferModel(self.data)

    def timestamp(self, top_of_hour: datetime = None) -> float:
        """
        Method to be called on an instance of RawPacket to get the absolute timestamp.

        :param top_of_hour: Optional datetime object representing the top of the hour.
        :return: Absolute timestamp as fractional seconds since the UNIX epoch.
        """
        return RawPacket.gps_timestamp_to_timestamp(self.gpsTimestamp, top_of_hour)

    @staticmethod
    def InferModel(packet: bytearray) -> Model | None:
        """
        Infer the model of a Velodyne device from a packet.
        :param packet:
        :return: model or None if the model is not supported
        """
        factory_id = packet[1205]

        if factory_id == FactoryId.HDL32E.value:
            return Model.HDL32E
        elif factory_id == FactoryId.VLP16.value:
            return Model.VLP16
        elif factory_id == FactoryId.VLP32AB.value:
            return None
        elif factory_id == FactoryId.VLP16HiRes.value:
            return Model.VLP16HiRes
        elif factory_id == FactoryId.VLP32C.value:
            return Model.VLP32C
        elif factory_id == FactoryId.Velarray.value:
            return None
        elif factory_id == FactoryId.HDL64.value:
            # Is it possible to distinguish HDL64E / HDL64E_S21 / HDL64E_S3?
            return Model.HDL64E
        elif factory_id in (FactoryId.VLS128Old.value, FactoryId.VLS128.value):
            return Model.VLS128
        else:
            return None

    @staticmethod
    def gps_timestamp_to_timestamp(gps_timestamp: int, top_of_hour: datetime = None) -> float:
        """
        Convert a gpsTimestamp field representing the number of microseconds since the top of the hour
        to an absolute timestamp as fractional seconds since the UNIX epoch

        :param gps_timestamp: Number of microseconds since the top of the hour. This field is a member of the RawPacket class
        :param top_of_hour: Optional Date representing the top of the hour the gpsTimestamp is relative to.
        If unspecified, the most recent top of the hour (relative to now) will be used
        :return: seconds since the UNIX epoch
        """
        if top_of_hour is None:
            now = datetime.now()
            top_of_hour = now.replace(minute=0, second=0, microsecond=0)

        # Convert top_of_hour to a timestamp in seconds
        top_of_hour_timestamp = top_of_hour.timestamp()

        # Convert gpsTimestamp from microseconds to seconds and add to top_of_hour_timestamp
        absolute_timestamp = top_of_hour_timestamp + gps_timestamp / 1e6

        return absolute_timestamp
