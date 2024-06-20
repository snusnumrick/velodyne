"""
This module provides a class to convert a Velodyne scan to a PointCloud.
"""
from .calibration import Calibration
from .types import Model, FactoryId, BlockId, LaserCorrection
from .raw_packet import RawPacket
from .point_cloud import PointCloud
from .transformer import Transformer


class VelodyneCloudConverter:
    """
    Class to convert a Velodyne scan to a PointCloud.
    """

    def __init__(self):
        self._transformers = {}  # Using a dictionary to map Model to Transformer

    def decode(self, velodyne_scan) -> PointCloud | None:
        """
        Convert a Velodyne scan to a PointCloud.
        :param velodyne_scan: Message from '/velodyne_packets' topic
        :return: pointcloud or None if not supported
        """
        packets = velodyne_scan.packets
        if len(packets) == 0:
            return None

        # use the timestamp of the first packet
        time_stamp = packets[0].stamp

        lidar_model: Model | None = RawPacket.infer_model(packets[0])
        if lidar_model is None:
            # unsupported model
            return None

        max_points = RawPacket.MAX_POINTS_PER_PACKET * len(packets)
        cloud = PointCloud(stamp=time_stamp, max_points=max_points)
        transformer = self._transformers.get(lidar_model)
        if transformer is None:
            transformer = self.get_transformer(lidar_model)
            self._transformers[lidar_model] = transformer

        for packet in packets:
            transformer.unpack(RawPacket(packet.data), time_stamp, cloud, packet.stamp)

        cloud.trim()

        if cloud.width == 0 or cloud.height == 0:
            return None

        return cloud

    def get_transformer(self, lidar_model: Model) -> Transformer:
        """
        Get the transformer for a specific model with caching.
        :param lidar_model:
        :return:
        """
        # try to get the transformer from the cache first
        transformer = self._transformers.get(lidar_model)
        if transformer is not None:
            return transformer

        # create a new transformer and add it to the cache
        transformer = Transformer(Calibration(lidar_model))
        self._transformers[lidar_model] = transformer
        return transformer
