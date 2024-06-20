"""
Internal representation of a point cloud and a way to convert it to Open3D.
"""
from enum import Enum
from typing import List, NamedTuple, Optional, Callable
import struct
import numpy as np
from dataclasses import dataclass


class PointFieldDataType(Enum):
    INT8 = 1
    UINT8 = 2
    INT16 = 3
    UINT16 = 4
    INT32 = 5
    UINT32 = 6
    FLOAT32 = 7
    FLOAT64 = 8


@dataclass
class PointField:
    name: str
    offset: int
    datatype: PointFieldDataType
    count: int


@dataclass
class Point:
    xyz: np.ndarray
    distance: float
    intensity: float
    ring: int
    azimuth: int
    delta_ns: int


class PointCloud:
    POINT_STEP = 28

    def __init__(self, stamp: float, max_points: int):
        self.stamp = stamp
        self.fields: List[PointField] = [
            PointField("x", 0, PointFieldDataType.FLOAT32, 1),
            PointField("y", 4, PointFieldDataType.FLOAT32, 1),
            PointField("z", 8, PointFieldDataType.FLOAT32, 1),
            PointField("distance", 12, PointFieldDataType.FLOAT32, 1),
            PointField("intensity", 16, PointFieldDataType.FLOAT32, 1),
            PointField("ring", 20, PointFieldDataType.UINT16, 1),
            PointField("azimuth", 22, PointFieldDataType.UINT16, 1),
            PointField("delta_ns", 24, PointFieldDataType.UINT32, 1),
        ]
        self.height = 1
        self.width = 0
        self.is_bigendian = False
        self.point_step = self.POINT_STEP
        self.row_step = 0
        self.data = bytearray(max_points * self.POINT_STEP)
        self.is_dense = True

    def add_point(self, x: float, y: float, z: float, distance: float, intensity: float,
                  ring: int, azimuth: int, delta_ns: int):
        offset = self.width * self.POINT_STEP
        struct.pack_into('<fff', self.data, offset, x, y, z)
        struct.pack_into('<f', self.data, offset + 12, distance)
        struct.pack_into('<f', self.data, offset + 16, intensity)
        struct.pack_into('<HH', self.data, offset + 20, ring, azimuth)
        struct.pack_into('<I', self.data, offset + 24, delta_ns)
        self.width += 1
        self.row_step = self.width * self.POINT_STEP

    def point(self, index: int) -> Point:
        offset = index * self.POINT_STEP
        x, y, z = struct.unpack_from('<fff', self.data, offset)
        distance = struct.unpack_from('<f', self.data, offset + 12)[0]
        intensity = struct.unpack_from('<f', self.data, offset + 16)[0]
        ring, azimuth = struct.unpack_from('<HH', self.data, offset + 20)
        delta_ns = struct.unpack_from('<I', self.data, offset + 24)[0]
        return Point(np.array([x, y, z]), distance, intensity, ring, azimuth, delta_ns)

    def trim(self):
        self.data = self.data[:self.row_step]

    def to_open3d(self, tr=None, filter_func: Callable | None = None):
        """
        Convert to Open3D point cloud
        :param tr:
        :param filter_func:
        :return: o3d.geometry.PointCloud
        """
        import open3d as o3d
        import matplotlib.pyplot as plt

        # Extract points
        np_points = np.zeros((self.width, 3))
        np_intensities = np.zeros(self.width)
        for i in range(self.width):
            point = self.point(i)
            xyz = point.xyz
            if filter_func is not None and not filter_func(point):
                continue
            if tr is not None:
                xyz = np.dot(tr, np.append(point.xyz, 1))[:3]
            np_points[i] = xyz
            np_intensities[i] = point.intensity

        # Create Open3D point cloud
        o3d_point_cloud = o3d.geometry.PointCloud()
        o3d_point_cloud.points = o3d.utility.Vector3dVector(np_points)

        # add intensities as colors:
        intensities_normalized = (np_intensities - np.min(np_intensities)) / (np.max(np_intensities) - np.min(np_intensities))
        colors = plt.get_cmap('jet')(intensities_normalized)[:, :3]  # Drop alpha channel
        o3d_point_cloud.colors = o3d.utility.Vector3dVector(colors)  # Normalize

        return o3d_point_cloud
