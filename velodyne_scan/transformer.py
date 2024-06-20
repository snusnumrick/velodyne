"""
Transformer class : unpack raw packets and convert them to a point cloud
"""
from typing import Optional, Tuple, List
from dataclasses import dataclass
from std_msgs.msg import Time
from .calibration import Calibration
from .types import Model, FactoryId, BlockId, LaserCorrection, ReturnMode
from .raw_packet import RawPacket
from .point_cloud import PointCloud

VLP16_SCANS_PER_FIRING = 16
VLP16_BLOCK_TDURATION = 110.592  # [µs]
VLP16_DSR_TOFFSET = 2.304  # [µs]
VLP16_FIRING_TOFFSET = 55.296  # [µs]
VLS128_DISTANCE_RESOLUTION = 0.004  # [m]


@dataclass
class TransformerOptions:
    min_range: Optional[float] = None
    max_range: Optional[float] = None
    min_angle: Optional[int] = None
    max_angle: Optional[int] = None


Point = Tuple[float, float, float]


class Transformer:
    """
    Class to unpack raw packets and convert them to a point cloud.
    """

    def __init__(self, calibration: Calibration, options: TransformerOptions = TransformerOptions()):
        self.calibration = calibration
        self.min_range, self.max_range = default_range(calibration.model)
        self.min_range = options.min_range or self.min_range
        self.max_range = options.max_range or self.max_range
        self.min_angle = options.min_angle or 0
        self.max_angle = options.max_angle or 35999

    def unpack(self, raw: RawPacket, scan_stamp: Time, output: PointCloud, packet_stamp: Time | None = None) -> None:
        """
        Unpack a raw packet and put result into provided point cloud.
        :param raw:
        :param scan_stamp:
        :param output:
        :param packet_stamp:
        """
        if packet_stamp is None:
            packet_stamp = raw.timestamp()
        if raw.factoryId in (FactoryId.VLP16, FactoryId.VLP16HiRes):
            self._unpack_vlp16(raw, scan_stamp, packet_stamp, output)
        elif raw.factoryId in (FactoryId.VLS128, FactoryId.VLS128Old):
            self._unpack_vls128(raw, scan_stamp, packet_stamp, output)
        else:
            self._unpack_generic(raw, scan_stamp, packet_stamp, output)

    def _unpack_generic(self, raw, scan_stamp, packet_stamp, output):
        time_diff_start_to_this_packet = packet_stamp - scan_stamp

        for i in range(raw.BLOCKS_PER_PACKET):
            block = raw.blocks[i]
            raw_rotation = block.rotation
            if not angle_in_range(raw_rotation, self.min_angle, self.max_angle):
                continue

            timing_offsets_row = self.calibration.timing_offsets.get(i, [])
            bank_origin = 32 if block.is_upper_block() else 0

            for j in range(raw.SCANS_PER_BLOCK):
                if not block.is_valid(j):
                    continue

                laser_number = j + bank_origin
                corrections = self.calibration.laser_corrections[laser_number]

                raw_distance = block.distance(j)
                distance = raw_distance * self.calibration.distance_resolution + corrections.dist_correction
                if not distance_in_range(distance, self.min_range, self.max_range):
                    continue

                xyz = compute_position(distance, raw_rotation, self.calibration, corrections)
                raw_intensity = block.intensity(j)
                intensity = compute_intensity(raw_intensity, raw_distance, corrections)

                offset_sec = time_diff_start_to_this_packet + timing_offsets_row.get(j, 0)
                output.add_point(xyz[0], xyz[1], xyz[2], distance, intensity, corrections.laser_id, block.rotation,
                                 offset_sec * 1e9)

    def _unpack_vls128(self, raw, scan_stamp, packet_stamp, output):
        time_diff_start_to_this_packet = packet_stamp - scan_stamp
        dual_return = 1 if raw.returnMode == ReturnMode.DualReturn else 0
        block_count = RawPacket.BLOCKS_PER_PACKET - 4 * dual_return
        azimuth = 0
        azimuth_next = 0
        azimuth_diff = 0
        last_azimuth_diff = 0
        xyz = [0, 0, 0]  # Placeholder for 3D position calculation

        for i in range(block_count):
            block = raw.blocks[i]
            raw_rotation = block.rotation
            timing_offsets_row = self.calibration.timing_offsets[int(i / 4)] if int(
                i / 4) in self.calibration.timing_offsets else []
            bank_origin = bank_origin_for_block(block.block_id)

            if i == 0:
                azimuth = raw_rotation
            else:
                azimuth = azimuth_next

            if i < RawPacket.BLOCKS_PER_PACKET - (1 + dual_return):
                next_block = raw.blocks[i + (1 + dual_return)]
                azimuth_next = next_block.rotation
                azimuth_diff = (36000 + azimuth_next - azimuth) % 36000
                last_azimuth_diff = azimuth_diff
            else:
                azimuth_diff = 0 if i == RawPacket.BLOCKS_PER_PACKET - 4 * dual_return - 1 else last_azimuth_diff

            for j in range(RawPacket.SCANS_PER_BLOCK):
                raw_distance = block.distance(j)
                distance = raw_distance * VLS128_DISTANCE_RESOLUTION
                if not distance_in_range(distance, self.min_range, self.max_range):
                    continue

                laser_number = j + bank_origin
                firing_order = int(laser_number / 8)
                corrections = self.calibration.laser_corrections[laser_number]

                azimuth_correction = self.calibration.vls128_laser_azimuth_cache[firing_order]
                azimuth_corrected_f = azimuth + azimuth_diff * azimuth_correction
                azimuth_corrected = round(azimuth_corrected_f) % 36000
                if not angle_in_range(azimuth_corrected, self.min_angle, self.max_angle):
                    continue

                xyz = compute_position(distance, raw_rotation, self.calibration, corrections)

                intensity = block.intensity(j)

                timing_index = firing_order + int(laser_number / 64)
                offset_sec = time_diff_start_to_this_packet.to_nsec() + (
                    timing_offsets_row[timing_index] * 1e9 if timing_index in timing_offsets_row else 0)

                output.add_point(xyz[1], -xyz[0], xyz[2], distance, intensity, corrections.laserId, block.rotation,
                                 offset_sec)

    def _unpack_vlp16(self, raw, scan_stamp, packet_stamp, output):
        time_diff_start_to_this_packet = packet_stamp - scan_stamp
        azimuth_diff = 0
        last_azimuth_diff = 0
        xyz = [0, 0, 0]  # Placeholder for 3D position

        for i in range(RawPacket.BLOCKS_PER_PACKET):
            block = raw.blocks[i]
            raw_rotation = block.rotation
            timing_offsets_row = self.calibration.timing_offsets.get(i, [])

            if i < RawPacket.BLOCKS_PER_PACKET - 1:
                next_block = raw.blocks[i + 1]
                raw_azimuth_diff = (next_block.rotation - block.rotation) % 36000
                azimuth_diff = (36000 + raw_azimuth_diff) % 36000
                if raw_azimuth_diff < 0:
                    azimuth_diff = last_azimuth_diff
                last_azimuth_diff = azimuth_diff
            else:
                azimuth_diff = last_azimuth_diff

            for j in range(RawPacket.SCANS_PER_BLOCK):
                azimuth_corrected = self.vlp16_azimuth_corrected(block, j, azimuth_diff)

                if not angle_in_range(azimuth_corrected, self.min_angle, self.max_angle):
                    continue

                dsr = j % VLP16_SCANS_PER_FIRING
                corrections = self.calibration.laser_corrections[dsr]

                raw_distance = block.distance(j)
                distance = raw_distance * self.calibration.distance_resolution + corrections.dist_correction
                if not distance_in_range(distance, self.min_range, self.max_range):
                    continue

                xyz = compute_position(distance, raw_rotation, self.calibration, corrections)

                raw_intensity = block.intensity(j)
                intensity = self.compute_intensity(raw_intensity, raw_distance, corrections)

                offset_sec = time_diff_start_to_this_packet + timing_offsets_row.get(j, 0)

                output.add_point(xyz[1], -xyz[0], xyz[2], distance, intensity, corrections.laser_id, block.rotation,
                                 offset_sec * 1e9)


def angle_in_range(angle: float, min_angle: float, max_angle: float) -> bool:
    """
    Check if a given angle is within [min_angle..max_angle], handling wraparound.
    """
    if min_angle <= max_angle:
        return min_angle <= angle <= max_angle
    else:
        return angle >= min_angle or angle <= max_angle


def distance_in_range(distance: float, min_distance: float, max_distance: float) -> bool:
    """
    Check if a given distance is within [min_distance..max_distance].
    """
    return min_distance <= distance <= max_distance


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    Clamp a value in the range of [min_value..max_value].
    """
    return max(min_value, min(max_value, value))


def default_range(model) -> tuple:
    """
    Return the default [min, max] range of valid distances for a given hardware model.
    """
    if model in [Model.VLP16, Model.VLP16HiRes, Model.HDL32E]:
        return 0.4, 100
    elif model in [Model.HDL64E, Model.HDL64E_S21, Model.HDL64E_S3]:
        return 0.4, 120
    elif model == Model.VLP32C:
        return 0.4, 200
    elif model == Model.VLS128:
        return 0.4, 300
    return 0, 0  # Default case if model not recognized


def bank_origin_for_block(block_id) -> int:
    """
    Get the first block index for a bank of 32 lasers.
    """
    if block_id == BlockId.Block_0_To_31:
        return 0
    elif block_id == BlockId.Block_32_To_63:
        return 32
    elif block_id == BlockId.Block_64_To_95:
        return 64
    elif block_id == BlockId.Block_96_To_127:
        return 96
    return 0  # Default case


def sqr(x: float) -> float:
    """
    Faster square calculation.
    """
    return x * x


def vlp16_azimuth_corrected(block, laser_index: int, azimuth_diff: float) -> int:
    """
    Correct for the laser rotation as a function of timing during the firings.
    """
    dsr = laser_index % VLP16_SCANS_PER_FIRING
    firing = laser_index // VLP16_SCANS_PER_FIRING
    azimuth_corrected_f = \
        block.rotation + \
        (azimuth_diff * (dsr * VLP16_DSR_TOFFSET + firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION
    return round(azimuth_corrected_f) % 36000


def compute_position(distance: float, raw_rotation: float, calibration: Calibration,
                     corrections: LaserCorrection) -> Point:
    """
    Given an adjusted distance and raw azimuth reading from a laser return and
    calibration data, returns a 3D position in the output list.
    """
    cos_vert_angle = corrections.cosVertCorrection
    sin_vert_angle = corrections.sinVertCorrection
    cos_rot_correction = corrections.cosRotCorrection
    sin_rot_correction = corrections.sinRotCorrection

    # Assuming calibration.cosRotTable and sinRotTable are lists or similar structures
    # that allow indexing to retrieve precomputed cosine and sine values
    cos_rot = calibration.cos_rot_table[raw_rotation]
    sin_rot = calibration.sin_rot_table[raw_rotation]

    # cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    cos_rot_angle = cos_rot * cos_rot_correction + sin_rot * sin_rot_correction
    # sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    sin_rot_angle = sin_rot * cos_rot_correction - cos_rot * sin_rot_correction

    horiz_offset = corrections.horizOffsetCorrection
    vert_offset = corrections.vertOffsetCorrection

    # Compute the distance in the xy plane (without accounting for rotation)
    xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle

    # Calculate temporal X, use absolute value
    xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle
    # Calculate temporal Y, use absolute value
    yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle
    xx = abs(xx)
    yy = abs(yy)

    # Get 2 point calibration values, linear interpolation to get distance correction for X and Y
    distance_corr_x = 0
    distance_corr_y = 0
    if corrections.twoPtCorrectionAvailable:
        distance_corr_x = (((corrections.distCorrection - corrections.distCorrectionX) * (xx - 2.4)) / (
                25.04 - 2.4)) + corrections.distCorrectionX
        distance_corr_x -= corrections.distCorrection
        distance_corr_y = (((corrections.distCorrection - corrections.distCorrectionY) * (yy - 1.93)) / (
                25.04 - 1.93)) + corrections.distCorrectionY
        distance_corr_y -= corrections.distCorrection

    distance_x = distance + distance_corr_x
    xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle
    output_x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle

    distance_y = distance + distance_corr_y
    xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle
    output_y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle

    output_z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle

    return output_x, output_y, output_z


def compute_intensity(raw_intensity: float, raw_distance: float, corrections: LaserCorrection) -> float:
    """
    Given raw intensity and distance readings from a laser return and calibration
    data, returns a corrected intensity reading.
    """
    min_intensity = corrections.minIntensity
    max_intensity = corrections.maxIntensity
    focal_offset = 256 * (1 - corrections.focalDistance / 13100) ** 2
    focal_slope = corrections.focalSlope
    corrected_intensity = raw_intensity + focal_slope * abs(focal_offset - 256 * (1 - raw_distance / 65535) ** 2)

    # Clamp the corrected intensity between min_intensity and max_intensity
    return clamp(corrected_intensity, min_intensity, max_intensity)
