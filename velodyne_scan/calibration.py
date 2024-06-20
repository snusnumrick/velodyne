"""
Calibration data for Velodyne LiDARs.
"""
from typing import Callable
from .types import Model, LaserCorrection
import json
from pathlib import Path
import math
from dataclasses import dataclass


@dataclass
class LaserEntry:
    """
    Laser entry in the calibration data.
    """
    rot_correction: float
    vert_correction: float
    dist_correction: float
    dist_correction_x: float
    dist_correction_y: float
    vert_offset_correction: float
    horiz_offset_correction: float
    focal_distance: float
    focal_slope: float
    laser_id: int
    two_pt_correction_available: bool | None = None
    max_intensity: int | None = None
    min_intensity: int | None = None


@dataclass
class CalibrationData:
    """
    Calibration data for a specific model.
    """
    lasers: list[LaserEntry]
    distance_resolution: float


def load_calibration_data(lidar_model: Model) -> CalibrationData | None:
    """
    Load calibration data for a specific model.
    :param lidar_model:
    :return:
    """
    json_fn = {
        Model.VLP16: "VLP16db",
        Model.VLP16HiRes: "VLP16_hires_db",
        Model.VLP32C: "VeloView-VLP-32C",
        Model.HDL32E: "32db",
        Model.HDL64E: "64e_utexas",
        Model.HDL64E_S21: "64e_s2.1-sztaki",
        Model.HDL64E_S3: "64e_s3-xiesc",
        Model.VLS128: "VLS128",
    }.get(lidar_model)
    if json_fn:
        json_path = Path(__file__).parent / "calibration" / f"{json_fn}.json"
        with json_path.open() as f:
            calibration_data_dict = json.load(f)
        if calibration_data_dict:
            # Convert the dictionary to a CalibrationData object
            lasers = [LaserEntry(**laser) for laser in calibration_data_dict['lasers']]
            return CalibrationData(lasers=lasers, distance_resolution=calibration_data_dict['distance_resolution'])
    return None


class Calibration:
    """
    Class to hold calibration data for a specific model.
    """
    ROTATION_RESOLUTION = 0.01  # [deg]
    ROTATION_MAX_UNITS = 36000  # [deg/100]

    VLP16_FIRINGS_PER_BLOCK = 2
    VLP16_SCANS_PER_FIRING = 16
    VLP16_BLOCK_TDURATION = 110.592  # [µs]
    VLP16_DSR_TOFFSET = 2.304  # [µs]
    VLP16_FIRING_TOFFSET = 55.296  # [µs]
    HDL32E_DSR_TOFFSET = 1.152  # [µs]
    HDL32E_FIRING_TOFFSET = 46.08  # [µs]
    VLS128_DSR_TOFFSET = 2.665  # [µs]
    VLS128_FIRING_TOFFSET = 53.5  # [µs]

    def __init__(self, model: Model, calibration_data: CalibrationData | None = None):
        # if calibration data not provided, load it from the file, based on the provided model
        calibration_data = calibration_data or load_calibration_data(model)

        self.model = model
        self.laser_corrections = [
            LaserCorrection(
                laserId=v.laser_id,
                rotCorrection=v.rot_correction,
                vertCorrection=v.vert_correction,
                distCorrection=v.dist_correction,
                twoPtCorrectionAvailable=v.two_pt_correction_available if v.two_pt_correction_available is not None else False,
                distCorrectionX=v.dist_correction_x,
                distCorrectionY=v.dist_correction_y,
                vertOffsetCorrection=v.vert_offset_correction,
                horizOffsetCorrection=v.horiz_offset_correction,
                maxIntensity=v.max_intensity if v.max_intensity is not None else 255,
                minIntensity=v.min_intensity if v.min_intensity is not None else 0,
                focalDistance=v.focal_distance,
                focalSlope=v.focal_slope,
                cosRotCorrection=math.cos(v.rot_correction),
                sinRotCorrection=math.sin(v.rot_correction),
                cosVertCorrection=math.cos(v.vert_correction),
                sinVertCorrection=math.sin(v.vert_correction),
            )
            for v in calibration_data.lasers
        ] if calibration_data else []
        self.distance_resolution = calibration_data.distance_resolution if calibration_data else 0
        self.timing_offsets = self.build_timings_for(model)

        # Set up cached values for sin and cos of all the possible headings
        self.cos_rot_table = [math.cos(math.radians(self.ROTATION_RESOLUTION * i)) for i in
                              range(self.ROTATION_MAX_UNITS)]
        self.sin_rot_table = [math.sin(math.radians(self.ROTATION_RESOLUTION * i)) for i in
                              range(self.ROTATION_MAX_UNITS)]

        self.vls128_laser_azimuth_cache = [0] * 16
        VLS128_CHANNEL_TDURATION = 2.665  # [µs]
        VLS128_SEQ_TDURATION = 53.3  # [µs]
        for i in range(16):
            self.vls128_laser_azimuth_cache[i] = (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8)

    @staticmethod
    def build_timings_for(model: Model) -> list[list[float]] | None:
        """
        Build timings for a specific model.
        :param model:
        :return: timings
        """

        block1 = lambda x, _y: x
        block16 = lambda x, y: x * 2 + y / 16
        point1 = lambda _x, y: y
        point2 = lambda _x, y: y / 2
        point16 = lambda _x, y: y % 16

        # Model specific configurations
        if model in (Model.VLP16, Model.VLP16HiRes):
            full = Calibration.VLP16_FIRING_TOFFSET
            single = Calibration.VLP16_DSR_TOFFSET
            return Calibration.build_timings(12, 32, full, single, 0, block16, point16)
        elif model == Model.VLP32C:
            full = Calibration.VLP16_FIRING_TOFFSET
            single = Calibration.VLP16_DSR_TOFFSET
            return Calibration.build_timings(12, 32, full, single, 0, block1, point2)
        elif model == Model.HDL32E:
            full = Calibration.HDL32E_FIRING_TOFFSET
            single = Calibration.HDL32E_DSR_TOFFSET
            return Calibration.build_timings(12, 32, full, single, 0, block1, point2)
        elif model == Model.VLS128:
            full = Calibration.VLS128_FIRING_TOFFSET
            single = Calibration.VLS128_DSR_TOFFSET
            return Calibration.build_timings(3, 17, full, single, -8.7, block1, point1)
        else:
            return []

    @staticmethod
    def build_timings(rows: int, cols: int, full_firing_us: float, single_firing_us: float, offset_us: float,
                      block: Callable, point: Callable) -> list[list[float]]:
        """
        Build timings for a specific model.
        :param rows:
        :param cols:
        :param full_firing_us:
        :param single_firing_us:
        :param offset_us:
        :param block:
        :param point:
        :return: timings
        """
        full_firing = full_firing_us * 1e-6
        single_firing = single_firing_us * 1e-6
        offset = offset_us * 1e-6

        timings = [
            [
                full_firing * block(x, y) + single_firing * point(x, y) + offset
                for y in range(cols)
            ]
            for x in range(rows)
        ]

        return timings
