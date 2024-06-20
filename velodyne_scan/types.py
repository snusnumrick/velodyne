"""
Data types
"""
from enum import Enum
from dataclasses import dataclass


class Model(Enum):
    VLP16 = 1
    VLP16HiRes = 2
    VLP32C = 3
    HDL32E = 4
    HDL64E = 5
    HDL64E_S21 = 6
    HDL64E_S3 = 7
    VLS128 = 8


class FactoryId(Enum):
    HDL32E = 0x21
    VLP16 = 0x22
    VLP32AB = 0x23
    VLP16HiRes = 0x24
    VLP32C = 0x28
    Velarray = 0x31
    VLS128Old = 0x63
    HDL64 = 0xa0
    VLS128 = 0xa1


class ReturnMode(Enum):
    Strongest = 0x37
    LastReturn = 0x38
    DualReturn = 0x39


class BlockId(Enum):
    Block_0_To_31 = 0xeeff
    Block_32_To_63 = 0xddff
    Block_64_To_95 = 0xccff
    Block_96_To_127 = 0xbbff


@dataclass
class LaserCorrection:
    laserId: int  # ring number for this laser
    rotCorrection: float
    vertCorrection: float
    distCorrection: float
    twoPtCorrectionAvailable: bool
    distCorrectionX: float
    distCorrectionY: float
    vertOffsetCorrection: float
    horizOffsetCorrection: float
    maxIntensity: int
    minIntensity: int
    focalDistance: float
    focalSlope: float
    # These cached values are calculated when the calibration file is read
    cosRotCorrection: float  # cosine of rotCorrection
    sinRotCorrection: float  # sine of rotCorrection
    cosVertCorrection: float  # cosine of vertCorrection
    sinVertCorrection: float  # sine of vertCorrection

