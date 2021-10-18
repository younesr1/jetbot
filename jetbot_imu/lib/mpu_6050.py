#!/usr/bin/python3
from typing import Iterable
import numpy as np
from enum import Enum
import smbus


class MPU_6050:
    class AccelerometereRange(Enum):
        RANGE_2G = 0x00,
        RANGE_4G = 0x08,
        RANGE_8G = 0x10,
        RANGE_16G = 0x18

    class GyroscopeRange(Enum):
        RANGE_250DEG_PER_SEC = 0x00,
        RANGE_500DEG_PER_SEC = 0x08,
        RANGE_1000DEG_PER_SEC = 0x10,
        RANGE_2000DEG_PER_SEC = 0x18

    class _Register(Enum):
        SMPLRT_DIV = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x27,
        ACCEL_CONFIG = 0x28,
        INT_ENABLE = 0X38,
        ACCEL_XOUT_H = 0x3B,
        ACCEL_XOUT_L = 0x3C,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_YOUT_L = 0x3E,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        TEMP_OUT_H = 0x41,
        TEMP_OUT_L = 0x42,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 = 0x6C,
        WHOAMI = 0x75

    def __init__(self, bus_num: int, address: int, g_range: GyroscopeRange, a_range: AccelerometereRange) -> None:
        self._addr = address
        self._bus = smbus.bus(bus_num)
        # write to sample rate register
        self._bus.write_byte_data(self._addr, self._Register.SMPLRT_DIV, 7)
        # Write to power management register
        self._bus.write_byte_data(self._addr, self._Register.PWR_MGMT_1, 1)
        # Write to Configuration register
        self._bus.write_byte_data(self._addr, self._Register.CONFIG, 0)
        # Write to interrupt enable register
        self._bus.write_byte_data(self._addr, self._Register.INT_ENABLE, 1)
        # Set gyro range
        self._bus.write_byte_data(
            self._addr, self._Register.GYRO_CONFIG, g_range)
        # Set accel range
        self._bus.write_byte_data(
            self._addr, self._Register.GYRO_CONFIG, a_range)

    def ReadGyroscope(self):
        pass

    def ReadAccelerometer(self):
        pass

    def ReadTemperature(self) -> float:
        pass

    def GetGyroCovariance(self):
        pass

    def GetAccelCovariance(self):
        pass

    def GetTempVariance(self) -> float:
        pass

    def Reset(self) -> None:
        pass

    def ReadID(self) -> None:
        pass

    def _ReadWord(self, lower: _Register, upper: _Register) -> int:
        # Accelero and Gyro value are 16-bit
        high = self._bus.read_byte_data(self._addr, upper)
        low = self._bus.read_byte_data(self._addr, lower)
        # concatenate higher and lower value
        value = ((high << 8) | low)
        # to get signed value from mpu6050
        return (value - 65536) if value > 32768 else value
