#!/usr/bin/python3
import numpy as np
from enum import IntEnum
import smbus


class MPU_6050:
    class AccelerometereRange(IntEnum):
        RANGE_2G = 0x00,
        RANGE_4G = 0x08,
        RANGE_8G = 0x10,
        RANGE_16G = 0x18

    class GyroscopeRange(IntEnum):
        RANGE_250DEG_PER_SEC = 0x00,
        RANGE_500DEG_PER_SEC = 0x08,
        RANGE_1000DEG_PER_SEC = 0x10,
        RANGE_2000DEG_PER_SEC = 0x18

    class _Register(IntEnum):
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
        self._grange = g_range
        self._arange = a_range
        self._bus = smbus.SMBus(bus_num)
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
            self._addr, self._Register.ACCEL_CONFIG, a_range)

    def ReadGyroscope(self) -> np.array:
        gyro_scale_map = {self.GyroscopeRange.RANGE_250DEG_PER_SEC: 131.0,
                          self.GyroscopeRange.RANGE_500DEG_PER_SEC: 65.5,
                          self.GyroscopeRange.RANGE_1000DEG_PER_SEC: 32.8,
                          self.GyroscopeRange.RANGE_2000DEG_PER_SEC: 16.4}

        x = self._ReadWord(self._Register.GYRO_XOUT_H,
                           self._Register.GYRO_XOUT_L)
        y = self._ReadWord(self._Register.GYRO_YOUT_H,
                           self._Register.GYRO_YOUT_L)
        z = self._ReadWord(self._Register.GYRO_ZOUT_H,
                           self._Register.GYRO_ZOUT_L)
        return np.radians(np.array([x, y, z]) / gyro_scale_map[self._grange])

    def ReadAccelerometer(self) -> np.array:
        accel_scale_map = {self.AccelerometereRange.RANGE_2G: 16384.0,
                           self.AccelerometereRange.RANGE_4G: 8192.0,
                           self.AccelerometereRange.RANGE_8G: 4096.0,
                           self.AccelerometereRange.RANGE_16G: 2048.0}

        x = self._ReadWord(self._Register.ACCEL_XOUT_H,
                           self._Register.ACCEL_XOUT_L)
        y = self._ReadWord(self._Register.ACCEL_YOUT_H,
                           self._Register.ACCEL_YOUT_L)
        z = self._ReadWord(self._Register.ACCEL_ZOUT_H,
                           self._Register.ACCEL_ZOUT_L)
        return np.array([x, y, z]) / accel_scale_map[self._arange] * 9.81

    def ReadTemperature(self) -> float:
        return self._ReadWord(self._Register.TEMP_OUT_H, self._Register.TEMP_OUT_L) / 340.0 + 36.53

    def GetGyroCovariance(self) -> np.array:
        ret = np.zeros([3, 3])
        ret[0, 0] = 2.10816e-06
        ret[1, 1] = 2.59068e-06
        ret[2, 2] = 6.79929e-05
        return ret

    def GetAccelCovariance(self) -> np.array:
        ret = np.zeros([3, 3])
        ret[0, 0] = 0.00105985
        ret[1, 1] = 0.00224151
        ret[2, 2] = 0.00295389
        return ret

    def GetTempVariance(self) -> float:
        return 0.0511839

    def ReadID(self) -> int:
        return self._bus.read_byte_data(self._addr, self._Register.WHOAMI)

    def _ReadWord(self, upper: _Register, lower: _Register) -> int:
        # Accelero and Gyro value are 16-bit
        high = self._bus.read_byte_data(self._addr, upper)
        low = self._bus.read_byte_data(self._addr, lower)
        # concatenate higher and lower value
        value = ((high << 8) | low)
        # to get signed value from mpu6050
        return (value - 65536) if value > 32768 else value
