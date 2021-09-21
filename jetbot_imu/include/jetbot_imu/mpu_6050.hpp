#pragma once
#include <Eigen/Core>
#include <unordered_map>

namespace Sensors
{
    class MPU_6050
    {
    public:
        enum AccelerometerRange
        {
            RANGE_2G = 0x00,
            RANGE_4G = 0x08,
            RANGE_8G = 0x10,
            RANGE_16G = 0x18
        };

        enum GyroscopeRange
        {
            RANGE_250DEG_PER_SEC = 0x00,
            RANGE_500DEG_PER_SEC = 0x08,
            RANGE_1000DEG_PER_SEC = 0x10,
            RANGE_2000DEG_PER_SEC = 0x18
        };

        MPU_6050(uint8_t i2c_bus, uint8_t i2c_slave_address);

        // Configure gyroscope range
        bool SetGyroRange(GyroscopeRange range);

        // Configure accelerometer range
        bool SetAccelRange(AccelerometerRange range);

        // Read angular velocity in rad/s
        bool ReadGyroscope(Eigen::Vector3d &theta_dot);

        // Read linear acceleration in m/s^2
        bool ReadAccelerometer(Eigen::Vector3d &accel);

        // Read the temperature in Celsius
        bool ReadTemperature(double &temp);

        // Returns gyroscope convariance
        Eigen::Matrix<double, 9, 1> GetGyroCovariance() const;

        // Returns accelerometer convariance
        Eigen::Matrix<double, 9, 1> GetAccelCovariance() const;

        double GetTempVariance() const;

        // Reset the sensor
        bool Reset();

        // Read the ID of the sensor
        bool ReadID(int8_t &data);

    private:
        int m_fd;
        AccelerometerRange m_arange;
        GyroscopeRange m_grange;

        enum Register
        {
            GYRO_CONFIG = 0x27,
            ACCEL_CONFIG = 0x28,
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
        };

        const std::unordered_map<GyroscopeRange, double> m_gyro_scale_factors = {
            {GyroscopeRange::RANGE_250DEG_PER_SEC, 131},
            {GyroscopeRange::RANGE_500DEG_PER_SEC, 65.5},
            {GyroscopeRange::RANGE_1000DEG_PER_SEC, 32.8},
            {GyroscopeRange::RANGE_2000DEG_PER_SEC, 16.4},
        };

        const std::unordered_map<AccelerometerRange, double> m_accel_scale_factors = {
            {AccelerometerRange::RANGE_2G, 16384},
            {AccelerometerRange::RANGE_4G, 8192},
            {AccelerometerRange::RANGE_8G, 4096},
            {AccelerometerRange::RANGE_16G, 2048},
        };

        bool ReadRegister(int8_t target_register, int8_t &data);
        bool ReadWord(Register higher, Register lower, int16_t &data);
        bool WriteRegister(int8_t target_register, int8_t data);

        int16_t ConcactenateBytes(int8_t high, int8_t low) const;
    };
}