#include <jetbot_imu/mpu_6050.hpp>
#include <string>
#include <stdexcept>
#include <linux/i2c-dev.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

extern "C"
{
#include <i2c/smbus.h>
}

namespace Sensors
{
    MPU_6050::MPU_6050(uint8_t i2c_bus, uint8_t i2c_slave_address, GyroscopeRange grange, AccelerometerRange arange)
    {
        std::string file = "/dev/i2c-" + std::to_string(i2c_bus);
        if ((m_fd = open(file.c_str(), O_RDWR)) < 0)
        {
            throw std::runtime_error("unable to open " + file);
        }

        if (ioctl(m_fd, I2C_SLAVE, i2c_slave_address) < 0)
        {
            throw std::runtime_error("unable to initiate i2c communication with slave " + i2c_slave_address);
        }

        if (!WriteRegister(Register::SMPLRT_DIV, 7))
        {
            throw std::runtime_error("unable to set IMU sample rate");
        }
        if (!WriteRegister(Register::PWR_MGMT_1, 1))
        {
            throw std::runtime_error("could not turn on IMU");
        }
        if (!WriteRegister(Register::CONFIG, 0))
        {
            throw std::runtime_error("could not write to config register");
        }
        if (!WriteRegister(Register::INT_ENABLE, 1))
        {
            throw std::runtime_error("could not write to interrupt register");
        }
        if (!SetGyroRange(grange))
        {
            throw std::runtime_error("could not set gyro range");
        }
        if (!SetAccelRange(arange))
        {
            throw std::runtime_error("could not set accel range");
        }
    }

    bool MPU_6050::SetGyroRange(MPU_6050::GyroscopeRange range)
    {
        if (WriteRegister(MPU_6050::Register::GYRO_CONFIG, range))
        {
            m_grange = range;
            return true;
        }
        return false;
    }

    bool MPU_6050::SetAccelRange(MPU_6050::AccelerometerRange range)
    {
        if (WriteRegister(MPU_6050::Register::ACCEL_CONFIG, range))
        {
            m_arange = range;
            return true;
        }
        return false;
    }

    bool MPU_6050::ReadRegister(int8_t target_register, int8_t &data)
    {
        auto res = i2c_smbus_read_word_data(m_fd, target_register);
        if (res < 0)
        {
            return false;
        }
        data = static_cast<int8_t>(res);
        return true;
    }

    bool MPU_6050::WriteRegister(int8_t target_register, int8_t data)
    {
        return i2c_smbus_write_word_data(m_fd, target_register, data) == 0;
    }

    int16_t MPU_6050::ConcactenateBytes(int8_t high, int8_t low) const
    {
        int16_t ret = low;
        ret |= high << 8;
        return ret;
    }

    bool MPU_6050::ReadWord(MPU_6050::Register upper, MPU_6050::Register lower, int16_t &data)
    {
        int8_t byte_high = 0, byte_low = 0;
        if (!ReadRegister(upper, byte_high))
        {
            return false;
        }
        if (!ReadRegister(lower, byte_low))
        {
            return false;
        }
        data = ConcactenateBytes(byte_high, byte_low);
        return true;
    }

    bool MPU_6050::ReadTemperature(double &temp)
    {
        int16_t temp_out = 0;
        if (ReadWord(Register::TEMP_OUT_H, Register::TEMP_OUT_L, temp_out))
        {
            temp = temp_out / 340.0 + 36.53;
            return true;
        }
        return false;
    }

    bool MPU_6050::ReadGyroscope(Eigen::Vector3d &theta_dot)
    {
        int16_t x = 0, y = 0, z = 0;
        if (!ReadWord(Register::GYRO_XOUT_H, Register::GYRO_XOUT_L, x))
        {
            return false;
        }
        if (!ReadWord(Register::GYRO_YOUT_H, Register::GYRO_YOUT_L, y))
        {
            return false;
        }
        if (!ReadWord(Register::GYRO_ZOUT_H, Register::GYRO_ZOUT_L, z))
        {
            return false;
        }

        double scale = m_gyro_scale_factors.at(m_grange);
        theta_dot.x() = x / scale;
        theta_dot.y() = y / scale;
        theta_dot.z() = z / scale;

        // younes todo convert from deg/s to rad/s
        constexpr auto deg_to_rad = M_PI / 180.0;
        theta_dot *= deg_to_rad;

        return true;
    }

    bool MPU_6050::ReadAccelerometer(Eigen::Vector3d &accel)
    {
        int16_t x = 0, y = 0, z = 0;
        if (!ReadWord(Register::ACCEL_XOUT_H, Register::ACCEL_XOUT_L, x))
        {
            return false;
        }
        if (!ReadWord(Register::ACCEL_YOUT_H, Register::ACCEL_YOUT_L, y))
        {
            return false;
        }
        if (!ReadWord(Register::ACCEL_ZOUT_H, Register::ACCEL_ZOUT_L, z))
        {
            return false;
        }

        double scale = m_accel_scale_factors.at(m_arange);
        accel.x() = x / scale;
        accel.y() = y / scale;
        accel.z() = z / scale;

        constexpr auto gravity = 9.80665;
        accel *= gravity;

        return true;
    }

    bool MPU_6050::Reset()
    {
        int8_t data = 1 << 7;
        return WriteRegister(Register::PWR_MGMT_1, data);
    }

    bool MPU_6050::ReadID(int8_t &data)
    {
        return ReadRegister(Register::WHOAMI, data);
    }

    Eigen::Matrix<double, 9, 1> MPU_6050::GetGyroCovariance() const
    {
        return Eigen::Matrix<double, 9, 1>::Zero();
    }

    Eigen::Matrix<double, 9, 1> MPU_6050::GetAccelCovariance() const
    {
        return Eigen::Matrix<double, 9, 1>::Zero();
    }

    double MPU_6050::GetTempVariance() const
    {
        return 0;
    }
}
