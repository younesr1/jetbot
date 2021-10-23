#!/usr/bin/python3
from mpu_6050 import MPU_6050
import rospy
import numpy as np
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Vector3
from typing import Iterable


def StringToGyroRange(range: str) -> MPU_6050.GyroscopeRange:
    return {"250": MPU_6050.GyroscopeRange.RANGE_250DEG_PER_SEC,
            "500": MPU_6050.GyroscopeRange.RANGE_500DEG_PER_SEC,
            "1000": MPU_6050.GyroscopeRange.RANGE_1000DEG_PER_SEC,
            "2000": MPU_6050.GyroscopeRange.RANGE_2000DEG_PER_SEC}.get(range)


def StringToAccelRange(range: str) -> MPU_6050.AccelerometereRange:
    return {"2G": MPU_6050.AccelerometereRange.RANGE_2G,
            "4G": MPU_6050.AccelerometereRange.RANGE_4G,
            "8G": MPU_6050.AccelerometereRange.RANGE_8G,
            "16G": MPU_6050.AccelerometereRange.RANGE_16G}.get(range)

def NpArrayToVector3(np: Iterable) -> Iterable:
    ret = Vector3()
    ret.x, ret.y, ret.z = np
    return ret

def main():
    # younes todo get thsese from param server
    bus_num = 0
    slave_address = 0x68
    buffer = 10
    freq = 50
    imu_topic = "/imu/raw/data"
    temp_topic = "/imu/raw/temperature"
    gyro_range = "250"
    accel_range = "2G"

    rospy.init_node("imu")
    rospy.loginfo(
        f"Contacting IMU at address {slave_address} on bus {bus_num}")
    imu = MPU_6050(bus_num, slave_address, StringToGyroRange(
        gyro_range), StringToAccelRange(accel_range))
    imu_pub = rospy.Publisher(imu_topic,Imu, queue_size = buffer)
    temperature_pub = rospy.Publisher(
        temp_topic,Temperature, queue_size = buffer)

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        imu_msg =Imu()
        imu_msg.angular_velocity = NpArrayToVector3(imu.ReadGyroscope())
        imu_msg.angular_velocity_covariance = imu.GetGyroCovariance().tolist()
        imu_msg.linear_acceleration = NpArrayToVector3(imu.ReadAccelerometer())
        imu_msg.linear_acceleration_covariance = imu.GetAccelCovariance().tolist()
        # MPU-6050 gives no orientation reading
        imu_msg.orientation_covariance = np.zeros(9).tolist()
        imu_msg.orientation_covariance[0] = -1
        imu_pub.publish(imu_msg)

        temperature_msg =Temperature()
        temperature_msg.temperature = imu.ReadTemperature()
        temperature_msg.variance = imu.GetTempVariance()
        temperature_pub.publish(temperature_msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass