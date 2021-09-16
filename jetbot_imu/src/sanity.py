#!/usr/bin/python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import adafruit_mpu6050
# import rospy


def main():
    mpu = adafruit_mpu6050.MPU6050(0, 0x68)

    while True:  # not rospy.is_shutdown():
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" %
              (mpu.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
        print("Temperature: %.2f C" % mpu.temperature)
        print("")
        time.sleep(0.5)


if __name__ == '__main__':
    main()
