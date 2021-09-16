#!/usr/bin/python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import adafruit_mpu6050
# import rospy
import board

def main():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    mpu = adafruit_mpu6050.MPU6050(i2c) # connects to bus 1

    while True:  # not rospy.is_shutdown():
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" %
              (mpu.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
        print("Temperature: %.2f C" % mpu.temperature)
        print("")
        time.sleep(1/10.0)


if __name__ == '__main__':
    main()
