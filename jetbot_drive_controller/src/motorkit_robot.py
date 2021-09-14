#!/usr/bin/python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

#
# NOTE - Only for use on Raspberry Pi or other SBC.
#

# Simple two DC motor robot class.  Exposes a simple LOGO turtle-like API for
# moving a robot forward, backward, and turning.  See RobotTest.py for an
# example of using this class.
# Author2: Tony DiCola, Chris Anderson
# License: MIT License https://opensource.org/licenses/MIT


# This assumes the Left motor is on Motor 1 and the Right motor is on Motor 2


import atexit
import board
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())


class Robot:
    def __init__(self, left_trim=0, right_trim=0, stop_at_exit=True):
        """Create an instance of the robot.  Can specify the following optional
        parameter
         - left_trim: Amount to offset the speed of the left motor, can be positive
                      or negative and use useful for matching the speed of both
                      motors.  Default is 0.
         - right_trim: Amount to offset the speed of the right motor (see above).
         - stop_at_exit: Boolean to indicate if the motors should stop on program
                         exit.  Default is True (highly recommended to keep this
                         value to prevent damage to the bot on program crash!).
        """

        self._left_trim = left_trim
        self._right_trim = right_trim
        if stop_at_exit:
            atexit.register(self.stop)

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self._left_trim
        # Constrain speed to 0-255 after trimming.
        speed = max(-1, min(1, speed))
        kit.motor1.throttle = speed

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset."""
        assert -1 <= speed <= 1, "Speed must be a value between -1 to 1 inclusive!"
        speed += self._right_trim
        # Constrain speed to 0-255 after trimming.
        speed = max(-1, min(1, speed))
        kit.motor2.throttle = speed

    @staticmethod
    def stop():
        """Stop all movement."""
        kit.motor1.throttle = 0
        kit.motor2.throttle = 0

    def linear(self, speed):
        """Move without rotation at the specified speed [-1 - 1]
        """
        self._left_speed(speed)
        self._right_speed(speed)

    def angular(self, speed):
        """Move angularly [-1 - 1]. +ve is ccw, -ve is cw,
        per typical cartesian coordinate system
        """
        self._left_speed(-speed)
        self._right_speed(speed)