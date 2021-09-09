# Simple two DC motor robot class.  Exposes a simple LOGO turtle-like API for
# moving a robot forward, backward, and turning.  See RobotTest.py for an
# example of using this class.
# Author: Tony DiCola
# License: MIT License https://opensource.org/licenses/MIT
import atexit

from Adafruit_MotorHAT import Adafruit_MotorHAT


class Robot(object):
    def __init__(self, addr=0x60, bus_num=1, left_id=1, right_id=2, left_trim=0, right_trim=0,
                 stop_at_exit=True):
        """Create an instance of the robot.  Can specify the following optional
        parameters:
         - addr: The I2C address of the motor HAT, default is 0x60.
         - left_id: The ID of the left motor, default is 1.
         - right_id: The ID of the right motor, default is 2.
         - left_trim: Amount to offset the speed of the left motor, can be positive
                      or negative and use useful for matching the speed of both
                      motors.  Default is 0.
         - right_trim: Amount to offset the speed of the right motor (see above).
         - stop_at_exit: Boolean to indicate if the motors should stop on program
                         exit.  Default is True (highly recommended to keep this
                         value to prevent damage to the bot on program crash!).
        """
        # Initialize motor HAT and left, right motor.
        self._mh = Adafruit_MotorHAT(addr=addr, i2c_bus=bus_num)
        self._left = self._mh.getMotor(left_id)
        self._right = self._mh.getMotor(right_id)
        self._left_trim = left_trim
        self._right_trim = right_trim
        # Start with motors turned off.
        self._left.run(Adafruit_MotorHAT.RELEASE)
        self._right.run(Adafruit_MotorHAT.RELEASE)
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._left_trim
        # Constrain speed to 0-255 after trimming.
        speed = max(0, min(255, speed))
        self._left.setSpeed(speed)

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._right_trim
        # Constrain speed to 0-255 after trimming.
        speed = max(0, min(255, speed))
        self._right.setSpeed(speed)

    def stop(self):
        """Stop all movement."""
        self._left.run(Adafruit_MotorHAT.RELEASE)
        self._right.run(Adafruit_MotorHAT.RELEASE)

    def linear(self, speed):
        """Move without rotation at the specified speed [-1 - 1]
        """
        speed = int(abs(speed*255))
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        dir = Adafruit_MotorHAT.FORWARD if speed > 0 else Adafruit_MotorHAT.BACKWARD
        self._left.run(dir)
        self._right.run(dir)

    def angular(self, speed):
        """Move angularly [-1 - 1]. +ve is ccw, -ve is cw,
        per typical cartesian coordinate system
        """
        speed = int(abs(speed*255))
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        left_dir = Adafruit_MotorHAT.BACKWARD if speed > 0 else Adafruit_MotorHAT.FORWARD
        right_dir = Adafruit_MotorHAT.BACKWARD if speed < 0 else Adafruit_MotorHAT.FORWARD
        self._left.run(left_dir)
        self._right.run(right_dir)
