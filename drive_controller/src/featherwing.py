#!/usr/bin/env python
from adafruit_motorkit import MotorKit


class FeatherWing:
    def __init__(self, max_speed):
        self.__drivetrain = MotorKit()
        self.__left = self.__drivetrain.motor1
        self.__right = self.__drivetrain.motor2
        self.__max_speed = max_speed

    # Speed [-max_speed, +max_speed]
    def SetLeftMotorSpeed(self, speed):
        speed = speed / self.__max_speed
        speed = min(1, max(-1, speed))
        self.__left.throttle(speed)

    # Speed [-max_speed, +max_speed]
    def SetRightMotorSpeed(self, speed):
        speed = speed / self.__max_speed
        speed = min(1, max(-1, speed))
        self.__right.throttle(speed)

    def __del__(self):
        self.StopLeftMotor()
        self.StopRightMotor()
