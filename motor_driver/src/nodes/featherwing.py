#!/usr/bin/python3
from adafruit_motorkit import MotorKit
import time


class FeatherWing:
    def __init__(self):
        self.__drivetrain = MotorKit()
        self.__left = self.__drivetrain.motor1
        self.__right = self.__drivetrain.motor2

    # Speed [-100, 100]
    def SetLeftMotorSpeed(self, speed):
        speed = min(100, max(-100, speed))
        self.__left.throttle(speed / 100.0)

    # Speed [-100, 100]
    def SetRightMotorSpeed(self, speed):
        speed = min(100, max(-100, speed))
        self.__right.throttle(speed / 100.0)

    def StopLeftMotor(self):
        self.SetLeftMotorSpeed(0)

    def StopRightMotor(self):
        self.SetRightMotorSpeed(0)

    def __del__(self):
        self.StopLeftMotor()
        self.StopRightMotor()
