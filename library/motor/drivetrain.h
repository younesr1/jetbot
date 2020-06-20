#pragma once
#include "adafruitdcmotor.h"
#include "adafruitmotorhat.h"

class drivetrain {
    private:
    AdafruitMotorHAT _hat;
    public:
    drivetrain();
    ~drivetrain();
    std::shared_ptr<AdafruitDCMotor> leftMotor;
    std::shared_ptr<AdafruitDCMotor> rightMotor;
};