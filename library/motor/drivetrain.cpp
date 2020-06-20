#include "config.h"
#include "drivetrain.h"

drivetrain::drivetrain() : _hat(){
    leftMotor = _hat.getMotor(CONFIG::MOTORS::MOTORLEFT);
    rightMotor = _hat.getMotor(CONFIG::MOTORS::MOTORRIGHT);
}

drivetrain::~drivetrain() {
    leftMotor->run(AdafruitDCMotor::kRelease);
    rightMotor->run(AdafruitDCMotor::kRelease);
}