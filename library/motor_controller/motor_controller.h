#pragma once

#include "drivetrain.h"
#include "controller.h"

class motor_controller {
private:
    drivetrain _drivetrain;
    controller _controller;
public:
    motor_controller();
    void run();
};