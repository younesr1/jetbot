#include "controller.h"
#include "drivetrain.h"
#include <thread>

int main() {
    controller ps4_controller;
    drivetrain motors;
    while(true) {
        auto ret = ps4_controller.pollOnce();
        if(ret) {
            motors.leftMotor->run(ret.get().motorLeftSpeed);
            motors.rightMotor->run(ret.get().motorRightSpeed);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}