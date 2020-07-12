#include "config.h"
#include "motor_controller.h"
#include <thread>

motor_controller::motor_controller() : _drivetrain(), _controller() {}

void motor_controller::run() {
    std::chrono::steady_clock::time_point begin(std::chrono::steady_clock::now()), end(std::chrono::steady_clock::now());
    while(std::chrono::duration_cast<std::chrono::seconds>(end - begin) < CONFIG::CAMERA::RECORDTIME) {
        end = std::chrono::steady_clock::now();
        auto ret = _controller.pollOnce();
        if(ret) {
            _drivetrain.leftMotor->run(ret.get().motorLeftSpeed);
            _drivetrain.rightMotor->run(ret.get().motorRightSpeed);
        }
        std::this_thread::sleep_for(CONFIG::MOTORCONTROLLER::SLEEP);
    }
}