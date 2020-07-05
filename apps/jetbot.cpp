#include "camera.h"
#include "controller.h"
#include "drivetrain.h"
#include <thread>

//static bool quit = false;
//void waitForQuit();
int main() {
    // controller ps4_controller;
    // drivetrain motors;
    camera slam_cam;
    slam_cam.record();
    // std::thread endProgram(waitForQuit);
    /*while(true) {
        auto ret = ps4_controller.pollOnce();
        if(ret) {
            motors.leftMotor->run(ret.get().motorLeftSpeed);
            motors.rightMotor->run(ret.get().motorRightSpeed);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }*/
    //endProgram.join();
}

/*void waitForQuit() {
    std::cin.get();
    quit = true;
}*/