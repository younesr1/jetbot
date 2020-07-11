#include "camera.h"
#include "motor_controller.h"
#include <thread>

int main() {
    motor_controller motorController;
    std::thread motorControllerThread(&motor_controller::run, &motorController);
    camera slamCam;
    std::thread slamCamThread(&camera::record, &slamCam);

    slamCamThread.join();
    motorControllerThread.join();
}
