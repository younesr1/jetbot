#include "camera.h"
#include "motor_controller.h"
#include <thread>
// TODO investigate robot not driving straight. Solutions: feedback control, make right wheel go slower than left wheel
// TODO investigate low responsiveness when controlling with ps4 controller. Potential reasons: Cable is too long. 3 threads is too much.
int main() {
    motor_controller motorController;
    std::thread motorControllerThread(&motor_controller::run, &motorController);
    camera slamCam;
    std::thread slamCamThread(&camera::record, &slamCam);
    slamCamThread.join();
    motorControllerThread.join();
}
