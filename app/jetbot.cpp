#include "camera.h"
#include "config.h"
#include "controller.h"
#include "drivetrain.h"
#include <chrono>
#include <thread>

int main() {
  controller ps4Controller;
  drivetrain robotDrivetrain;
  camera slamCam;
  auto begin(std::chrono::steady_clock::now());
  auto end(std::chrono::steady_clock::now());
  std::thread slamThread(&camera::record, &slamCam, CONFIG::CAMERA::RECORDTIME);
  while (end - begin < CONFIG::CAMERA::RECORDTIME) {
    end = std::chrono::steady_clock::now();
    auto controllerOutput = ps4Controller.pollOnce();
    if (controllerOutput.has_value()) {
      robotDrivetrain.leftMotor->run(controllerOutput.value().motorLeftSpeed);
      robotDrivetrain.rightMotor->run(controllerOutput.value().motorRightSpeed);
    }
    std::this_thread::sleep_for(CONFIG::MOTORCONTROLLER::SLEEP);
  }
  slamThread.join();
}
