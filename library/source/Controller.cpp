#include "Controller.h"
#include <fcntl.h>
#include <unistd.h>
#include <string>

controller::controller() {
    std::string devicePath = "/dev/input/js0";
    _fd = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
    if(_fd == -1) perror("Fatal: Unable to open joystick file");
}

controller::~controller() {
    close(_fd);
}

bool controller::sample()
{
  int bytes = read(_fd, &_event, sizeof(_event)); 
  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // controller instance is likely unusable
  return bytes == sizeof(_event);
}

void controller::run() {
    while(true) {
        // Restrict rate
        usleep(1000);
        // only act if there's new data
        if(sample()) {
            if(R2triggered()) {
                // pf[0,100]
                _pf = ((_event.value + MAX_JS) / (MAX_JS*2.0))*100;
            }
            else if(leftJStriggered()) {
                // temp[-100,100]
                int8_t temp = (((_event.value + MAX_JS) / (MAX_JS*2.0))*200)-100;
                _lr = temp < 0 ? 100 + temp : 100;
                _rr = temp > 0 ? 100 - temp : 100; 
            }
        }
    }
}

bool controller::R2triggered() {
    if(_event.type & JS_EVENT_AXIS)
        return _event.number == R2;
}

bool controller::leftJStriggered() {
    if(_event.type & JS_EVENT_AXIS)
        return _event.number == LEFTJSX; // || _event.number == LEFTJSY) {
}