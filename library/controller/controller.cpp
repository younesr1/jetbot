#include "controller.h"
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

bool controller::sample() {
  int bytes = read(_fd, &_event, sizeof(_event)); 
  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // controller instance is likely unusable
  return bytes == sizeof(_event);
}

boost::optional<controller::motorSpeeds> controller::pollOnce() {
    bool newData = false;
    // only act if there's new data
    if(sample()) {
        if(R2triggered()) {
            // pf[0,100]
            _pf = ((_event.value / CONFIG::CONTROLLER::MAX_JS) + 1) * 50;
            newData = true;
        }
        if(leftJStriggered()) {
            // temp[-100,100]
            int8_t temp = 100 * _event.value / CONFIG::CONTROLLER::MAX_JS;
            _lr = temp < 0 ? 100 + temp : 100;
            _rr = temp > 0 ? 100 - temp : 100;
            newData = true;
        }
    }
   controller::motorSpeeds ret = {_lr*_pf/100, _rr*_pf/100};
    if(newData)
        return ret;
    return boost::none;
}

bool controller::R2triggered() {
    if(_event.type & CONFIG::CONTROLLER::JS_EVENT_AXIS)
        return _event.number == CONFIG::CONTROLLER::R2;
}

bool controller::leftJStriggered() {
    if(_event.type & CONFIG::CONTROLLER::JS_EVENT_AXIS)
        return _event.number == CONFIG::CONTROLLER::LEFTJSX;
}