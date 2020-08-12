// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "controller.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include "unistd.h"

controller::controller() : _pf(0), _lr(100), _rr(100)
{
  std::string devicePath = "/dev/input/js0";
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
  if(!(_fd >= 0)) {
    printf("open failed.\n");
    exit(1);
  }
}

bool controller::sample(controllerEvent* controllerEvent)
{
  int bytes = read(_fd, controllerEvent, sizeof(*controllerEvent)); 
  if (bytes == -1)
    return false;
  return bytes == sizeof(*controllerEvent);
}

std::optional<controller::motorSpeeds> controller::pollOnce() {
  bool newData = false;
  if (sample(&_event))
  {
      if(_event.R2triggered() && !_event.isInitialState()) {
        _pf = ((_event.value / 32767.0) + 1) * 25;
        newData = true;
      }
      else if(_event.leftJStriggered() && !_event.isInitialState()) {
        int8_t temp = 100 * _event.value / 32767.0;
        _lr = temp < 0 ? 100 + temp : 100;
        _rr = temp > 0 ? 100 - temp : 100;
        newData = true;
      }
      else if(_event.L2triggered() && !_event.isInitialState()) {
        _pf = ((_event.value / 32767.0) + 1) * 50 * -1;
        newData = true;
      }
  }
  if(newData) {
    controller::motorSpeeds ret = {_lr*_pf/100, _rr*_pf/100};
    return ret;
  }
  return std::nullopt;
}

controller::~controller() {
  close(_fd);
}