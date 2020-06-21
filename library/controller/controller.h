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

#pragma once

#include "boost/optional.hpp"
#include <string>
#include <iostream>

#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // controller moved
#define JS_EVENT_INIT   0x80 // initial state of device

/**
 * Represents a controller device. Allows data to be sampled from it.
 */
class controller {
private:
  struct controllerEvent {
    //static const int16_t MIN_AXES_VALUE = -32768;
    //static const int16_t MAX_AXES_VALUE = 32767;
    uint32_t time;
    short value;
    uint8_t type;
    uint8_t number;
    bool isAxis() {
      return (type & JS_EVENT_AXIS) != 0;
    }
    bool isInitialState() {
      return (type & JS_EVENT_INIT) != 0;
    }
    bool R2triggered() {
      return isAxis() && number == 5;
    }
    bool leftJStriggered() {
      return isAxis() && number == 0;
    }
  };
  struct motorSpeeds {
    int32_t motorLeftSpeed;
    int32_t motorRightSpeed;
  };
  int32_t _fd;
  int32_t _pf;
  int32_t _lr;
  int32_t _rr;
  controllerEvent _event;
  bool sample(controllerEvent* controllerEvent);
public:
  ~controller();
  controller();
  boost::optional<motorSpeeds> pollOnce();
};