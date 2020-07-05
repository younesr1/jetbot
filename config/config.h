#pragma once
#include <cstdint>

namespace CONFIG {
    namespace MOTORS {
        constexpr uint8_t DEADZONE = 5;
        enum {
            MOTORLEFT = 1,
            MOTORRIGHT = 2,
        };
    }
    namespace CONTROLLER{
        enum {
            LEFTJSX = 0,
            LEFTJSY = 1,
            JS_EVENT_AXIS = 0x02,
            JS_EVENT_INIT = 0x80,
            L2 = 3,
            R2 = 4,
            MAX_JS = 32767
        };
    }
}