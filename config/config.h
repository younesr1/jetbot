#pragma once

namespace CONFIG {
    namespace MOTORS {
        enum {
            MOTORLEFT = 1,
            MOTORRIGHT = 2,
        };
        enum {
            DEADZONE = 5
        };
    }
    namespace CONTROLLER{
        enum {
            LEFTJSX = 0,
            LEFTJSY = 1,
            JS_EVENT_BUTTON = 1,
            JS_EVENT_AXIS = 2,
            R2 = 5,
            MAX_JS = 32767
        };
    }
}
