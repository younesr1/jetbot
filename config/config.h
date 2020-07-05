#pragma once
#include <cstdint>
#include <string>

namespace CONFIG {
    namespace MOTORS {
        constexpr uint8_t DEADZONE = 5;
        enum {
            MOTORLEFT = 1,
            MOTORRIGHT = 2,
        };
    }
    namespace CONTROLLER {
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
    namespace CAMERA {
        const std::string WINDOWNAME = "SLAM";
        constexpr uint16_t WIDTH = 1280;
        constexpr uint16_t HEIGHT = 720;
        constexpr uint8_t FPS = 60;
        constexpr bool FLIP = false;

    }
}